// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2021-2024 NXP
 */
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_edid.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <sound/hdmi-codec.h>

#include "it6161.h"

#define AUX_WAIT_TIMEOUT_MS 100
#define DEFAULT_DRV_HOLD 0

#define RGB_24b         0x3E
#define RGB_18b         0x1E

#define InvMCLK		TRUE
#define PDREFCLK	FALSE
#define SkipStg		4
#define MShift		8
#define PPSFFRdStg	0x04
#define RegAutoSync	TRUE

#define LMDbgSel		0   /* 0~7 */
#define InvPCLK			FALSE
#define PDREFCNT		0   /* when PDREFCLK=TRUE, 0:div2, 1:div4, 2:div8, 3:divg16 */
#define EnIOIDDQ		FALSE
#define EnStb2Rst		FALSE
#define EnExtStdby		FALSE
#define EnStandby		FALSE
#define MPLaneSwap		FALSE
#define MPPNSwap		FALSE   /* TRUE: MTK , FALSE: Solomon */

/* PPI */
#define EnContCK		TRUE
#define HSSetNum		3
#define EnDeSkew		TRUE
#define PPIDbgSel		12
#define RegIgnrNull		1
#define RegIgnrBlk		1
#define RegEnDummyECC	0
#define EOTPSel			0

/* PPS option */
#define EnMBPM			FALSE   /* enable MIPI Bypass Mode */
#if (EnMBPM == TRUE)
	#define PREC_Update	TRUE   /* enable P-timing update */
	#define MREC_Update	TRUE   /* enable M-timing update */
	#define EnTBPM		TRUE   /* enable HDMITX Bypass Mode */
#else
	#define PREC_Update	FALSE
	#define MREC_Update	FALSE
	#define EnTBPM		FALSE
#endif

#define REGSELDEF		FALSE
#define EnHReSync		FALSE
#define EnVReSync		FALSE
#define EnFReSync		FALSE
#define EnVREnh			FALSE
#define EnVREnhSel		1   /* 0:Div2, 1:Div4, 2:Div8, 3:Div16, 4:Div32 */
#define EnMAvg			TRUE

#define PShift			3
#define EnFFAutoRst		TRUE
#define RegEnSyncErr	FALSE
#define EnTxCRC			TRUE
#define TxCRCnum		0x20

#define ENABLE_MIPI_RX_EXTERNAL_CLOCK FALSE

#define NRTXRCLK		TRUE   /* true:set TRCLK by self */
#define RCLKFreqSel		TRUE   /* false: 10MHz(div1); true: 20 MHz(OSSDIV2) */
#define ForceTxCLKStb	TRUE

#define HDMI_TX_PCLK_DIV2			FALSE

#define HDMI_TX_MODE				HDMI_TX_ENABLE_DE_ONLY

enum hdmi_tx_mode {
	HDMI_TX_NONE,
	HDMI_TX_BY_PASS,
	HDMI_TX_ENABLE_DE_ONLY,
	HDMI_TX_ENABLE_PATTERN_GENERATOR,
};

enum it6161_active_level {
	LOW,
	HIGH,
};

const struct RegSetEntry HDMITX_Init_Table[] = {
	{0x0F, 0x40, 0x00},
	/*PLL Reset */
	{0x62, 0x08, 0x00},	/* XP_RESETB */
	{0x64, 0x04, 0x00},	/* IP_RESETB */
	{0x0F, 0x01, 0x00},	/* bank 0 ;3 */
	{0x8D, 0xFF, CEC_I2C_SLAVE_ADDR},	/* EnCEC */
	{0xA9, 0x80, (EnTBPM << 7)},
	{0xBF, 0x80, (NRTXRCLK << 7)},

	/* Initial Value */
	{0xF8, 0xFF, 0xC3},
	{0xF8, 0xFF, 0xA5},
	{0xF4, 0x0C, 0x00},
	{0xF3, 0x02, 0x00},
	{0xF8, 0xFF, 0xFF},
	{0x5A, 0x0C, 0x0C},
	{0xD1, 0x0A, ((ForceTxCLKStb) << 3) + 0x02},
	{0x5D, 0x04, ((RCLKFreqSel) << 2)},
	{0x65, 0x03, 0x00},
	{0x71, 0xF9, 0x18},
	{0xCF, 0xFF, 0x00},
	{0xd1, 0x02, 0x00},
	{0x59, 0xD0, 0x40},
	{0xE1, 0x20, 0x00},
	{0xF5, 0x40, 0x00},
	{0x05, 0xC0, 0x40},	/* Setup INT Pin: Active Low & Open-Drain */
	{0x0C, 0xFF, 0xFF},
	{0x0D, 0xFF, 0xFF},
	{0x0E, 0x03, 0x03},	/* Clear all Interrupt */
	{0x0C, 0xFF, 0x00},
	{0x0D, 0xFF, 0x00},
	{0x0E, 0x02, 0x00},
	{0x20, 0x01, 0x00}
};

const struct RegSetEntry HDMITX_DefaultVideo_Table[] = {
	/* Config default output format */
	{0x72, 0xff, 0x00},
	{0x70, 0xff, 0x00},
/* GenCSC\RGB2YUV_ITU709_16_235 */
	{0x72, 0xFF, 0x02},
	{0x73, 0xFF, 0x00},
	{0x74, 0xFF, 0x80},
	{0x75, 0xFF, 0x00},
	{0x76, 0xFF, 0xB8},
	{0x77, 0xFF, 0x05},
	{0x78, 0xFF, 0xB4},
	{0x79, 0xFF, 0x01},
	{0x7A, 0xFF, 0x93},
	{0x7B, 0xFF, 0x00},
	{0x7C, 0xFF, 0x49},
	{0x7D, 0xFF, 0x3C},
	{0x7E, 0xFF, 0x18},
	{0x7F, 0xFF, 0x04},
	{0x80, 0xFF, 0x9F},
	{0x81, 0xFF, 0x3F},
	{0x82, 0xFF, 0xD9},
	{0x83, 0xFF, 0x3C},
	{0x84, 0xFF, 0x10},
	{0x85, 0xFF, 0x3F},
	{0x86, 0xFF, 0x18},
	{0x87, 0xFF, 0x04},
	{0x88, 0xF0, 0x00},
};

/* Config default HDMI Mode */
const struct RegSetEntry HDMITX_SetHDMI_Table[] = {
	{0xC0, 0x01, 0x01},
	{0xC1, 0x03, 0x03},
	{0xC6, 0x03, 0x03}
};

/* Config default avi infoframe */
const struct RegSetEntry HDMITX_DefaultAVIInfo_Table[] = {
	{0x0F, 0x01, 0x01},
	{0x58, 0xFF, 0x10},
	{0x59, 0xFF, 0x08},
	{0x5A, 0xFF, 0x00},
	{0x5B, 0xFF, 0x00},
	{0x5C, 0xFF, 0x00},
	{0x5D, 0xFF, 0x57},
	{0x5E, 0xFF, 0x00},
	{0x5F, 0xFF, 0x00},
	{0x60, 0xFF, 0x00},
	{0x61, 0xFF, 0x00},
	{0x62, 0xFF, 0x00},
	{0x63, 0xFF, 0x00},
	{0x64, 0xFF, 0x00},
	{0x65, 0xFF, 0x00},
	{0x0F, 0x01, 0x00},
	{0xCD, 0x03, 0x03}
};

/* Config default audio infoframe */
const struct RegSetEntry HDMITX_DeaultAudioInfo_Table[] = {
	{0x0F, 0x01, 0x01},
	{0x68, 0xFF, 0x00},
	{0x69, 0xFF, 0x00},
	{0x6A, 0xFF, 0x00},
	{0x6B, 0xFF, 0x00},
	{0x6C, 0xFF, 0x00},
	{0x6D, 0xFF, 0x71},
	{0x0F, 0x01, 0x00},
	{0xCE, 0x03, 0x03}
};

const struct RegSetEntry HDMITX_Aud_CHStatus_LPCM_20bit_48Khz[] = {
	{0x0F, 0x01, 0x01},
	{0x33, 0xFF, 0x00},
	{0x34, 0xFF, 0x18},
	{0x35, 0xFF, 0x00},
	{0x91, 0xFF, 0x00},
	{0x92, 0xFF, 0x00},
	{0x93, 0xFF, 0x01},
	{0x94, 0xFF, 0x00},
	{0x98, 0xFF, 0x02},
	{0x99, 0xFF, 0xDA},
	{0x0F, 0x01, 0x00}
};

const struct RegSetEntry HDMITX_AUD_SPDIF_2ch_24bit[] = {
	{0x0F, 0x11, 0x00},
	{0x04, 0x14, 0x04},
	{0xE0, 0xFF, 0xD1},
	{0xE1, 0xFF, 0x01},
	{0xE2, 0xFF, 0xE4},
	{0xE3, 0xFF, 0x10},
	{0xE4, 0xFF, 0x00},
	{0xE5, 0xFF, 0x00},
	{0x04, 0x14, 0x00}
};

const struct RegSetEntry HDMITX_PwrOn_Table[] = {
	/* PwrOn RCLK , IACLK ,TXCLK */
	{0x0F, 0x70, 0x00},
	/* PLL PwrOn */
	/* PwrOn DRV */
	{0x61, 0x20, 0x00},
	/* PwrOn XPLL */
	{0x62, 0x44, 0x00},
	/* PwrOn IPLL */
	{0x64, 0x40, 0x00},
	/* PLL Reset OFF */
	/* DRV_RST */
	{0x61, 0x10, 0x00},
	/* XP_RESETB */
	{0x62, 0x08, 0x08},
	/* IP_RESETB */
	{0x64, 0x04, 0x04}
};

struct it6161 {
	struct drm_bridge bridge;
	struct drm_connector connector;
	struct i2c_client *i2c_mipi_rx;
	struct i2c_client *i2c_hdmi_tx;
	struct device_node *host_node;
	struct mipi_dsi_device *dsi;
	struct mutex mode_lock;

	struct regmap *regmap_mipi_rx;
	struct regmap *regmap_hdmi_tx;

	u32 it6161_addr_hdmi_tx;

	struct gpio_desc *enable_gpio;

	u32 hdmi_tx_pclk;
	u32 mipi_rx_rclk;

	/* video mode output to hdmi tx */
	struct drm_display_mode display_mode;
	struct hdmi_avi_infoframe source_avi_infoframe;

	u8 mipi_rx_lane_count;
	bool enable_drv_hold;
	u8 hdmi_tx_output_color_space;
	u8 hdmi_tx_input_color_space;
	u8 hdmi_tx_mode;
	u8 support_audio;
	bool hdmi_mode;
	u8 bAudioChannelEnable;
	u8 bridge_enable;
};

struct it6161 *it6161;

static const struct regmap_range it6161_mipi_rx_bridge_volatile_ranges[] = {
	{.range_min = 0, .range_max = 0xFF},
};

static const struct regmap_access_table it6161_mipi_rx_bridge_volatile_table = {
	.yes_ranges = it6161_mipi_rx_bridge_volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(it6161_mipi_rx_bridge_volatile_ranges),
};

static const struct regmap_config it6161_mipi_rx_bridge_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.volatile_table = &it6161_mipi_rx_bridge_volatile_table,
	.cache_type = REGCACHE_NONE,
};

static const struct regmap_range it6161_hdmi_tx_bridge_volatile_ranges[] = {
	{.range_min = 0, .range_max = 0xFF},
};

static const struct regmap_access_table it6161_hdmi_tx_bridge_volatile_table = {
	.yes_ranges = it6161_hdmi_tx_bridge_volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(it6161_hdmi_tx_bridge_volatile_ranges),
};

static const struct regmap_config it6161_hdmi_tx_bridge_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.volatile_table = &it6161_hdmi_tx_bridge_volatile_table,
	.cache_type = REGCACHE_NONE,
};

static int it6161_mipi_rx_read(struct it6161 *it6161, u32 reg_addr)
{
	struct device *dev = &it6161->i2c_mipi_rx->dev;
	u32 value;
	int err;

	err = regmap_read(it6161->regmap_mipi_rx, reg_addr, &value);
	if (err < 0) {
		DRM_DEV_ERROR(dev, "mipi rx read failed reg[0x%x] err: %d", reg_addr, err);
		return err;
	}

	return value;
}

static int it6161_mipi_rx_write(struct it6161 *it6161, u32 addr, u32 val)
{
	struct device *dev = &it6161->i2c_mipi_rx->dev;
	int err;

	err = regmap_write(it6161->regmap_mipi_rx, addr, val);
	if (err < 0) {
		DRM_DEV_ERROR(dev, "mipi rx write failed reg[0x%x] = 0x%x err = %d",
			      addr, val, err);
		return err;
	}

	return 0;
}

static int it6161_mipi_rx_set_bits(struct it6161 *it6161, u32 reg,
				   u32 mask, u32 value)
{
	struct device *dev = &it6161->i2c_mipi_rx->dev;
	int err;

	err = regmap_update_bits(it6161->regmap_mipi_rx, reg, mask, value);
	if (err < 0) {
		DRM_DEV_ERROR(dev, "mipi rx set reg[0x%x] = 0x%x mask = 0x%x failed err %d",
			      reg, value, mask, err);
		return err;
	}

	return 0;
}

static int it6161_hdmi_tx_read(struct it6161 *it6161, u32 reg_addr)
{
	struct device *dev = &it6161->i2c_mipi_rx->dev;
	u32 value;
	int err;

	err = regmap_read(it6161->regmap_hdmi_tx, reg_addr, &value);
	if (err < 0) {
		DRM_DEV_ERROR(dev, "hdmi tx read failed reg[0x%x] err: %d",
			      reg_addr, err);
		return err;
	}

	return value;
}

static int it6161_hdmi_tx_write(struct it6161 *it6161, u32 addr, u32 val)
{
	struct device *dev = &it6161->i2c_mipi_rx->dev;
	int err;

	err = regmap_write(it6161->regmap_hdmi_tx, addr, val);

	if (err < 0) {
		DRM_DEV_ERROR(dev, "hdmi tx write failed reg[0x%x] = 0x%x err = %d",
			      addr, val, err);
		return err;
	}

	return 0;
}

static int it6161_hdmi_tx_set_bits(struct it6161 *it6161, u32 reg,
				   u32 mask, u32 value)
{
	int err;
	struct device *dev = &it6161->i2c_mipi_rx->dev;

	err = regmap_update_bits(it6161->regmap_hdmi_tx, reg, mask, value);
	if (err < 0) {
		DRM_DEV_ERROR(dev, "hdmi tx set reg[0x%x] = 0x%x mask = 0x%x failed err %d",
			      reg, value, mask, err);
		return err;
	}

	return 0;
}

static inline int it6161_hdmi_tx_change_bank(struct it6161 *it6161, int x)
{
	return it6161_hdmi_tx_set_bits(it6161, 0x0F, 0x03, x & 0x03);
}

static inline struct it6161 *connector_to_it6161(struct drm_connector *c)
{
	return container_of(c, struct it6161, connector);
}

static inline struct it6161 *bridge_to_it6161(struct drm_bridge *bridge)
{
	return container_of(bridge, struct it6161, bridge);
}

static void mipi_rx_logic_reset(struct it6161 *it6161)
{
	it6161_mipi_rx_set_bits(it6161, 0x05, 0x08, 0x08);
}

static void mipi_rx_logic_reset_release(struct it6161 *it6161)
{
	it6161_mipi_rx_set_bits(it6161, 0x05, 0x08, 0x00);
}

static void it6161_mipi_rx_int_mask_disable(struct it6161 *it6161)
{
	it6161_mipi_rx_set_bits(it6161, 0x0F, 0x03, 0x00);
	it6161_mipi_rx_write(it6161, 0x09, 0x00);
	it6161_mipi_rx_write(it6161, 0x0A, 0x00);
	it6161_mipi_rx_write(it6161, 0x0B, 0x00);
}

static void it6161_mipi_rx_int_mask_enable(struct it6161 *it6161)
{
	it6161_hdmi_tx_set_bits(it6161, 0x0F, 0x03, 0x00);
	it6161_mipi_rx_write(it6161, 0x09, 0x11);
	it6161_mipi_rx_write(it6161, 0x0A, 0xc0);
	it6161_mipi_rx_write(it6161, 0x0B, 0x40);
}

static void it6161_hdmi_tx_int_mask_disable(struct it6161 *it6161)
{
	it6161_mipi_rx_set_bits(it6161, 0x0F, 0x03, 0x00);
	it6161_hdmi_tx_write(it6161, REG_TX_INT_MASK1, 0xFF);
	it6161_hdmi_tx_write(it6161, REG_TX_INT_MASK3, 0xFF);
}

static void it6161_hdmi_tx_int_mask_enable(struct it6161 *it6161)
{
	it6161_hdmi_tx_set_bits(it6161, 0x0F, 0x03, 0x00);
	it6161_hdmi_tx_write(it6161, REG_TX_INT_MASK1,
				~(B_TX_AUDIO_OVFLW_MASK | B_TX_DDC_FIFO_ERR_MASK |
				B_TX_DDC_BUS_HANG_MASK | B_TX_HPD_MASK));
	it6161_hdmi_tx_write(it6161, REG_TX_INT_MASK3, ~B_TX_VIDSTABLE_MASK);
}

static void it6161_hdmi_tx_write_table(struct it6161 *it6161,
				const struct RegSetEntry table[], int size)
{
	int i;

	for (i = 0; i < size; i++) {
		if (table[i].mask == 0 && table[i].value == 0)
			msleep(table[i].offset);
		else if (table[i].mask == 0xFF)
			it6161_hdmi_tx_write(it6161, table[i].offset, table[i].value);
		else
			it6161_hdmi_tx_set_bits(it6161, table[i].offset, table[i].mask, table[i].value);
	}
}

static inline void it6161_set_interrupts_active_level(enum it6161_active_level level)
{
	it6161_mipi_rx_set_bits(it6161, 0x0D, 0x02, level == HIGH ? 0x02 : 0x00);
	it6161_hdmi_tx_set_bits(it6161, 0x05, 0xC0,	level == HIGH ? 0x80 : 0x40);
}

static void hdmi_tx_init(struct it6161 *it6161)
{
	struct device *dev = &it6161->i2c_hdmi_tx->dev;

	DRM_DEV_DEBUG_DRIVER(dev, "it6161 init\n");

	it6161_hdmi_tx_write_table(it6161, HDMITX_Init_Table,
				   ARRAY_SIZE(HDMITX_Init_Table));
	it6161_hdmi_tx_write_table(it6161, HDMITX_PwrOn_Table,
				   ARRAY_SIZE(HDMITX_PwrOn_Table));
	it6161_hdmi_tx_write_table(it6161, HDMITX_DefaultVideo_Table,
				   ARRAY_SIZE(HDMITX_DefaultVideo_Table));
	it6161_hdmi_tx_write_table(it6161, HDMITX_SetHDMI_Table,
				   ARRAY_SIZE(HDMITX_SetHDMI_Table));
	it6161_hdmi_tx_write_table(it6161, HDMITX_DefaultAVIInfo_Table,
				   ARRAY_SIZE(HDMITX_DefaultAVIInfo_Table));
	it6161_hdmi_tx_write_table(it6161, HDMITX_DeaultAudioInfo_Table,
				   ARRAY_SIZE(HDMITX_DeaultAudioInfo_Table));
	it6161_hdmi_tx_write_table(it6161, HDMITX_Aud_CHStatus_LPCM_20bit_48Khz,
				   ARRAY_SIZE(HDMITX_Aud_CHStatus_LPCM_20bit_48Khz));
	it6161_hdmi_tx_write_table(it6161, HDMITX_AUD_SPDIF_2ch_24bit,
				   ARRAY_SIZE(HDMITX_AUD_SPDIF_2ch_24bit));
}

static bool mipi_rx_get_m_video_stable(struct it6161 *it6161)
{
	return it6161_mipi_rx_read(it6161, 0x0D) & 0x10;
}

static bool mipi_rx_get_p_video_stable(struct it6161 *it6161)
{
	return it6161_mipi_rx_read(it6161, 0x0D) & 0x20;
}

static void mipi_rx_afe_configuration(struct it6161 *it6161, u8 data_id)
{
	struct device *dev = &it6161->i2c_hdmi_tx->dev;
	u8 MPLaneNum = (it6161->mipi_rx_lane_count - 1);

	DRM_DEV_DEBUG_DRIVER(dev, "afe configuration data_id: 0x%02x", data_id);

	if (data_id == RGB_18b) {
		if (MPLaneNum == 3)
			/* MPPCLKSel = 1; 4-lane : MCLK = 1/1 PCLK */
			it6161_mipi_rx_set_bits(it6161, 0x80, 0x1F, 0x02);
		else if (MPLaneNum == 1)
			/* MPPCLKSel = 6; 2-lane : MCLK = 1/1 PCLK */
			it6161_mipi_rx_set_bits(it6161, 0x80, 0x1F, 0x05);
		else if (MPLaneNum == 0)
			/* MPPCLKSel = 8; 1-lane : MCLK = 3/4 PCLK */
			it6161_mipi_rx_set_bits(it6161, 0x80, 0x1F, 0x08);
	} else {
		if (MPLaneNum == 3)
			/* MPPCLKSel = 1; 4-lane : MCLK = 3/4 PCLK */
			it6161_mipi_rx_set_bits(it6161, 0x80, 0x1F, 0x02);
		else if (MPLaneNum == 1)
			/* MPPCLKSel = 3; 2-lane : MCLK = 3/4 PCLK */
			it6161_mipi_rx_set_bits(it6161, 0x80, 0x1F, 0x05);
		else if (MPLaneNum == 0)
			/* MPPCLKSel = 5; 1-lane : MCLK = 3/4 PCLK */
			it6161_mipi_rx_set_bits(it6161, 0x80, 0x1F, 0x0b);
	}
}

static void mipi_rx_configuration(struct it6161 *it6161)
{
	struct device *dev = &it6161->i2c_hdmi_tx->dev;
	u8 mipi_lane_config = (it6161->mipi_rx_lane_count - 1);

	DRM_DEV_DEBUG_DRIVER(dev, "MIPI_LANE=%d\n", it6161->mipi_rx_lane_count);

	it6161_mipi_rx_set_bits(it6161, 0x10, 0x0F, 0x0F);
	msleep(1);
	it6161_mipi_rx_set_bits(it6161, 0x10, 0x0F, 0x00);

	mipi_rx_logic_reset(it6161);
	msleep(1);
	mipi_rx_logic_reset_release(it6161);

	it6161_mipi_rx_int_mask_disable(it6161);

	/* setup INT pin: active low */
	it6161_mipi_rx_set_bits(it6161, 0x0d, 0x02, 0x00);

	it6161_mipi_rx_set_bits(it6161, 0x0C, 0x0F,
				(MPLaneSwap << 3) + (MPPNSwap << 2) + mipi_lane_config);

	it6161_mipi_rx_set_bits(it6161, 0x11, 0x3F,
				(EnIOIDDQ << 5) + (EnStb2Rst << 4) + (EnExtStdby << 3) +
				(EnStandby << 2) + (InvPCLK << 1) + InvMCLK);

	it6161_mipi_rx_set_bits(it6161, 0x12, 0x03, (PDREFCNT << 1) + PDREFCLK);

	it6161_mipi_rx_set_bits(it6161, 0x18, 0xf7,
				(RegEnSyncErr << 7) + (SkipStg << 4) + HSSetNum);
	it6161_mipi_rx_set_bits(it6161, 0x19, 0xf3,
				(PPIDbgSel << 4) + (EnContCK << 1) + EnDeSkew);
	it6161_mipi_rx_set_bits(it6161, 0x20, 0xf7,
				(EOTPSel << 4) + (RegEnDummyECC << 2) +	(RegIgnrBlk << 1) + RegIgnrNull);
	it6161_mipi_rx_set_bits(it6161, 0x21, 0x07, LMDbgSel);

	it6161_mipi_rx_set_bits(it6161, 0x44, 0x3a,
			(MREC_Update << 5) + (PREC_Update << 4) + (REGSELDEF << 3) + (RegAutoSync << 1));
	it6161_mipi_rx_set_bits(it6161, 0x4B, 0x1f,
				(EnFReSync << 4) + (EnVREnh << 3) + EnVREnhSel);
	it6161_mipi_rx_write(it6161, 0x4C, PPSFFRdStg);
	it6161_mipi_rx_set_bits(it6161, 0x4D, 0x01, (PPSFFRdStg >> 8) & 0x01);
	it6161_mipi_rx_set_bits(it6161, 0x4E, 0x0C,
				(EnVReSync << 3) + (EnHReSync << 2));
	it6161_mipi_rx_set_bits(it6161, 0x4F, 0x03, EnFFAutoRst);

	it6161_mipi_rx_set_bits(it6161, 0x70, 0x01, EnMAvg);
	it6161_mipi_rx_write(it6161, 0x72, MShift);
	it6161_mipi_rx_write(it6161, 0x73, PShift);
	it6161_mipi_rx_set_bits(it6161, 0x80, 0x20, ENABLE_MIPI_RX_EXTERNAL_CLOCK << 5);

	it6161_mipi_rx_write(it6161, 0x21, 0x00);
	it6161_mipi_rx_set_bits(it6161, 0x84, 0x70, 0x00);

	it6161_mipi_rx_set_bits(it6161, 0xA0, 0x01, EnMBPM);

	/* enable auto detect format */
	it6161_mipi_rx_set_bits(it6161, 0x21, 0x08, 0x08);

	it6161_mipi_rx_set_bits(it6161, 0x70, 0x01, EnMAvg);
	/* Video Clock Domain Reset */
	it6161_mipi_rx_set_bits(it6161, 0x05, 0x02, 0x02);

	if (EnMBPM) {
		/* HRS offset  */
		it6161_mipi_rx_write(it6161, 0xA1, 0x00);
		/* VRS offset  */
		it6161_mipi_rx_write(it6161, 0xA2, 0x00);
		it6161_mipi_rx_write(it6161, 0xA3, 0x08);
		it6161_mipi_rx_write(it6161, 0xA5, 0x04);
	}

	if (REGSELDEF == false) {
		it6161_mipi_rx_set_bits(it6161, 0x31, 0x80, 0x00);
		it6161_mipi_rx_set_bits(it6161, 0x33, 0x80, 0x00);
		it6161_mipi_rx_set_bits(it6161, 0x35, 0x80, 0x00);
		it6161_mipi_rx_set_bits(it6161, 0x37, 0x80, 0x00);
		it6161_mipi_rx_set_bits(it6161, 0x39, 0x80, 0x00);
		it6161_mipi_rx_set_bits(it6161, 0x3A, 0x80, 0x00);
		it6161_mipi_rx_set_bits(it6161, 0x3C, 0x80, 0x00);
		it6161_mipi_rx_set_bits(it6161, 0x3E, 0x80, 0x00);
		it6161_mipi_rx_set_bits(it6161, 0x41, 0x80, 0x00);
		it6161_mipi_rx_set_bits(it6161, 0x43, 0x80, 0x00);
	}
}

static void mipi_rx_init(struct it6161 *it6161)
{
	mipi_rx_configuration(it6161);
	/* Enable MPRX clock domain */
	it6161_mipi_rx_set_bits(it6161, 0x05, 0x03, 0x00);
}

static void hdmi_tx_video_reset(struct it6161 *it6161)
{
	struct device *dev = &it6161->i2c_hdmi_tx->dev;

	DRM_DEV_DEBUG_DRIVER(dev, "reg04: 0x%02x reg05: 0x%02x reg6: 0x%02x reg07: 0x%02x reg08: 0x%02x reg0e: 0x%02x",
		 it6161_hdmi_tx_read(it6161, 0x04),
	     it6161_hdmi_tx_read(it6161, 0x05),
		 it6161_hdmi_tx_read(it6161, 0x06),
	     it6161_hdmi_tx_read(it6161, 0x07),
		 it6161_hdmi_tx_read(it6161, 0x08),
	     it6161_hdmi_tx_read(it6161, 0x0e));

	it6161_hdmi_tx_set_bits(it6161, REG_TX_SW_RST, B_HDMITX_VID_RST, B_HDMITX_VID_RST);
	it6161_hdmi_tx_set_bits(it6161, REG_TX_SW_RST, B_HDMITX_VID_RST, 0x00);
	msleep(10);
}

/* DDC master will set to be host */
static void it6161_hdmi_tx_clear_ddc_fifo(struct it6161 *it6161)
{
	it6161_hdmi_tx_change_bank(it6161, 0);
	it6161_hdmi_tx_write(it6161, REG_TX_DDC_MASTER_CTRL,
			     B_TX_MASTERDDC | B_TX_MASTERHOST);
	it6161_hdmi_tx_write(it6161, REG_TX_DDC_CMD, CMD_FIFO_CLR);
	it6161_hdmi_tx_set_bits(it6161, REG_TX_DDC_MASTER_CTRL, B_TX_MASTERHOST, 0x00);
}

static void hdmi_tx_generate_blank_timing(struct it6161 *it6161)
{
	struct drm_display_mode *display_mode = &it6161->display_mode;
	bool force_hdmi_tx_clock_stable = true;
	bool force_hdmi_tx_video_stable = true;
	bool hdmi_tx_by_pass_mode = false;
	bool de_generation = false;
	bool enable_de_only = true;
	u8 polarity;
	u16 hsync_start, hsync_end, vsync_start, vsync_end, htotal, hde_start, vtotal;
	u16 vsync_start_2nd = 0, vsync_end_2nd = 0, vsync_rising_at_h_2nd;

	polarity =
	    ((display_mode->flags & DRM_MODE_FLAG_PHSYNC) == DRM_MODE_FLAG_PHSYNC) ? 0x02 : 0x00;
	polarity |=
	    ((display_mode->flags & DRM_MODE_FLAG_PVSYNC) == DRM_MODE_FLAG_PVSYNC) ? 0x04 : 0x00;

	hsync_start = display_mode->hsync_start - display_mode->hdisplay - 1;
	hsync_end = hsync_start + display_mode->hsync_end - display_mode->hsync_start;
	vsync_rising_at_h_2nd = hsync_start + display_mode->htotal / 2;
	hde_start = display_mode->htotal - display_mode->hsync_start;

	it6161_hdmi_tx_set_bits(it6161, 0xD1, 0x0C,
				force_hdmi_tx_clock_stable << 3 | force_hdmi_tx_video_stable << 2);
	it6161_hdmi_tx_set_bits(it6161, 0xA9, 0x80, hdmi_tx_by_pass_mode << 7);
	it6161_hdmi_tx_set_bits(it6161, 0x90, 0x01, de_generation);
	it6161_hdmi_tx_write(it6161, 0x91, vsync_rising_at_h_2nd >> 4);
	it6161_hdmi_tx_set_bits(it6161, 0x90, 0xF0, (vsync_rising_at_h_2nd & 0x00F) << 4);
	it6161_hdmi_tx_set_bits(it6161, 0x90, 0x06, polarity);
	it6161_hdmi_tx_write(it6161, 0x95, (u8) hsync_start);
	it6161_hdmi_tx_write(it6161, 0x96, (u8) hsync_end);
	it6161_hdmi_tx_write(it6161, 0x97, (hsync_end & 0x0F00) >> 4 | hsync_start >> 8);

	vsync_start = display_mode->vsync_start - display_mode->vdisplay;
	vsync_end = display_mode->vsync_end - display_mode->vdisplay;

	if ((display_mode->flags & DRM_MODE_FLAG_INTERLACE) != DRM_MODE_FLAG_INTERLACE) {
		vsync_start_2nd = 0x0FFF;
		vsync_end_2nd = 0x3F;
		vtotal = display_mode->vtotal - 1;
		it6161_hdmi_tx_set_bits(it6161, 0xA5, 0x10, 0x00);
	} else {
		vtotal = display_mode->vtotal * 2;
		it6161_hdmi_tx_set_bits(it6161, 0xA5, 0x10, 0x10);
	}
	it6161_hdmi_tx_write(it6161, 0xA0, (u8) vsync_start);
	it6161_hdmi_tx_write(it6161, 0xA1, (vsync_end & 0x0F) << 4 | vsync_start >> 8);
	it6161_hdmi_tx_write(it6161, 0xA2, (u8) vsync_start_2nd);
	it6161_hdmi_tx_write(it6161, 0xA6, (vsync_end_2nd & 0xF0) | vsync_end >> 4);
	it6161_hdmi_tx_write(it6161, 0xA3, (vsync_end_2nd & 0x0F) << 4 | vsync_start_2nd >> 8);
	it6161_hdmi_tx_write(it6161, 0xA4, vsync_rising_at_h_2nd);

	it6161_hdmi_tx_set_bits(it6161, 0xB1, 0x51,
				(hsync_end & 0x1000) >> 6 | (hsync_start & 0x1000) >> 8 | hde_start >> 12);
	it6161_hdmi_tx_set_bits(it6161, 0xA5, 0x2F,
				enable_de_only << 5 | vsync_rising_at_h_2nd >> 8);
	it6161_hdmi_tx_set_bits(it6161, 0xB2, 0x05,
				(vsync_rising_at_h_2nd & 0x1000) >> 10 | (vsync_rising_at_h_2nd & 0x1000) >> 12);

	htotal = display_mode->htotal - 1;
	it6161_hdmi_tx_set_bits(it6161, 0x90, 0xF0, (htotal & 0x0F) << 4);
	it6161_hdmi_tx_write(it6161, 0x91, (htotal & 0x0FF0) >> 4);
	it6161_hdmi_tx_set_bits(it6161, 0xB2, 0x01, (htotal & 0x1000) >> 12);
	it6161_hdmi_tx_write(it6161, 0x98, vtotal & 0x0FF);
	it6161_hdmi_tx_write(it6161, 0x99, (vtotal & 0xF00) >> 8);
}

/* force abort DDC and reset DDC bus */
static void it6161_hdmi_tx_abort_ddc(struct it6161 *it6161)
{
	struct device *dev = &it6161->i2c_hdmi_tx->dev;
	u8 sw_reset, retry = 2;
	u8 uc, timeout, i;

	DRM_DEV_DEBUG_DRIVER(dev, "ddc abort\n");
	/* save the sw reset, ddc master and cp desire setting */
	sw_reset = it6161_hdmi_tx_read(it6161, REG_TX_SW_RST);

	it6161_hdmi_tx_write(it6161, REG_TX_SW_RST, sw_reset | B_TX_HDCP_RST_HDMITX);
	it6161_hdmi_tx_write(it6161, REG_TX_DDC_MASTER_CTRL, B_TX_MASTERDDC | B_TX_MASTERHOST);

	/* do abort DDC */
	for (i = 0; i < retry; i++) {
		it6161_hdmi_tx_write(it6161, REG_TX_DDC_CMD, CMD_DDC_ABORT);
		it6161_hdmi_tx_write(it6161, REG_TX_DDC_CMD, CMD_GEN_SCLCLK);

		for (timeout = 0; timeout < 200; timeout++) {
			uc = it6161_hdmi_tx_read(it6161, REG_TX_DDC_STATUS);
			if (uc & B_TX_DDC_DONE)
				break;

			if (uc & (B_TX_DDC_NOACK | B_TX_DDC_WAITBUS | B_TX_DDC_ARBILOSE)) {
				DRM_DEV_ERROR(dev, "it6161_hdmi_tx_abort_ddc Fail by reg16=%02X\n", (int)uc);
				break;
			}
			/* delay 1 ms to stable */
			msleep(1);
		}
	}
}

static bool hdmi_tx_get_video_state(struct it6161 *it6161)
{
	return B_TXVIDSTABLE & it6161_hdmi_tx_read(it6161, REG_TX_SYS_STATUS);
}

static inline bool hdmi_tx_get_sink_hpd(struct it6161 *it6161)
{
	return it6161_hdmi_tx_read(it6161, REG_TX_SYS_STATUS) & B_TX_HPDETECT;
}

static bool it6161_ddc_op_finished(struct it6161 *it6161)
{
	int reg16 = it6161_hdmi_tx_read(it6161, REG_TX_DDC_STATUS);

	if (reg16 < 0)
		return false;

	return (reg16 & B_TX_DDC_DONE) == B_TX_DDC_DONE;
}

static int it6161_ddc_wait(struct it6161 *it6161)
{
	struct device *dev = &it6161->i2c_mipi_rx->dev;
	int status;
	unsigned long timeout;

	timeout = jiffies + msecs_to_jiffies(AUX_WAIT_TIMEOUT_MS) + 1;

	while (!it6161_ddc_op_finished(it6161)) {
		if (time_after(jiffies, timeout)) {
			DRM_DEV_ERROR(dev, "Timed out waiting AUX to finish");
			return -ETIMEDOUT;
		}
		usleep_range(1000, 2000);
	}

	status = it6161_hdmi_tx_read(it6161, REG_TX_DDC_STATUS);
	if (status < 0) {
		DRM_DEV_ERROR(dev, "Failed to read DDC channel: 0x%02x", status);
		return status;
	}

	if (status & B_TX_DDC_DONE)
		return 0;
	else {
		DRM_DEV_ERROR(dev, "DDC error: 0x%02x", status);
		return -EIO;
	}
}

static void hdmi_tx_ddc_operation(struct it6161 *it6161, u8 addr, u8 offset, u8 size,
			   u8 segment, u8 cmd)
{
	size = min_t(u8, size, DDC_FIFO_MAXREQ);
	it6161_hdmi_tx_change_bank(it6161, 0);
	it6161_hdmi_tx_write(it6161, REG_TX_DDC_MASTER_CTRL, B_TX_MASTERDDC | B_TX_MASTERHOST);
	it6161_hdmi_tx_write(it6161, REG_TX_DDC_HEADER, addr);
	it6161_hdmi_tx_write(it6161, REG_TX_DDC_REQOFF, offset);
	it6161_hdmi_tx_write(it6161, REG_TX_DDC_REQCOUNT, size);
	it6161_hdmi_tx_write(it6161, REG_TX_DDC_EDIDSEG, segment);
	it6161_hdmi_tx_write(it6161, REG_TX_DDC_CMD, cmd);
}

static int it6161_ddc_get_edid_operation(struct it6161 *it6161, u8 *buffer,
					 u8 segment, u8 offset, u8 size)
{
	struct device *dev = &it6161->i2c_hdmi_tx->dev;
	int status, i;

	if (!buffer)
		return -ENOMEM;

	if (it6161_hdmi_tx_read(it6161, REG_TX_INT_STAT1) & B_TX_INT_DDC_BUS_HANG) {
		DRM_DEV_ERROR(dev, "Called it6161_hdmi_tx_abort_ddc()");
		it6161_hdmi_tx_abort_ddc(it6161);
	}

	it6161_hdmi_tx_clear_ddc_fifo(it6161);
	status = it6161_ddc_wait(it6161);
	if (status < 0)
		goto error;

	hdmi_tx_ddc_operation(it6161, DDC_EDID_ADDRESS, offset, size, segment, CMD_EDID_READ);
	status = it6161_ddc_wait(it6161);
	if (status < 0)
		goto error;

	for (i = 0; i < size; i++) {
		status = it6161_hdmi_tx_read(it6161, REG_TX_DDC_READFIFO);
		if (status < 0)
			goto error;

		buffer[i] = status;
	}

	return i;

error:
	return status;
}

static int it6161_get_edid_block(void *data, u8 *buf, u32 block_num, size_t len)
{
	struct it6161 *it6161 = data;
	u8 offset, step = 8;
	int ret;

	step = min_t(u8, step, DDC_FIFO_MAXREQ);

	for (offset = 0; offset < len; offset += step) {
		ret = it6161_ddc_get_edid_operation(it6161, buf + offset,
						  block_num / 2, (block_num % 2) * EDID_LENGTH + offset, step);
		if (ret < 0)
			return ret;
	}
	return 0;
}

static void hdmi_tx_set_capability_from_edid_parse(struct it6161 *it6161, const struct edid *edid)
{
	struct device *dev = &it6161->i2c_hdmi_tx->dev;
	struct drm_display_info *info = &it6161->connector.display_info;

	it6161->hdmi_mode = drm_detect_hdmi_monitor(edid);
	it6161->support_audio = drm_detect_monitor_audio(edid);

	it6161->hdmi_tx_output_color_space = OUTPUT_COLOR_MODE;
	it6161->hdmi_tx_input_color_space = INPUT_COLOR_MODE;
	if (it6161->hdmi_tx_output_color_space == F_MODE_YUV444) {
		if ((info->color_formats & DRM_COLOR_FORMAT_YCBCR444) != DRM_COLOR_FORMAT_YCBCR444) {
			it6161->hdmi_tx_output_color_space &= ~F_MODE_CLRMOD_MASK;
			it6161->hdmi_tx_output_color_space |= F_MODE_RGB444;
		}
	}

	if (it6161->hdmi_tx_output_color_space == F_MODE_YUV422) {
		if ((info->color_formats & DRM_COLOR_FORMAT_YCBCR422) != DRM_COLOR_FORMAT_YCBCR422) {
			it6161->hdmi_tx_output_color_space &= ~F_MODE_CLRMOD_MASK;
			it6161->hdmi_tx_output_color_space |= F_MODE_RGB444;
		}
	}
	DRM_DEV_DEBUG_DRIVER(dev, "%s mode, monitor %ssupport audio, outputcolormode:%d color_formats:0x%08x color_depth:%d",
	     it6161->hdmi_mode ? "HDMI" : "DVI",
	     it6161->support_audio ? "" : "not ",
	     it6161->hdmi_tx_output_color_space,
		 info->color_formats,
	     info->bpc);

	if ((info->color_formats & DRM_COLOR_FORMAT_RGB444) == DRM_COLOR_FORMAT_RGB444)
		DRM_DEV_INFO(dev, "support RGB444 output");
	if ((info->color_formats & DRM_COLOR_FORMAT_YCBCR444) == DRM_COLOR_FORMAT_YCBCR444)
		DRM_DEV_INFO(dev, "support YUV444 output");
	if ((info->color_formats & DRM_COLOR_FORMAT_YCBCR422) == DRM_COLOR_FORMAT_YCBCR422)
		DRM_DEV_INFO(dev, "support YUV422 output");
}

static void it6161_variable_config(struct it6161 *it6161)
{
	it6161->hdmi_tx_mode = HDMI_TX_MODE;
	it6161->mipi_rx_lane_count = MIPI_RX_LANE_COUNT;
}

static const struct drm_edid *it6161_get_edid(struct it6161 *it6161)
{
	struct device *dev = &it6161->i2c_hdmi_tx->dev;
	const struct drm_edid *drm_edid;
	const struct edid *edid;

	drm_edid = drm_edid_read_custom(&it6161->connector, it6161_get_edid_block, it6161);
	if (!drm_edid) {
		DRM_DEV_ERROR(dev, "Failed to read EDID\n");
		return 0;
	}

	edid = drm_edid_raw(drm_edid);
	hdmi_tx_set_capability_from_edid_parse(it6161, edid);

	return drm_edid;
}

static int it6161_get_modes(struct drm_connector *connector)
{
	struct it6161 *it6161 = connector_to_it6161(connector);
	int err, num_modes = 0;
	const struct drm_edid *drm_edid;
	struct device *dev = &it6161->i2c_mipi_rx->dev;

	mutex_lock(&it6161->mode_lock);

	drm_edid = it6161_get_edid(it6161);
	if (!drm_edid) {
		DRM_DEV_ERROR(dev, "Failed to read EDID\n");
		return 0;
	}

	err = drm_edid_connector_update(connector, drm_edid);
	if (err) {
		DRM_DEV_ERROR(dev, "Failed to update connector from EDID: %d", err);
		goto unlock;
	}

	num_modes = drm_edid_connector_add_modes(connector);

	kfree(drm_edid);
unlock:
	DRM_DEV_DEBUG_DRIVER(dev, "edid mode number:%d", num_modes);
	mutex_unlock(&it6161->mode_lock);

	return num_modes;
}

static const struct drm_connector_helper_funcs it6161_connector_helper_funcs = {
	.get_modes = it6161_get_modes,
};

static enum drm_connector_status it6161_detect(struct drm_connector *connector, bool force)
{
	struct it6161 *it6161 = connector_to_it6161(connector);
	struct device *dev = &it6161->i2c_hdmi_tx->dev;
	enum drm_connector_status status = connector_status_disconnected;
	bool hpd;

	hpd = hdmi_tx_get_sink_hpd(it6161);
	if (hpd) {
		it6161_variable_config(it6161);
		status = connector_status_connected;
	}
	DRM_DEV_INFO(dev, "hpd:%s\n", hpd ? "high" : "low");

	it6161_set_interrupts_active_level(HIGH);
	it6161_mipi_rx_int_mask_enable(it6161);
	it6161_hdmi_tx_int_mask_enable(it6161);

	return status;
}

static const struct drm_connector_funcs it6161_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = it6161_detect,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int it6161_attach_dsi(struct it6161 *it6161)
{
	struct mipi_dsi_host *host;
	struct mipi_dsi_device *dsi;
	int ret = 0;
	const struct mipi_dsi_device_info info = {.type = "it6161", };
	struct device *dev = &it6161->i2c_hdmi_tx->dev;

	DRM_DEV_DEBUG_DRIVER(dev, "attach\n");
	host = of_find_mipi_dsi_host_by_node(it6161->host_node);
	if (!host) {
		DRM_DEV_ERROR(dev, "it6161 failed to find dsi host\n");
		return -EPROBE_DEFER;
	}

	dsi = mipi_dsi_device_register_full(host, &info);
	if (IS_ERR(dsi)) {
		DRM_DEV_ERROR(dev, "it6161 failed to create dsi device\n");
		ret = PTR_ERR(dsi);
		goto err_dsi_device;
	}

	it6161->dsi = dsi;

	dsi->lanes = MIPI_RX_LANE_COUNT;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
	    MIPI_DSI_MODE_NO_EOT_PACKET | MIPI_DSI_MODE_VIDEO_HSE |
	    MIPI_DSI_MODE_VIDEO_NO_HFP | MIPI_DSI_MODE_VIDEO_NO_HBP |
	    MIPI_DSI_MODE_VIDEO_NO_HSA;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "it6161 failed to attach dsi to host\n");
		goto err_dsi_attach;
	}

	return 0;

err_dsi_attach:
	mipi_dsi_device_unregister(dsi);
err_dsi_device:
	return ret;
}

static int it6161_connector_init(struct drm_bridge *bridge, struct it6161 *it6161)
{
	struct device *dev;
	int ret;

	dev = &it6161->i2c_mipi_rx->dev;

	if (!bridge->encoder) {
		DRM_DEV_ERROR(dev, "Parent encoder object not found");
		return -ENODEV;
	}

	it6161->connector.polled = DRM_CONNECTOR_POLL_HPD;

	ret = drm_connector_init(bridge->dev, &it6161->connector,
				 &it6161_connector_funcs, DRM_MODE_CONNECTOR_HDMIA);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to initialize connector: %d", ret);
		return ret;
	}

	drm_connector_helper_add(&it6161->connector, &it6161_connector_helper_funcs);
	drm_connector_attach_encoder(&it6161->connector, bridge->encoder);

	return 0;
}

static int it6161_bridge_attach(struct drm_bridge *bridge,
				enum drm_bridge_attach_flags flags)
{
	struct it6161 *it6161 = bridge_to_it6161(bridge);
	int ret;

	if (!(flags & DRM_BRIDGE_ATTACH_NO_CONNECTOR)) {
		ret = it6161_connector_init(bridge, it6161);
		if (ret < 0)
			return ret;
	}

	ret = it6161_attach_dsi(it6161);

	return ret;
}

static void it6161_detach_dsi(struct it6161 *it6161)
{
	mipi_dsi_detach(it6161->dsi);
	mipi_dsi_device_unregister(it6161->dsi);
}

static void it6161_bridge_detach(struct drm_bridge *bridge)
{
	struct it6161 *it6161 = bridge_to_it6161(bridge);

	drm_connector_unregister(&it6161->connector);
	drm_connector_cleanup(&it6161->connector);
	it6161_detach_dsi(it6161);
}

static enum drm_mode_status
it6161_bridge_mode_valid(struct drm_bridge *bridge,
			 const struct drm_display_info *info,
			 const struct drm_display_mode *mode)
{
	if (mode->clock > 108000)
		return MODE_CLOCK_HIGH;

	/* TODO, Only 480p60 work with imx8ulp now */
	if (mode->vdisplay > 480)
		return MODE_BAD_VVALUE;

	return MODE_OK;
}

static void it6161_bridge_mode_set(struct drm_bridge *bridge,
				   const struct drm_display_mode *mode,
				   const struct drm_display_mode *adjusted_mode)
{
	struct it6161 *it6161 = bridge_to_it6161(bridge);
	struct device *dev = &it6161->i2c_hdmi_tx->dev;
	u8 polarity;

	DRM_DEV_DEBUG_DRIVER(dev, "    mode " DRM_MODE_FMT "\n", DRM_MODE_ARG(mode));
	DRM_DEV_DEBUG_DRIVER(dev, "adj mode " DRM_MODE_FMT "\n", DRM_MODE_ARG(adjusted_mode));

	memcpy(&it6161->display_mode, mode, sizeof(struct drm_display_mode));

	polarity = ((adjusted_mode->flags & DRM_MODE_FLAG_PHSYNC) == DRM_MODE_FLAG_PHSYNC) ? 0x01 : 0x00;
	polarity |= ((adjusted_mode->flags & DRM_MODE_FLAG_PVSYNC) == DRM_MODE_FLAG_PVSYNC) ? 0x02 : 0x00;

	it6161_mipi_rx_set_bits(it6161, 0x4E, 0x03, polarity);
}

static void it6161_bridge_enable(struct drm_bridge *bridge)
{
	struct it6161 *it6161 = bridge_to_it6161(bridge);
	struct device *dev = &it6161->i2c_hdmi_tx->dev;

	DRM_DEV_DEBUG_DRIVER(dev, "start");

	if (it6161->bridge_enable)
		return;

	mipi_rx_init(it6161);
	hdmi_tx_init(it6161);
	it6161_set_interrupts_active_level(HIGH);
	it6161_mipi_rx_int_mask_enable(it6161);
	it6161_hdmi_tx_int_mask_enable(it6161);

	it6161->bridge_enable = true;

}

static void it6161_bridge_disable(struct drm_bridge *bridge)
{
	struct it6161 *it6161 = bridge_to_it6161(bridge);
	struct device *dev = &it6161->i2c_hdmi_tx->dev;

	DRM_DEV_DEBUG_DRIVER(dev, "start");

	/* only keep HPD for cabe detect */
	it6161_mipi_rx_int_mask_disable(it6161);
	it6161_hdmi_tx_int_mask_disable(it6161);
	it6161_hdmi_tx_set_bits(it6161, 0x0F, 0x03, 0x00);
	it6161_hdmi_tx_write(it6161, REG_TX_INT_MASK1, ~B_TX_HPD_MASK);

	it6161->bridge_enable = false;
}

static enum drm_connector_status it6161_bridge_detect(struct drm_bridge *bridge)
{
	struct it6161 *it6161 = bridge_to_it6161(bridge);
	enum drm_connector_status status = connector_status_disconnected;
	bool hpd = hdmi_tx_get_sink_hpd(it6161);
	struct device *dev = &it6161->i2c_hdmi_tx->dev;

	DRM_DEV_DEBUG_DRIVER(dev, "hpd:%s", hpd ? "high" : "low");

	if (hpd) {
		it6161_variable_config(it6161);
		status = connector_status_connected;
	}

	it6161_set_interrupts_active_level(HIGH);
	it6161_mipi_rx_int_mask_enable(it6161);
	it6161_hdmi_tx_int_mask_enable(it6161);

	return status;
}

static const struct drm_edid *
it6161_bridge_edid_read(struct drm_bridge *bridge,
			struct drm_connector *connector)
{
	struct it6161 *it6161 = bridge_to_it6161(bridge);

	return it6161_get_edid(it6161);
}

static const struct drm_bridge_funcs it6161_bridge_funcs = {
	.attach = it6161_bridge_attach,
	.detach = it6161_bridge_detach,
	.mode_valid = it6161_bridge_mode_valid,
	.mode_set = it6161_bridge_mode_set,
	.enable = it6161_bridge_enable,
	.disable = it6161_bridge_disable,
	.detect = it6161_bridge_detect,
	.edid_read = it6161_bridge_edid_read,
};

static bool it6161_check_device_ready(struct it6161 *it6161)
{
	struct device *dev = &it6161->i2c_hdmi_tx->dev;
	u8 Vendor_ID[2], Device_ID[2];

	Vendor_ID[0] = it6161_mipi_rx_read(it6161, 0x00);
	Vendor_ID[1] = it6161_mipi_rx_read(it6161, 0x01);
	Device_ID[0] = it6161_mipi_rx_read(it6161, 0x02);
	Device_ID[1] = it6161_mipi_rx_read(it6161, 0x03);
	if (Vendor_ID[0] == 0x54 && Vendor_ID[1] == 0x49 &&
			Device_ID[0] == 0x61 && Device_ID[1] == 0x61) {
		DRM_DEV_INFO(dev, "Find it6161 revision: 0x%2x",
				(u32) it6161_mipi_rx_read(it6161, 0x04));
		return true;
	}
	DRM_DEV_INFO(dev, "find it6161 Fail");
	return false;
}

static void it6161_hdmi_tx_set_av_mute(struct it6161 *it6161, u8 bEnable)
{
	it6161_hdmi_tx_change_bank(it6161, 0);
	it6161_hdmi_tx_set_bits(it6161, REG_TX_GCP,
			B_TX_SETAVMUTE,	bEnable ? B_TX_SETAVMUTE : 0);
	it6161_hdmi_tx_write(it6161, REG_TX_PKT_GENERAL_CTRL,
			B_TX_ENABLE_PKT | B_TX_REPEAT_PKT);
}

static void hdmi_tx_setup_pclk_div2(struct it6161 *it6161)
{
	struct device *dev = &it6161->i2c_hdmi_tx->dev;

	if (HDMI_TX_PCLK_DIV2) {
		DRM_DEV_DEBUG_DRIVER(dev, "PCLK Divided by 2 mode");
		it6161_hdmi_tx_set_bits(it6161, REG_TX_INPUT_MODE,
					B_TX_PCLKDIV2, B_TX_PCLKDIV2);
	}
}

/*************************************************************************
 * Function: hdmi_tx_setup_csc
 * Parameter: input_mode -
 *      D[1:0] - Color Mode
 *      D[4] - Colorimetry 0: ITU_BT601 1: ITU_BT709
 *      D[5] - Quantization 0: 0_255 1: 16_235
 *      D[6] - Up/Dn Filter 'Required'
 *         0: no up/down filter
 *         1: enable up/down filter when csc need.
 *      D[7] - Dither Filter 'Required'
 *         0: no dither enabled.
 *         1: enable dither and dither free go "when required".
 * output_mode -
 *      D[1:0] - Color mode.
 * Return: N/A
 * Remark: reg72~reg8D will be programmed depended the input with table
 * **********************************************************************/
static void hdmi_tx_setup_csc(struct it6161 *it6161)
{
	struct device *dev = &it6161->i2c_hdmi_tx->dev;
	u8 ucData, csc = 0, i;
	u8 filter = 0;	/* filter is for Video CTRL DN_FREE_GO,EN_DITHER,and ENUDFILT */
	u8 input_mode = it6161->hdmi_tx_input_color_space;
	u8 output_mode = it6161->hdmi_tx_output_color_space;
	u8 *ptable = NULL;

	/* (1) YUV422 in,RGB/YUV444 output (Output is 8-bit,input is 12-bit)
	 * (2) YUV444/422  in,RGB output (CSC enable,and output is not YUV422)
	 * (3) RGB in,YUV444 output   (CSC enable,and output is not YUV422)
	 *
	 * YUV444/RGB24 <-> YUV422 need set up/down filter.
	 */
	DRM_DEV_DEBUG_DRIVER(dev, "hdmi_tx_setup_csc(u8 input_mode = %x,u8 output_mode = %x)\n",
		 (int)input_mode, (int)output_mode);

	switch (input_mode & F_MODE_CLRMOD_MASK) {
	/* YUV444 INPUT */
	case F_MODE_YUV444:
		switch (output_mode & F_MODE_CLRMOD_MASK) {
		case F_MODE_YUV444:
			csc = B_HDMITX_CSC_BYPASS;
			break;
		case F_MODE_YUV422:
			/* YUV444 to YUV422 need up/down filter for processing. */
			if (input_mode & F_VIDMODE_EN_UDFILT)
				filter |= B_TX_EN_UDFILTER;
			csc = B_HDMITX_CSC_BYPASS;
			break;
		case F_MODE_RGB444:
			csc = B_HDMITX_CSC_YUV2RGB;
			/* YUV444 to RGB24 need dither */
			if (input_mode & F_VIDMODE_EN_DITHER)
				filter |= B_TX_EN_DITHER | B_TX_DNFREE_GO;
			break;
		}
		break;

	/* YUV422 INPUT */
	case F_MODE_YUV422:
		switch (output_mode & F_MODE_CLRMOD_MASK) {
		case F_MODE_YUV444:
			csc = B_HDMITX_CSC_BYPASS;
			if (input_mode & F_VIDMODE_EN_UDFILT)
				filter |= B_TX_EN_UDFILTER;
			else if (input_mode & F_VIDMODE_EN_DITHER)
				filter |= B_TX_EN_DITHER | B_TX_DNFREE_GO;
			break;
		case F_MODE_YUV422:
			csc = B_HDMITX_CSC_BYPASS;
			break;
		case F_MODE_RGB444:
			csc = B_HDMITX_CSC_YUV2RGB;
			if (input_mode & F_VIDMODE_EN_UDFILT)
				filter |= B_TX_EN_UDFILTER;
			else if (input_mode & F_VIDMODE_EN_DITHER)
				filter |= B_TX_EN_DITHER | B_TX_DNFREE_GO;
			break;
		}
		break;

	/* RGB444 INPUT */
	case F_MODE_RGB444:
		switch (output_mode & F_MODE_CLRMOD_MASK) {
		case F_MODE_YUV444:
			csc = B_HDMITX_CSC_RGB2YUV;
			if (input_mode & F_VIDMODE_EN_DITHER)
				filter |= B_TX_EN_DITHER | B_TX_DNFREE_GO;
			break;
		case F_MODE_YUV422:
			if (input_mode & F_VIDMODE_EN_UDFILT)
				filter |= B_TX_EN_UDFILTER;
			else if (input_mode & F_VIDMODE_EN_DITHER)
				filter |= B_TX_EN_DITHER | B_TX_DNFREE_GO;
			csc = B_HDMITX_CSC_RGB2YUV;
			break;
		case F_MODE_RGB444:
			csc = B_HDMITX_CSC_BYPASS;
			break;
		}
		break;
	}

	/* set the CSC metrix registers by colorimetry and quantization */
	if (csc == B_HDMITX_CSC_RGB2YUV) {
		switch (input_mode & (F_VIDMODE_ITU709 | F_VIDMODE_16_235)) {
		case F_VIDMODE_ITU709 | F_VIDMODE_16_235:
			ptable = bCSCMtx_RGB2YUV_ITU709_16_235;
			break;
		case F_VIDMODE_ITU709 | F_VIDMODE_0_255:
			ptable = bCSCMtx_RGB2YUV_ITU709_0_255;
			break;
		case F_VIDMODE_ITU601 | F_VIDMODE_16_235:
			ptable = bCSCMtx_RGB2YUV_ITU601_16_235;
			break;
		case F_VIDMODE_ITU601 | F_VIDMODE_0_255:
		default:
			ptable = bCSCMtx_RGB2YUV_ITU601_0_255;
			break;
		}
	}

	if (csc == B_HDMITX_CSC_YUV2RGB) {
		switch (input_mode & (F_VIDMODE_ITU709 | F_VIDMODE_16_235)) {
		case F_VIDMODE_ITU709 | F_VIDMODE_16_235:
			ptable = bCSCMtx_YUV2RGB_ITU709_16_235;
			break;
		case F_VIDMODE_ITU709 | F_VIDMODE_0_255:
			ptable = bCSCMtx_YUV2RGB_ITU709_0_255;
			break;
		case F_VIDMODE_ITU601 | F_VIDMODE_16_235:
			ptable = bCSCMtx_YUV2RGB_ITU601_16_235;
			break;
		case F_VIDMODE_ITU601 | F_VIDMODE_0_255:
		default:
			ptable = bCSCMtx_YUV2RGB_ITU601_0_255;
			break;
		}
	}

	if (csc == B_HDMITX_CSC_BYPASS)
		it6161_hdmi_tx_set_bits(it6161, 0xF, 0x10, 0x10);
	else {
		if (ptable != NULL)
			for (i = 0; i < SIZEOF_CSCMTX; i++)
				it6161_hdmi_tx_write(it6161, REG_TX_CSC_YOFF + i, ptable[i]);
		it6161_hdmi_tx_set_bits(it6161, 0xF, 0x10, 0x00);
	}

	ucData = it6161_hdmi_tx_read(it6161,
				REG_TX_CSC_CTRL) & ~(M_TX_CSC_SEL |
						     B_TX_DNFREE_GO |
						     B_TX_EN_DITHER |
						     B_TX_EN_UDFILTER);
	ucData |= filter | csc;

	it6161_hdmi_tx_write(it6161, REG_TX_CSC_CTRL, ucData);
}

static void hdmi_tx_setup_afe(struct it6161 *it6161, u8 level)
{
	struct device *dev = &it6161->i2c_hdmi_tx->dev;

	it6161_hdmi_tx_write(it6161, REG_TX_AFE_DRV_CTRL, B_TX_AFE_DRV_RST);
	switch (level) {
	case PCLK_HIGH:
		it6161_hdmi_tx_set_bits(it6161, 0x62, 0x90, 0x80);
		it6161_hdmi_tx_set_bits(it6161, 0x64, 0x89, 0x80);
		it6161_hdmi_tx_set_bits(it6161, 0x68, 0x10, 0x00);
		it6161_hdmi_tx_set_bits(it6161, 0x66, 0x80, 0x80);
		break;
	default:
		it6161_hdmi_tx_set_bits(it6161, 0x62, 0x90, 0x10);
		it6161_hdmi_tx_set_bits(it6161, 0x64, 0x89, 0x09);
		it6161_hdmi_tx_set_bits(it6161, 0x68, 0x10, 0x10);
		break;
	}
	DRM_DEV_DEBUG_DRIVER(dev, "setup afe: %s", level ? "high" : "low");
}

static void hdmi_tx_fire_afe(struct it6161 *it6161)
{
	it6161_hdmi_tx_change_bank(it6161, 0x00);
	it6161_hdmi_tx_write(it6161, REG_TX_AFE_DRV_CTRL, 0x00);
}

static void hdmi_tx_disable_video_output(struct it6161 *it6161)
{
	it6161_hdmi_tx_set_bits(it6161, REG_TX_SW_RST, B_HDMITX_VID_RST, B_HDMITX_VID_RST);
	it6161_hdmi_tx_write(it6161, REG_TX_AFE_DRV_CTRL, B_TX_AFE_DRV_RST | B_TX_AFE_DRV_PWD);
	it6161_hdmi_tx_set_bits(it6161, 0x62, 0x90, 0x00);
	it6161_hdmi_tx_set_bits(it6161, 0x64, 0x89, 0x00);
}

static void hdmi_tx_enable_video_output(struct it6161 *it6161, u8 level)
{
	it6161_hdmi_tx_write(it6161, REG_TX_SW_RST,
			     B_HDMITX_AUD_RST | B_TX_AREF_RST | B_TX_HDCP_RST_HDMITX);
	it6161_hdmi_tx_change_bank(it6161, 1);
	it6161_hdmi_tx_write(it6161, REG_TX_AVIINFO_DB1, 0x00);
	it6161_hdmi_tx_change_bank(it6161, 0);

	if (it6161->hdmi_mode)
		it6161_hdmi_tx_set_av_mute(it6161, true);

	hdmi_tx_setup_pclk_div2(it6161);
	hdmi_tx_setup_csc(it6161);
	it6161_hdmi_tx_write(it6161, REG_TX_HDMI_MODE,
			     it6161->hdmi_mode ? B_TX_HDMI_MODE : B_TX_DVI_MODE);
	hdmi_tx_setup_afe(it6161, level);
	hdmi_tx_fire_afe(it6161);
}

static void setHDMITX_ChStat(struct it6161 *it6161, u8 ucIEC60958ChStat[])
{
	u8 uc;

	it6161_hdmi_tx_change_bank(it6161, 1);
	uc = (ucIEC60958ChStat[0] << 1) & 0x7C;
	it6161_hdmi_tx_write(it6161, REG_TX_AUDCHST_MODE, uc);
	it6161_hdmi_tx_write(it6161, REG_TX_AUDCHST_CAT, ucIEC60958ChStat[1]);
	it6161_hdmi_tx_write(it6161, REG_TX_AUDCHST_SRCNUM, ucIEC60958ChStat[2] & 0xF);
	it6161_hdmi_tx_write(it6161, REG_TX_AUD0CHST_CHTNUM, (ucIEC60958ChStat[2] >> 4) & 0xF);
	it6161_hdmi_tx_write(it6161, REG_TX_AUDCHST_CA_FS, ucIEC60958ChStat[3]);
	it6161_hdmi_tx_write(it6161, REG_TX_AUDCHST_OFS_WL, ucIEC60958ChStat[4]);
	it6161_hdmi_tx_change_bank(it6161, 0);
}

static void setHDMITX_LPCMAudio(u8 AudioSrcNum, u8 AudSWL, u8 bAudInterface)
{
	u8 AudioEnable = 0, AudioFormat = 0, bTDMSetting;

	switch (AudSWL) {
	case 16:
		AudioEnable |= M_TX_AUD_16BIT;
		break;
	case 18:
		AudioEnable |= M_TX_AUD_18BIT;
		break;
	case 20:
		AudioEnable |= M_TX_AUD_20BIT;
		break;
	case 24:
	default:
		AudioEnable |= M_TX_AUD_24BIT;
		break;
	}
	if (bAudInterface == SPDIF) {
		AudioFormat &= ~0x40;
		AudioEnable |= B_TX_AUD_SPDIF | B_TX_AUD_EN_I2S0;
	} else {
		AudioFormat |= 0x40;
		switch (AudioSrcNum) {
		case 4:
			AudioEnable |=
			    B_TX_AUD_EN_I2S3 | B_TX_AUD_EN_I2S2 |
			    B_TX_AUD_EN_I2S1 | B_TX_AUD_EN_I2S0;
			break;

		case 3:
			AudioEnable |=
			    B_TX_AUD_EN_I2S2 | B_TX_AUD_EN_I2S1 | B_TX_AUD_EN_I2S0;
			break;

		case 2:
			AudioEnable |= B_TX_AUD_EN_I2S1 | B_TX_AUD_EN_I2S0;
			break;

		case 1:
		default:
			AudioFormat &= ~0x40;
			AudioEnable |= B_TX_AUD_EN_I2S0;
			break;

		}
	}
	AudioFormat |= 0x01;
	it6161->bAudioChannelEnable = AudioEnable;

	it6161_hdmi_tx_change_bank(it6161, 0);
	it6161_hdmi_tx_write(it6161, REG_TX_AUDIO_CTRL0, AudioEnable & 0xF0);

	it6161_hdmi_tx_write(it6161, REG_TX_AUDIO_CTRL1, AudioFormat);
	it6161_hdmi_tx_write(it6161, REG_TX_AUDIO_FIFOMAP, 0xE4);

	if (bAudInterface == SPDIF)
		it6161_hdmi_tx_write(it6161, REG_TX_AUDIO_CTRL3, B_TX_CHSTSEL);
	else
		it6161_hdmi_tx_write(it6161, REG_TX_AUDIO_CTRL3, 0);

	it6161_hdmi_tx_write(it6161, REG_TX_AUD_SRCVALID_FLAT, 0x00);
	it6161_hdmi_tx_write(it6161, REG_TX_AUD_HDAUDIO, 0x00);

	if (bAudInterface == SPDIF) {
		u8 i;

		it6161_hdmi_tx_set_bits(it6161, 0x5c, (1 << 6), (1 << 6));
		for (i = 0; i < 100; i++)
			if (it6161_hdmi_tx_read(it6161, REG_TX_CLK_STATUS2) & B_TX_OSF_LOCK)
				break;	/* stable clock. */
	} else {
		bTDMSetting = it6161_hdmi_tx_read(it6161, REG_TX_AUD_HDAUDIO);
		if (bAudInterface == TDM) {
			bTDMSetting |= B_TX_TDM;
			bTDMSetting &= 0x9F;
			bTDMSetting |= (AudioSrcNum - 1) << 5;
		} else
			bTDMSetting &= ~B_TX_TDM;

		/* 1 channel NLPCM, no TDM mode. */
		it6161_hdmi_tx_write(it6161, REG_TX_AUD_HDAUDIO, bTDMSetting);
	}
}

static void setHDMITX_NLPCMAudio(u8 bAudInterface)
{
	u8 AudioEnable;
	u8 i;

	if (bAudInterface == SPDIF)
		AudioEnable = M_TX_AUD_24BIT | B_TX_AUD_SPDIF;
	else
		AudioEnable = M_TX_AUD_24BIT;

	it6161_hdmi_tx_change_bank(it6161, 0);
	it6161_hdmi_tx_write(it6161, REG_TX_AUDIO_CTRL0, AudioEnable);
	it6161_hdmi_tx_write(it6161, REG_TX_AUDIO_CTRL1, 0x01);
	it6161_hdmi_tx_write(it6161, REG_TX_AUDIO_FIFOMAP, 0xE4);
	it6161_hdmi_tx_write(it6161, REG_TX_AUDIO_CTRL3, B_TX_CHSTSEL);
	it6161_hdmi_tx_write(it6161, REG_TX_AUD_SRCVALID_FLAT, 0x00);
	it6161_hdmi_tx_write(it6161, REG_TX_AUD_HDAUDIO, 0x00);

	if (bAudInterface == SPDIF) {
		for (i = 0; i < 100; i++)
			if (it6161_hdmi_tx_read(it6161, REG_TX_CLK_STATUS2) & B_TX_OSF_LOCK)
				break;
	} else {
		i = it6161_hdmi_tx_read(it6161, REG_TX_AUD_HDAUDIO);
		i &= ~B_TX_TDM;
		/* 2 channel NLPCM, no TDM mode. */
		it6161_hdmi_tx_write(it6161, REG_TX_AUD_HDAUDIO, i);
	}
	it6161_hdmi_tx_write(it6161, REG_TX_AUDIO_CTRL0,
			     AudioEnable | B_TX_AUD_EN_I2S0);
}

static void setHDMITX_HBRAudio(u8 bAudInterface)
{
	it6161_hdmi_tx_change_bank(it6161, 0);

	it6161_hdmi_tx_write(it6161, REG_TX_AUDIO_CTRL1, 0x47);
	it6161_hdmi_tx_write(it6161, REG_TX_AUDIO_FIFOMAP, 0xE4);

	if (bAudInterface == SPDIF) {
		it6161_hdmi_tx_write(it6161, REG_TX_AUDIO_CTRL0,
				     M_TX_AUD_24BIT | B_TX_AUD_SPDIF);
		it6161_hdmi_tx_write(it6161, REG_TX_AUDIO_CTRL3, B_TX_CHSTSEL);
	} else {
		it6161_hdmi_tx_write(it6161, REG_TX_AUDIO_CTRL0, M_TX_AUD_24BIT);
		it6161_hdmi_tx_write(it6161, REG_TX_AUDIO_CTRL3, 0);
	}
	it6161_hdmi_tx_write(it6161, REG_TX_AUD_SRCVALID_FLAT, 0x08);
	it6161_hdmi_tx_write(it6161, REG_TX_AUD_HDAUDIO, B_TX_HBR);

	if (bAudInterface == SPDIF) {
		u8 i;

		for (i = 0; i < 100; i++)
			if (it6161_hdmi_tx_read(it6161, REG_TX_CLK_STATUS2) & B_TX_OSF_LOCK)
				break;
		it6161_hdmi_tx_write(it6161, REG_TX_AUDIO_CTRL0,
				     M_TX_AUD_24BIT | B_TX_AUD_SPDIF | B_TX_AUD_EN_SPDIF);
	} else {
		it6161_hdmi_tx_write(it6161, REG_TX_AUDIO_CTRL0,
				     M_TX_AUD_24BIT | B_TX_AUD_EN_I2S3 |
				     B_TX_AUD_EN_I2S2 | B_TX_AUD_EN_I2S1 |
				     B_TX_AUD_EN_I2S0);
	}
	it6161_hdmi_tx_set_bits(it6161, 0x5c, 1 << 6, 0x00);
	it6161->bAudioChannelEnable = it6161_hdmi_tx_read(it6161, REG_TX_AUDIO_CTRL0);
}

static void setHDMITX_DSDAudio(void)
{
	it6161_hdmi_tx_write(it6161, REG_TX_AUDIO_CTRL1, 0x41);
	it6161_hdmi_tx_write(it6161, REG_TX_AUDIO_FIFOMAP, 0xE4);
	it6161_hdmi_tx_write(it6161, REG_TX_AUDIO_CTRL0, M_TX_AUD_24BIT);
	it6161_hdmi_tx_write(it6161, REG_TX_AUDIO_CTRL3, 0);
	it6161_hdmi_tx_write(it6161, REG_TX_AUD_SRCVALID_FLAT, 0x00);
	it6161_hdmi_tx_write(it6161, REG_TX_AUD_HDAUDIO, B_TX_DSD);

	it6161_hdmi_tx_write(it6161, REG_TX_AUDIO_CTRL0,
			     M_TX_AUD_24BIT | B_TX_AUD_EN_I2S3 |
			     B_TX_AUD_EN_I2S2 | B_TX_AUD_EN_I2S1 |
			     B_TX_AUD_EN_I2S0);
}

static void HDMITX_DisableAudioOutput(struct it6161 *it6161)
{
	it6161_hdmi_tx_set_bits(it6161, REG_TX_SW_RST,
				(B_HDMITX_AUD_RST | B_TX_AREF_RST),
				(B_HDMITX_AUD_RST | B_TX_AREF_RST));
	it6161_hdmi_tx_set_bits(it6161, 0x0F, 0x10, 0x10);
}

static void setHDMITX_NCTS(u8 Fs)
{
	u32 n, LastCTS = 0;
	bool HBR_mode;

	if (B_TX_HBR & it6161_hdmi_tx_read(it6161, REG_TX_AUD_HDAUDIO))
		HBR_mode = true;
	else
		HBR_mode = false;

	switch (Fs) {
	case AUDFS_32KHz:
		n = 4096;
		break;
	case AUDFS_44p1KHz:
		n = 6272;
		break;
	case AUDFS_48KHz:
		n = 6144;
		break;
	case AUDFS_88p2KHz:
		n = 12544;
		break;
	case AUDFS_96KHz:
		n = 12288;
		break;
	case AUDFS_176p4KHz:
		n = 25088;
		break;
	case AUDFS_192KHz:
		n = 24576;
		break;
	case AUDFS_768KHz:
		n = 24576;
		break;
	default:
		n = 6144;
	}

	it6161_hdmi_tx_change_bank(it6161, 1);
	it6161_hdmi_tx_write(it6161, REGPktAudN0, (u8) ((n) & 0xFF));
	it6161_hdmi_tx_write(it6161, REGPktAudN1, (u8) ((n >> 8) & 0xFF));
	it6161_hdmi_tx_write(it6161, REGPktAudN2, (u8) ((n >> 16) & 0xF));

	it6161_hdmi_tx_write(it6161, REGPktAudCTS0, (u8) ((LastCTS) & 0xFF));
	it6161_hdmi_tx_write(it6161, REGPktAudCTS1, (u8) ((LastCTS >> 8) & 0xFF));
	it6161_hdmi_tx_write(it6161, REGPktAudCTS2, (u8) ((LastCTS >> 16) & 0xF));
	it6161_hdmi_tx_change_bank(it6161, 0);

	it6161_hdmi_tx_write(it6161, 0xF8, 0xC3);
	it6161_hdmi_tx_write(it6161, 0xF8, 0xA5);
	/* D[1] = 0, HW auto count CTS */
	it6161_hdmi_tx_set_bits(it6161, REG_TX_PKT_SINGLE_CTRL, B_TX_SW_CTS, 0x00);
	it6161_hdmi_tx_write(it6161, 0xF8, 0xFF);

	if (false == HBR_mode) {
		/* LPCM */
		u8 uData;

		it6161_hdmi_tx_change_bank(it6161, 1);
		Fs = AUDFS_768KHz;
		it6161_hdmi_tx_write(it6161, REG_TX_AUDCHST_CA_FS, 0x00 | Fs);
		/* OFS is the one's complement of FS */
		Fs = ~Fs;
		uData = (0x0f & it6161_hdmi_tx_read(it6161, REG_TX_AUDCHST_OFS_WL));
		it6161_hdmi_tx_write(it6161, REG_TX_AUDCHST_OFS_WL, (Fs << 4) | uData);
		it6161_hdmi_tx_change_bank(it6161, 0);
	}
}

static void HDMITX_EnableAudioOutput(struct it6161 *it6161, u8 AudioType,
			      u8 bAudInterface, u32 SampleFreq, u8 ChNum, u8 *pIEC60958ChStat)
{
	struct device *dev = &it6161->i2c_hdmi_tx->dev;
	static u8 ucIEC60958ChStat[5];
	u8 Fs;

	it6161->bAudioChannelEnable = 0;
	it6161_hdmi_tx_set_bits(it6161, REG_TX_SW_RST,
				(B_HDMITX_AUD_RST | B_TX_AREF_RST),
				(B_HDMITX_AUD_RST | B_TX_AREF_RST));
	it6161_hdmi_tx_write(it6161, REG_TX_CLK_CTRL0,
			     B_TX_AUTO_OVER_SAMPLING_CLOCK | B_TX_EXT_256FS | 0x01);

	/* power on the ACLK */
	it6161_hdmi_tx_set_bits(it6161, 0x0F, 0x10, 0x00);

	if (bAudInterface == SPDIF) {
		if (AudioType == T_AUDIO_HBR)
			it6161_hdmi_tx_write(it6161, REG_TX_CLK_CTRL0, 0x81);

		it6161_hdmi_tx_set_bits(it6161, REG_TX_AUDIO_CTRL0,
					B_TX_AUD_SPDIF, B_TX_AUD_SPDIF);
	} else
		it6161_hdmi_tx_set_bits(it6161, REG_TX_AUDIO_CTRL0, B_TX_AUD_SPDIF, 0x00);

	if (AudioType != T_AUDIO_DSD) {
		/* one bit audio have no channel status. */
		switch (SampleFreq) {
		case 44100L:
			Fs = AUDFS_44p1KHz;
			break;
		case 88200L:
			Fs = AUDFS_88p2KHz;
			break;
		case 176400L:
			Fs = AUDFS_176p4KHz;
			break;
		case 32000L:
			Fs = AUDFS_32KHz;
			break;
		case 48000L:
			Fs = AUDFS_48KHz;
			break;
		case 96000L:
			Fs = AUDFS_96KHz;
			break;
		case 192000L:
			Fs = AUDFS_192KHz;
			break;
		case 768000L:
			Fs = AUDFS_768KHz;
			break;
		default:
			SampleFreq = 48000L;
			Fs = AUDFS_48KHz;
			break;
		}

		setHDMITX_NCTS(Fs);
		if (pIEC60958ChStat == NULL) {
			ucIEC60958ChStat[0] = 0;
			ucIEC60958ChStat[1] = 0;
			ucIEC60958ChStat[2] = (ChNum + 1) / 2;

			if (ucIEC60958ChStat[2] < 1)
				ucIEC60958ChStat[2] = 1;
			else if (ucIEC60958ChStat[2] > 4)
				ucIEC60958ChStat[2] = 4;

			ucIEC60958ChStat[3] = Fs;
			ucIEC60958ChStat[4] = (((~Fs) << 4) & 0xF0) | CHTSTS_SWCODE;
			pIEC60958ChStat = ucIEC60958ChStat;
		}
	}
	it6161_hdmi_tx_set_bits(it6161, REG_TX_SW_RST,
				(B_HDMITX_AUD_RST | B_TX_AREF_RST), B_TX_AREF_RST);

	switch (AudioType) {
	case T_AUDIO_HBR:
		DRM_DEV_DEBUG_DRIVER(dev, "T_AUDIO_HBR\n");
		pIEC60958ChStat[0] |= 1 << 1;
		pIEC60958ChStat[2] = 0;
		pIEC60958ChStat[3] &= 0xF0;
		pIEC60958ChStat[3] |= AUDFS_768KHz;
		pIEC60958ChStat[4] |= (((~AUDFS_768KHz) << 4) & 0xF0) | 0xB;
		setHDMITX_ChStat(it6161, pIEC60958ChStat);
		setHDMITX_HBRAudio(bAudInterface);
		break;
	case T_AUDIO_DSD:
		DRM_DEV_DEBUG_DRIVER(dev, "T_AUDIO_DSD\n");
		setHDMITX_DSDAudio();
		break;
	case T_AUDIO_NLPCM:
		DRM_DEV_DEBUG_DRIVER(dev, "T_AUDIO_NLPCM\n");
		pIEC60958ChStat[0] |= 1 << 1;
		setHDMITX_ChStat(it6161, pIEC60958ChStat);
		setHDMITX_NLPCMAudio(bAudInterface);
		break;
	case T_AUDIO_LPCM:
		DRM_DEV_DEBUG_DRIVER(dev, "T_AUDIO_LPCM\n");
		pIEC60958ChStat[0] &= ~(1 << 1);

		setHDMITX_ChStat(it6161, pIEC60958ChStat);
		setHDMITX_LPCMAudio((ChNum + 1) / 2, SUPPORT_AUDI_AudSWL, bAudInterface);
		break;
	}
	it6161_hdmi_tx_set_bits(it6161, REG_TX_INT_MASK1, B_TX_AUDIO_OVFLW_MASK, 0x00);
	it6161_hdmi_tx_write(it6161, REG_TX_AUDIO_CTRL0, it6161->bAudioChannelEnable);

	it6161_hdmi_tx_set_bits(it6161, REG_TX_SW_RST, (B_HDMITX_AUD_RST | B_TX_AREF_RST), 0);
}

static void hdmi_audio_info_frame_set(struct it6161 *it6161, u8 channels)
{
	struct hdmi_audio_infoframe frame;
	u8 buf[16];
	int ret;

	hdmi_audio_infoframe_init(&frame);

	frame.channels = channels;
	frame.coding_type = HDMI_AUDIO_CODING_TYPE_STREAM;

	ret = hdmi_audio_infoframe_pack(&frame, buf, sizeof(buf));
	if (ret < 0) {
		DRM_ERROR("failed to pack audio infoframe: %d\n", ret);
		return;
	}

	/* set audio Data byte */
	it6161_hdmi_tx_change_bank(it6161, 1);
	it6161_hdmi_tx_write(it6161, REG_TX_PKT_AUDINFO_SUM, buf[3]);
	it6161_hdmi_tx_write(it6161, REG_TX_PKT_AUDINFO_CC, buf[4]);
	it6161_hdmi_tx_write(it6161, REG_TX_PKT_AUDINFO_SF, buf[5]);
	it6161_hdmi_tx_write(it6161, REG_TX_PKT_AUDINFO_CA, buf[7]);
	it6161_hdmi_tx_write(it6161, REG_TX_PKT_AUDINFO_DM_LSV, buf[8]);

	/* Enable Audio info frame */
	it6161_hdmi_tx_change_bank(it6161, 0);
	it6161_hdmi_tx_write(it6161, REG_TX_AUD_INFOFRM_CTRL, B_TX_ENABLE_PKT | B_TX_REPEAT_PKT);
}

static void hdmi_tx_audio_process(struct it6161 *it6161)
{
	if (it6161->support_audio) {
		hdmi_audio_info_frame_set(it6161, (u8) OUTPUT_CHANNEL);

		HDMITX_EnableAudioOutput(it6161,
						CNOFIG_INPUT_AUDIO_TYPE,
						CONFIG_INPUT_AUDIO_INTERFACE,
						INPUT_SAMPLE_FREQ,
						OUTPUT_CHANNEL,
						NULL);
	}
}

static inline void hdmi_tx_disable_avi_infoframe(struct it6161 *it6161)
{
	it6161_hdmi_tx_change_bank(it6161, 0);
	it6161_hdmi_tx_write(it6161, REG_TX_AVI_INFOFRM_CTRL, 0x00);
}

static inline void hdmi_tx_enable_avi_infoframe(struct it6161 *it6161)
{
	it6161_hdmi_tx_change_bank(it6161, 0);
	it6161_hdmi_tx_write(it6161, REG_TX_AVI_INFOFRM_CTRL,
			     B_TX_ENABLE_PKT | B_TX_REPEAT_PKT);
}

static int hdmi_tx_avi_infoframe_set(struct it6161 *it6161)
{
	struct device *dev = &it6161->i2c_hdmi_tx->dev;
	struct hdmi_avi_infoframe *frame = &it6161->source_avi_infoframe;
	struct drm_display_mode *display_mode = &it6161->display_mode;
	u8 buf[32], i, *ptr;
	int ret;

	DRM_DEV_DEBUG_DRIVER(dev, "avinfo set\n");
	hdmi_tx_disable_avi_infoframe(it6161);

	ret = drm_hdmi_avi_infoframe_from_display_mode(
					frame, &it6161->connector, display_mode);
	if (ret) {
		DRM_DEV_ERROR(dev, "Failed to setup AVI infoframe: %d", ret);
		return ret;
	}

	if ((it6161->hdmi_tx_output_color_space & F_MODE_CLRMOD_MASK) == F_MODE_RGB444)
		frame->colorspace = HDMI_COLORSPACE_RGB;

	if ((it6161->hdmi_tx_output_color_space & F_MODE_CLRMOD_MASK) == F_MODE_YUV444)
		frame->colorspace = HDMI_COLORSPACE_YUV444;

	if ((it6161->hdmi_tx_output_color_space & F_MODE_CLRMOD_MASK) == F_MODE_YUV422)
		frame->colorspace = HDMI_COLORSPACE_YUV422;

	ret = hdmi_avi_infoframe_pack(&it6161->source_avi_infoframe, buf, sizeof(buf));
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to pack AVI infoframe: %d", ret);
		return ret;
	}

	/* fill PB */
	it6161_hdmi_tx_change_bank(it6161, 1);
	ptr = buf + HDMI_INFOFRAME_HEADER_SIZE;
	for (i = 0; i < it6161->source_avi_infoframe.length; i++)
		it6161_hdmi_tx_write(it6161, REG_TX_AVIINFO_DB1 + i, ptr[i]);

	it6161_hdmi_tx_write(it6161, REG_TX_AVIINFO_SUM, buf[3]);

	/* Enable */
	hdmi_tx_enable_avi_infoframe(it6161);
	return 0;
}

static void hdmi_tx_set_output_process(struct it6161 *it6161)
{
	struct device *dev = &it6161->i2c_hdmi_tx->dev;
	u8 level;
	u32 TMDSClock;

	DRM_DEV_DEBUG_DRIVER(dev, "hdmi tx set\n");

	TMDSClock = it6161->hdmi_tx_pclk * 1000 *
	    (it6161->source_avi_infoframe.pixel_repeat + 1);

	HDMITX_DisableAudioOutput(it6161);
	hdmi_tx_disable_avi_infoframe(it6161);

	if (TMDSClock > 80000000L)
		level = PCLK_HIGH;
	else if (TMDSClock > 20000000L)
		level = PCLK_MEDIUM;
	else
		level = PCLK_LOW;

	hdmi_tx_enable_video_output(it6161, level);

	if (it6161->hdmi_mode) {
		hdmi_tx_avi_infoframe_set(it6161);
		hdmi_tx_audio_process(it6161);
	}

	it6161_hdmi_tx_set_av_mute(it6161, false);
}

static void mipi_rx_calc_rclk(struct it6161 *it6161)
{
	struct device *dev = &it6161->i2c_mipi_rx->dev;
	u32 sum = 0, i, retry = 5;
	int t10usint;

	for (i = 0; i < retry; i++) {
		/* Enable RCLK 100ms count */
		it6161_mipi_rx_set_bits(it6161, 0x94, 0x80, 0x80);
		msleep(100);
		/* Disable RCLK 100ms count */
		it6161_mipi_rx_set_bits(it6161, 0x94, 0x80, 0x00);

		it6161->mipi_rx_rclk = it6161_mipi_rx_read(it6161, 0x97);
		it6161->mipi_rx_rclk <<= 8;
		it6161->mipi_rx_rclk += it6161_mipi_rx_read(it6161, 0x96);
		it6161->mipi_rx_rclk <<= 8;
		it6161->mipi_rx_rclk += it6161_mipi_rx_read(it6161, 0x95);
		sum += it6161->mipi_rx_rclk;
	}

	sum /= retry;
	it6161->mipi_rx_rclk = sum / 108;
	t10usint = it6161->mipi_rx_rclk;
	DRM_DEV_DEBUG_DRIVER(dev, "mipi_rx_rclk = %d,%03d,%03d\n",
				(sum * 10) / 1000000, ((sum * 10) % 1000000) / 1000, ((sum * 10) % 100));
	it6161_mipi_rx_write(it6161, 0x91, t10usint & 0xFF);
}

static void mipi_rx_reset_p_domain(struct it6161 *it6161)
{
	/* Video Clock Domain Reset */
	it6161_mipi_rx_set_bits(it6161, 0x05, 0x04, 0x04);
	/* Release Video Clock Domain Reset */
	it6161_mipi_rx_set_bits(it6161, 0x05, 0x04, 0x00);
}

static void it6161_mipi_rx_interrupt_clear(struct it6161 *it6161, u8 reg06, u8 reg07, u8 reg08)
{
	it6161_mipi_rx_write(it6161, 0x06, reg06);
	it6161_mipi_rx_write(it6161, 0x07, reg07);
	it6161_mipi_rx_write(it6161, 0x08, reg08);
}

static void it6161_mipi_rx_interrupt_reg06_process(struct it6161 *it6161, u8 reg06)
{
	struct device *dev = &it6161->i2c_mipi_rx->dev;
	bool m_video_stable, p_video_stable;
	u8 data_id;

	if (reg06 & 0x01) {
		m_video_stable = mipi_rx_get_m_video_stable(it6161);
		DRM_DEV_DEBUG_DRIVER(dev, "PPS M video stable Change Interrupt, %sstable",
					m_video_stable ? "" : "un");

		if (m_video_stable) {
			data_id = it6161_mipi_rx_read(it6161, 0x28);
			DRM_DEV_DEBUG_DRIVER(dev, "mipi receive video format: 0x%02x", data_id);
			mipi_rx_calc_rclk(it6161);
			mipi_rx_afe_configuration(it6161, data_id);
			mipi_rx_reset_p_domain(it6161);
		}
	} else if (reg06 & 0x10) {

		p_video_stable = mipi_rx_get_p_video_stable(it6161);
		DRM_DEV_DEBUG_DRIVER(dev, "PPS P video stable Change Interrupt, %sstable", p_video_stable ? "" : "un");
		if (p_video_stable) {
			DRM_DEV_DEBUG_DRIVER(dev, "PVidStb Change to HIGH");
			mipi_rx_calc_rclk(it6161);

			it6161_mipi_rx_write(it6161, 0xC0, (EnTxCRC << 7) + TxCRCnum);
			/* setup 1 sec timer interrupt */
			it6161_mipi_rx_set_bits(it6161, 0x0b, 0x40, 0x40);

			switch (it6161->hdmi_tx_mode) {
			case HDMI_TX_BY_PASS:
				it6161_hdmi_tx_set_bits(it6161, 0xA9, 0x80, 0x80);
				break;

			case HDMI_TX_ENABLE_DE_ONLY:
				hdmi_tx_generate_blank_timing(it6161);
				break;

			default:
				DRM_DEV_ERROR(dev, "use hdmi tx normal mode");
				break;
			}

			hdmi_tx_video_reset(it6161);
		}
	} else
		DRM_DEV_DEBUG_DRIVER(dev, "MIPI Rx int reg06=0x%x\n", reg06);
}

static void it6161_mipi_rx_interrupt_reg07_process(struct it6161 *it6161, u8 reg07)
{
	struct device *dev = &it6161->i2c_mipi_rx->dev;

	if (reg07 & 0x40) {
		DRM_DEV_DEBUG_DRIVER(dev, "PPS FIFO over read Interrupt !!! tx video statle:%d",
						hdmi_tx_get_video_state(it6161));
		it6161_mipi_rx_set_bits(it6161, 0x07, 0x40, 0x40);
	} else if (reg07 & 0x80) {
		DRM_DEV_DEBUG_DRIVER(dev, "PPS FIFO over write Interrupt !!!\n");
		it6161_mipi_rx_set_bits(it6161, 0x07, 0x80, 0x80);
	} else
		DRM_DEV_DEBUG_DRIVER(dev, "MIPI Rx int reg07=0x%x\n", reg07);
}

static void it6161_mipi_rx_interrupt_reg08_process(struct it6161 *it6161, u8 reg08)
{
	struct device *dev = &it6161->i2c_mipi_rx->dev;
	int crc;

	if (reg08 & 0x40) {
		it6161_mipi_rx_set_bits(it6161, 0x0b, 0x40, 0x00);

		if ((it6161_mipi_rx_read(it6161, 0xC1) & 0x03) == 0x03)
			DRM_DEV_DEBUG_DRIVER(dev, "CRC Fail !!!\n");

		if ((it6161_mipi_rx_read(it6161, 0xC1) & 0x05) == 0x05) {
			DRM_DEV_DEBUG_DRIVER(dev, "CRC Pass !!!\n");
			crc = it6161_mipi_rx_read(it6161, 0xC2) +
			    (it6161_mipi_rx_read(it6161, 0xC3) << 8);
			DRM_DEV_DEBUG_DRIVER(dev, "CRCR = 0x%x !!!\n", crc);
			crc = it6161_mipi_rx_read(it6161, 0xC4) +
			    (it6161_mipi_rx_read(it6161, 0xC5) << 8);
			DRM_DEV_DEBUG_DRIVER(dev, "CRCG = 0x%x !!!\n", crc);
			crc = it6161_mipi_rx_read(it6161, 0xC6) +
			    (it6161_mipi_rx_read(it6161, 0xC7) << 8);
			DRM_DEV_DEBUG_DRIVER(dev, "CRCB = 0x%x !!!\n", crc);
		}
	} else
		DRM_DEV_DEBUG_DRIVER(dev, "MIPI Rx int reg08=0x%x\n", reg08);
}

static void it6161_hdmi_tx_interrupt_clear(struct it6161 *it6161, u8 reg06, u8 reg08)
{
	struct device *dev = &it6161->i2c_hdmi_tx->dev;
	u8 int_clear;

	if (reg06 & B_TX_INT_AUD_OVERFLOW) {
		DRM_DEV_ERROR(dev, "B_TX_INT_AUD_OVERFLOW");
		it6161_hdmi_tx_set_bits(it6161, REG_TX_SW_RST,
					(B_HDMITX_AUD_RST | B_TX_AREF_RST),
					(B_HDMITX_AUD_RST | B_TX_AREF_RST));
		it6161_hdmi_tx_set_bits(it6161, REG_TX_SW_RST, B_HDMITX_AUD_RST | B_TX_AREF_RST, 0x00);
	} else if (reg06 & B_TX_INT_DDCFIFO_ERR) {
		DRM_DEV_ERROR(dev, "DDC FIFO Error");
		it6161_hdmi_tx_clear_ddc_fifo(it6161);
	} else if (reg06 & B_TX_INT_DDC_BUS_HANG) {
		DRM_DEV_ERROR(dev, "DDC BUS HANG");
		it6161_hdmi_tx_abort_ddc(it6161);
	}

	/* clear interrupt */
	it6161_hdmi_tx_write(it6161, REG_TX_INT_CLR0, 0xFF);
	it6161_hdmi_tx_write(it6161, REG_TX_INT_CLR1, 0xFF);
	/* write B_TX_INTACTDONE '1' to trigger clear interrupt */
	int_clear = (it6161_hdmi_tx_read(it6161, REG_TX_SYS_STATUS)) | B_TX_CLR_AUD_CTS | B_TX_INTACTDONE;
	it6161_hdmi_tx_write(it6161, REG_TX_SYS_STATUS, int_clear);
}

static void it6161_hdmi_tx_interrupt_reg06_process(struct it6161 *it6161, u8 reg06)
{
	struct device *dev = &it6161->i2c_hdmi_tx->dev;

	if (reg06 & B_TX_INT_HPD_PLUG) {
		/*
		 * sometimes the interrupt is triggered before init bridge.dev.
		 * So avoid null pointer
		 */
		if (it6161->bridge.dev)
			drm_helper_hpd_irq_event(it6161->bridge.dev);
		if (hdmi_tx_get_sink_hpd(it6161)) {
			DRM_DEV_INFO(dev, "HDMI Cable Plug In\n");
			hdmi_tx_video_reset(it6161);
		} else {
			DRM_DEV_INFO(dev, "HDMI Cable Plug Out");
			hdmi_tx_disable_video_output(it6161);
		}
	}
}

static void it6161_hdmi_tx_interrupt_reg08_process(struct it6161 *it6161, u8 reg08)
{
	if (reg08 & B_TX_INT_VIDSTABLE) {
		it6161_hdmi_tx_write(it6161, REG_TX_INT_STAT3, reg08);
		if (hdmi_tx_get_video_state(it6161)) {
			hdmi_tx_set_output_process(it6161);
			it6161_hdmi_tx_set_av_mute(it6161, FALSE);
		}
	}
}

static irqreturn_t it6161_intp_threaded_handler(int unused, void *data)
{
	struct it6161 *it6161 = data;
	struct device *dev = &it6161->i2c_hdmi_tx->dev;
	u8 mipi_rx_reg06, mipi_rx_reg07, mipi_rx_reg08, mipi_rx_reg0d;
	u8 hdmi_tx_reg06, hdmi_tx_reg08;

	mipi_rx_reg06 = it6161_mipi_rx_read(it6161, 0x06);
	mipi_rx_reg07 = it6161_mipi_rx_read(it6161, 0x07);
	mipi_rx_reg08 = it6161_mipi_rx_read(it6161, 0x08);
	mipi_rx_reg0d = it6161_mipi_rx_read(it6161, 0x0D);

	hdmi_tx_reg06 = it6161_hdmi_tx_read(it6161, 0x06);
	hdmi_tx_reg08 = it6161_hdmi_tx_read(it6161, 0x08);

	if ((mipi_rx_reg06 != 0) || (mipi_rx_reg07 != 0) || (mipi_rx_reg08 != 0)) {
		DRM_DEV_DEBUG_DRIVER(dev, "[MIPI rx ++] reg06: 0x%02x reg07: 0x%02x reg08: 0x%02x reg0d: 0x%02x",
		     mipi_rx_reg06, mipi_rx_reg07, mipi_rx_reg08, mipi_rx_reg0d);
		it6161_mipi_rx_interrupt_clear(it6161, mipi_rx_reg06, mipi_rx_reg07, mipi_rx_reg08);
	}

	if ((hdmi_tx_reg06 != 0) || (hdmi_tx_reg08 != 0)) {
		DRM_DEV_DEBUG_DRIVER(dev, "[HDMI tx ++] reg06: 0x%02x reg08: 0x%02x",
		     hdmi_tx_reg06, hdmi_tx_reg08);
		it6161_hdmi_tx_interrupt_clear(it6161, hdmi_tx_reg06, hdmi_tx_reg08);
	}

	it6161_mipi_rx_interrupt_reg08_process(it6161, mipi_rx_reg08);
	it6161_mipi_rx_interrupt_reg06_process(it6161, mipi_rx_reg06);
	it6161_mipi_rx_interrupt_reg07_process(it6161, mipi_rx_reg07);
	it6161_hdmi_tx_interrupt_reg06_process(it6161, hdmi_tx_reg06);
	it6161_hdmi_tx_interrupt_reg08_process(it6161, hdmi_tx_reg08);

	return IRQ_HANDLED;
}

static ssize_t hdmi_output_color_space_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	struct it6161 *it6161 = dev_get_drvdata(dev);

	DRM_DEV_DEBUG_DRIVER(dev, "config color space: %s", buf);
	it6161->hdmi_tx_output_color_space &= ~F_MODE_CLRMOD_MASK;

	if (strncmp(buf, "ycbcr444", strlen(buf) - 1) == 0
	    || strncmp(buf, "yuv444", strlen(buf) - 1) == 0) {
		it6161->hdmi_tx_output_color_space |= F_MODE_YUV444;
		goto end;
	}

	if (strncmp(buf, "ycbcr422", strlen(buf) - 1) == 0
	    || strncmp(buf, "yuv422", strlen(buf) - 1) == 0) {
		it6161->hdmi_tx_output_color_space |= F_MODE_YUV422;
		goto end;
	}

	if (strncmp(buf, "rgb444", strlen(buf) - 1) == 0) {
		it6161->hdmi_tx_output_color_space |= F_MODE_RGB444;
		goto end;
	}

	DRM_DEV_DEBUG_DRIVER(dev,
				"not support this color space, only support ycbcr444/yuv444, ycbcr422/yuv422, rgb444");
	return count;

end:
	DRM_DEV_INFO(dev, "nconfig color space: %s value:0x%02x", buf,
		 it6161->hdmi_tx_output_color_space);
	return count;
}

static ssize_t hdmi_output_color_space_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct it6161 *it6161 = dev_get_drvdata(dev);
	char *str = buf, *end = buf + PAGE_SIZE;

	str += scnprintf(str, end - str,
			"it6161->hdmi_tx_output_color_space:%d\n", it6161->hdmi_tx_output_color_space);

	return str - buf;
}

static DEVICE_ATTR_RW(hdmi_output_color_space);

static const struct attribute *it6161_attrs[] = {
	&dev_attr_hdmi_output_color_space.attr,
	NULL,
};

static int it6161_parse_dt(struct it6161 *it6161, struct device_node *np)
{
	struct device *dev = &it6161->i2c_mipi_rx->dev;

	it6161->host_node = of_graph_get_remote_node(np, 0, 0);
	if (!it6161->host_node) {
		DRM_DEV_ERROR(dev, "no host node");
		return -ENODEV;
	}
	of_node_put(it6161->host_node);

	return 0;
}

static int it6161_i2c_probe(struct i2c_client *i2c_mipi_rx)
{
	struct device *dev = &i2c_mipi_rx->dev;
	int err, intp_irq;

	it6161 = devm_kzalloc(dev, sizeof(*it6161), GFP_KERNEL);
	if (!it6161)
		return -ENOMEM;

	it6161->i2c_mipi_rx = i2c_mipi_rx;
	mutex_init(&it6161->mode_lock);

	it6161->bridge.of_node = i2c_mipi_rx->dev.of_node;

	it6161_parse_dt(it6161, dev->of_node);
	it6161->regmap_mipi_rx = devm_regmap_init_i2c(i2c_mipi_rx, &it6161_mipi_rx_bridge_regmap_config);
	if (IS_ERR(it6161->regmap_mipi_rx)) {
		DRM_DEV_ERROR(dev, "regmap_mipi_rx i2c init failed");
		return PTR_ERR(it6161->regmap_mipi_rx);
	}

	if (device_property_read_u32(dev, "it6161-addr-hdmi-tx", &it6161->it6161_addr_hdmi_tx) < 0)
		it6161->it6161_addr_hdmi_tx = 0x4C;
	it6161->i2c_hdmi_tx = i2c_new_dummy_device(i2c_mipi_rx->adapter, it6161->it6161_addr_hdmi_tx);
	it6161->regmap_hdmi_tx = devm_regmap_init_i2c(it6161->i2c_hdmi_tx, &it6161_hdmi_tx_bridge_regmap_config);
	if (IS_ERR(it6161->regmap_hdmi_tx)) {
		DRM_DEV_ERROR(dev, "regmap_hdmi_tx i2c init failed");
		return PTR_ERR(it6161->regmap_hdmi_tx);
	}

	if (!it6161_check_device_ready(it6161))
		return -ENODEV;

	/* The enable GPIO is optional. */
	it6161->enable_gpio = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(it6161->enable_gpio))
		DRM_DEV_INFO(dev, "No enable GPIO");
	else
		gpiod_set_value_cansleep(it6161->enable_gpio, 1);

	it6161->enable_drv_hold = DEFAULT_DRV_HOLD;
	it6161_set_interrupts_active_level(HIGH);

	intp_irq = i2c_mipi_rx->irq;

	if (!intp_irq) {
		DRM_DEV_ERROR(dev, "it6112 failed to get INTP IRQ");
		return -ENODEV;
	}

	err = devm_request_threaded_irq(&i2c_mipi_rx->dev, intp_irq, NULL,
					it6161_intp_threaded_handler,
					IRQF_TRIGGER_HIGH | IRQF_ONESHOT, "it6161-intp", it6161);
	if (err) {
		DRM_DEV_ERROR(dev, "it6112 failed to request INTP threaded IRQ: %d", err);
		return err;
	}

	i2c_set_clientdata(i2c_mipi_rx, it6161);
	it6161->bridge.funcs = &it6161_bridge_funcs;
	it6161->bridge.of_node = i2c_mipi_rx->dev.of_node;
	it6161->bridge.ops = DRM_BRIDGE_OP_DETECT | DRM_BRIDGE_OP_EDID |
	    DRM_BRIDGE_OP_HPD | DRM_BRIDGE_OP_MODES;
	it6161->bridge.type = DRM_MODE_CONNECTOR_HDMIA;

	drm_bridge_add(&it6161->bridge);

	err = sysfs_create_files(&i2c_mipi_rx->dev.kobj, it6161_attrs);
	if (err)
		return err;

	return 0;
}

static void it6161_remove(struct i2c_client *i2c_mipi_rx)
{
	struct it6161 *it6161 = i2c_get_clientdata(i2c_mipi_rx);

	drm_connector_unregister(&it6161->connector);
	drm_connector_cleanup(&it6161->connector);
	drm_bridge_remove(&it6161->bridge);
}

static const struct i2c_device_id it6161_id[] = {
	{"it6161", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, it6161_id);

static const struct of_device_id it6161_of_match[] = {
	{.compatible = "ite,it6161"},
	{}
};

static struct i2c_driver it6161_i2c_driver = {
	.driver = {
		.name = "it6161_mipirx_hdmitx",
		.of_match_table = it6161_of_match,
	},
	.id_table = it6161_id,
	.probe = it6161_i2c_probe,
	.remove = it6161_remove,
};

module_i2c_driver(it6161_i2c_driver);

MODULE_AUTHOR("allen chen <allen.chen@ite.com.tw>");
MODULE_DESCRIPTION("it6161 HDMI Transmitter driver");
MODULE_LICENSE("GPL v2");
