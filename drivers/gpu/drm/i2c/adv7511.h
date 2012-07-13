/**
 * Analog Devices ADV7511 HDMI transmitter driver
 *
 * Copyright 2012 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#ifndef __ADV7511_H__
#define __ADV7511_H__

#define ADV7511_REG_CHIP_REVISION		0x00
#define ADV7511_REG_N0				0x01
#define ADV7511_REG_N1				0x02
#define ADV7511_REG_N2				0x03
#define ADV7511_REG_SPDIF_FREQ			0x04
#define ADV7511_REG_CTS_AUTOMATIC1		0x05
#define ADV7511_REG_CTS_AUTOMATIC2		0x06
#define ADV7511_REG_CTS_MANUAL0			0x07
#define ADV7511_REG_CTS_MANUAL1			0x08
#define ADV7511_REG_CTS_MANUAL2			0x09
#define ADV7511_REG_AUDIO_SOURCE		0x0a
#define ADV7511_REG_AUDIO_CONFIG		0x0b
#define ADV7511_REG_I2S_CONFIG			0x0c
#define ADV7511_REG_I2S_WIDTH			0x0d
#define ADV7511_REG_AUDIO_SUB_SRC0		0x0e
#define ADV7511_REG_AUDIO_SUB_SRC1		0x0f
#define ADV7511_REG_AUDIO_SUB_SRC2		0x10
#define ADV7511_REG_AUDIO_SUB_SRC3		0x11
#define ADV7511_REG_AUDIO_CFG1			0x12
#define ADV7511_REG_AUDIO_CFG2			0x13
#define ADV7511_REG_AUDIO_CFG3			0x14
#define ADV7511_REG_I2C_FREQ_ID_CFG		0x15
#define ADV7511_REG_VIDEO_INPUT_CFG1		0x16
#define ADV7511_REG_CSC_UPPER(x)		(0x18 + (x) * 2)
#define ADV7511_REG_CSC_LOWER(x)		(0x19 + (x) * 2)
#define ADV7511_REG_SYNC_DECODER(x)		(0x30 + (x))
#define ADV7511_REG_DE_GENERATOR		(0x35 + (x))
#define ADV7511_REG_PIXEL_REPETITION		0x3b
#define ADV7511_REG_VIC_MANUAL			0x3c
#define ADV7511_REG_VIC_SEND			0x3d
#define ADV7511_REG_VIC_DETECTED		0x3e
#define ADV7511_REG_AUX_VIC_DETECTED		0x3f
#define ADV7511_REG_PACKET_ENABLE0		0x40
#define ADV7511_REG_POWER			0x41
#define ADV7511_REG_STATUS			0x42
#define ADV7511_REG_EDID_I2C_ADDR		0x43
#define ADV7511_REG_PACKET_ENABLE1		0x44
#define ADV7511_REG_PACKET_I2C_ADDR		0x45
#define ADV7511_REG_DSD_ENABLE			0x46
#define ADV7511_REG_VIDEO_INPUT_CFG2		0x48
#define ADV7511_REG_GC(x)			(0x4b + (x)) /* 0x4b - 0x51 */
#define ADV7511_REG_AVI_INFOFRAME_VERSION	0x52
#define ADV7511_REG_AVI_INFOFRAME_LENGTH	0x53
#define ADV7511_REG_AVI_INFOFRAME_CHECKSUM	0x54
#define ADV7511_REG_AVI_INFOFRAME(x)		(0x55 + (x)) /* 0x55 - 0x6f */
#define ADV7511_REG_AUDIO_INFOFRAME_VERSION	0x70
#define ADV7511_REG_AUDIO_INFOFRAME_LENGTH	0x71
#define ADV7511_REG_AUDIO_INFOFRAME_CHECKSUM	0x72
#define ADV7511_REG_AUDIO_INFOFRAME(x)		(0x73 + (x)) /* 0x73 - 0x7c */
#define ADV7511_REG_INT_ENABLE(x)		(0x94 + (x))
#define ADV7511_REG_INT(x)			(0x96 + (x))
#define ADV7511_REG_INPUT_CLK_DIV		0x9d
#define ADV7511_REG_PLL_STATUS			0x9e
#define ADV7511_REG_HDMI_POWER			0xa1
#define ADV7511_REG_HDCP_HDMI_CFG		0xaf
#define ADV7511_REG_AN(x)			(0xb0 + (x)) /* 0xb0 - 0xb7 */
#define ADV7511_REG_HDCP_STATUS			0xb8
#define ADV7511_REG_BCAPS			0xbe
#define ADV7511_REG_BKSV(x)			(0xc0 + (x)) /* 0xc0 - 0xc3 */
#define ADV7511_REG_EDID_SEGMENT		0xc4
#define ADV7511_REG_DDC_STATUS			0xc8
#define ADV7511_REG_EDID_READ_CTRL		0xc9
#define ADV7511_REG_BSTATUS(x)			(0xca + (x)) /* 0xca - 0xcb */
#define ADV7511_REG_TIMING_GEN_SEQ		0xd0
#define ADV7511_REG_POWER2			0xd6
#define ADV7511_REG_HSYNC_PLACEMENT_MSB		0xfa

#define ADV7511_REG_SYNC_ADJUSTMENT(x)		(0xd7 + (x)) /* 0xd7 - 0xdc */
#define ADV7511_REG_TMDS_CLOCK_INV		0xde
#define ADV7511_REG_ARC_CTRL			0xdf
#define ADV7511_REG_CEC_I2C_ADDR		0xe1
#define ADV7511_REG_CEC_CTRL			0xe2
#define ADV7511_REG_CHIP_ID_HIGH		0xf5
#define ADV7511_REG_CHIP_ID_LOW			0xf6

#define ADV7511_CSC_ENABLE			BIT(7)
#define ADV7511_CSC_UPDATE_MODE			BIT(5)

#define ADV7511_INT0_HDP			BIT(7)
#define ADV7511_INT0_VSYNC			BIT(5)
#define ADV7511_INT0_AUDIO_FIFO_FULL		BIT(4)
#define ADV7511_INT0_EDID_READY			BIT(2)
#define ADV7511_INT0_HDCP_AUTHENTICATED		BIT(1)

#define ADV7511_INT1_DDC_ERROR			BIT(7)
#define ADV7511_INT1_BKSV			BIT(6)
#define ADV7511_INT1_CEC_TX_READY		BIT(5)
#define ADV7511_INT1_CEC_TX_ARBIT_LOST		BIT(4)
#define ADV7511_INT1_CEC_TX_RETRY_TIMEOUT	BIT(3)
#define ADV7511_INT1_CEC_RX_READY3		BIT(2)
#define ADV7511_INT1_CEC_RX_READY2		BIT(1)
#define ADV7511_INT1_CEC_RX_READY1		BIT(0)

#define ADV7511_ARC_CTRL_POWER_DOWN		BIT(0)

#define ADV7511_CEC_CTRL_POWER_DOWN		BIT(0)

#define ADV7511_POWER_POWER_DOWN		BIT(6)

#define ADV7511_AUDIO_SELECT_I2C		0x0
#define ADV7511_AUDIO_SELECT_SPDIF		0x1
#define ADV7511_AUDIO_SELECT_DSD		0x2
#define ADV7511_AUDIO_SELECT_HBR		0x3
#define ADV7511_AUDIO_SELECT_DST		0x4

#define ADV7511_I2S_SAMPLE_LEN_16		0x2
#define ADV7511_I2S_SAMPLE_LEN_20		0x3
#define ADV7511_I2S_SAMPLE_LEN_18		0x4
#define ADV7511_I2S_SAMPLE_LEN_22		0x5
#define ADV7511_I2S_SAMPLE_LEN_19		0x8
#define ADV7511_I2S_SAMPLE_LEN_23		0x9
#define ADV7511_I2S_SAMPLE_LEN_24		0xb
#define ADV7511_I2S_SAMPLE_LEN_17		0xc
#define ADV7511_I2S_SAMPLE_LEN_21		0xd

#define ADV7511_SAMPLE_FREQ_44100		0x0
#define ADV7511_SAMPLE_FREQ_48000		0x2
#define ADV7511_SAMPLE_FREQ_32000		0x3
#define ADV7511_SAMPLE_FREQ_88200		0x8
#define ADV7511_SAMPLE_FREQ_96000		0xa
#define ADV7511_SAMPLE_FREQ_176400		0xc
#define ADV7511_SAMPLE_FREQ_192000		0xe

#define ADV7511_STATUS_POWER_DOWN_POLARITY	BIT(7)
#define ADV7511_STATUS_HPD			BIT(6)
#define ADV7511_STATUS_MONITOR_SENSE		BIT(5)
#define ADV7511_STATUS_I2S_32BIT_MODE		BIT(3)

#define ADV7511_PACKET_ENABLE_N_CTS		BIT(8+6)
#define ADV7511_PACKET_ENABLE_AUDIO_SAMPLE	BIT(8+5)
#define ADV7511_PACKET_ENABLE_AVI_INFOFRAME	BIT(8+4)
#define ADV7511_PACKET_ENABLE_AUDIO_INFOFRAME	BIT(8+3)
#define ADV7511_PACKET_ENABLE_GC		BIT(7)
#define ADV7511_PACKET_ENABLE_SPD		BIT(6)
#define ADV7511_PACKET_ENABLE_MPEG		BIT(5)
#define ADV7511_PACKET_ENABLE_ACP		BIT(4)
#define ADV7511_PACKET_ENABLE_ISRC		BIT(3)
#define ADV7511_PACKET_ENABLE_GM		BIT(2)
#define ADV7511_PACKET_ENABLE_SPARE2		BIT(1)
#define ADV7511_PACKET_ENABLE_SPARE1		BIT(0)

#define ADV7511_REG_POWER2_HDP_SRC_MASK		0xc0
#define ADV7511_REG_POWER2_HDP_SRC_BOTH		0x00
#define ADV7511_REG_POWER2_HDP_SRC_HDP		0x40
#define ADV7511_REG_POWER2_HDP_SRC_CEC		0x80
#define ADV7511_REG_POWER2_HDP_SRC_NONE		0xc0
#define ADV7511_REG_POWER2_TDMS_ENABLE		BIT(4)
#define ADV7511_REG_POWER2_GATE_INPUT_CLK	BIT(0)

#define ADV7511_LOW_REFRESH_RATE_NONE 0x0
#define ADV7511_LOW_REFRESH_RATE_24HZ 0x1
#define ADV7511_LOW_REFRESH_RATE_25HZ 0x2
#define ADV7511_LOW_REFRESH_RATE_30HZ 0x3

#include <drm/drmP.h>

struct i2c_client;
struct regmap;

struct adv7511 {
	struct i2c_client *i2c_main;
	struct i2c_client *i2c_edid;
	struct i2c_client *i2c_packet;
	struct i2c_client *i2c_cec;

	struct regmap *regmap;
	enum drm_connector_status status;
	int dpms_mode;

	unsigned int f_tmds;
	unsigned int f_audio;

	unsigned int audio_source;

	unsigned int current_edid_segment;
	uint8_t edid_buf[256];

	wait_queue_head_t wq;
	struct drm_encoder *encoder;

	bool embedded_sync;
};

int adv7511_packet_enable(struct adv7511 *adv7511, unsigned int packet);
int adv7511_packet_disable(struct adv7511 *adv7511, unsigned int packet);

int adv7511_audio_init(struct device *dev);
void adv7511_audio_exit(struct device *dev);

enum adv7511_input_id {
	ADV7511_INPUT_ID_24BIT_RGB444_YCbCr444 = 0,
	ADV7511_INPUT_ID_16_20_24BIT_YCbCr422_SEPARATE_SYNC = 1,
	ADV7511_INPUT_ID_16_20_24BIT_YCbCr422_EMBEDDED_SYNC = 2,
	ADV7511_INPUT_ID_8_10_12BIT_YCbCr422_SEPARATE_SYNC = 3,
	ADV7511_INPUT_ID_8_10_12BIT_YCbCr422_EMBEDDED_SYNC = 4,
	ADV7511_INPUT_ID_12_15_16BIT_RGB444_YCbCr444 = 5,
};

enum adv7511_input_color_depth {
	ADV7511_INPUT_COLOR_DEPTH_10BIT = 1,
	ADV7511_INPUT_COLOR_DEPTH_12BIT = 2,
	ADV7511_INPUT_COLOR_DEPTH_8BIT = 3,
};

enum adv7511_input_style {
	ADV7511_INPUT_STYLE2 = 1,
	ADV7511_INPUT_STYLE1 = 2,
	ADV7511_INPUT_STYLE3 = 3,
};

enum adv7511_input_sync_pulse {
	ADV7511_INPUT_SYNC_PULSE_DE = 0,
	ADV7511_INPUT_SYNC_PULSE_HSYNC = 1,
	ADV7511_INPUT_SYNC_PULSE_VSYNC = 2,
	ADV7511_INPUT_SYNC_PULSE_NONE = 3,
};

enum adv7511_input_clock_delay {
	ADV7511_INPUT_CLOCK_DELAY_MINUS_1200PS = 0,
	ADV7511_INPUT_CLOCK_DELAY_MINUS_800PS = 1,
	ADV7511_INPUT_CLOCK_DELAY_MINUS_400PS = 2,
	ADV7511_INPUT_CLOCK_DELAY_NONE = 3,
	ADV7511_INPUT_CLOCK_DELAY_PLUS_400PS = 4,
	ADV7511_INPUT_CLOCK_DELAY_PLUS_800PS = 5,
	ADV7511_INPUT_CLOCK_DELAY_PLUS_1200PS = 6,
	ADV7511_INPUT_CLOCK_DELAY_PLUS_1600PS = 7,
};

enum adv7511_input_bit_justifiction {
	ADV7511_INPUT_BIT_JUSTIFICATION_EVENLY = 0,
	ADV7511_INPUT_BIT_JUSTIFICATION_RIGHT = 1,
	ADV7511_INPUT_BIT_JUSTIFICATION_LEFT = 2,
};

enum adv7511_output_format {
	ADV7511_OUTPUT_FORMAT_RGB_444 = 0,
	ADV7511_OUTPUT_FORMAT_YCBCR_422 = 1,
	ADV7511_OUTPUT_FORMAT_YCBCR_444 = 2,
};

enum adv7511_csc_scaling {
	ADV7511_CSC_SCALING_1 = 0,
	ADV7511_CSC_SCALING_2 = 1,
	ADV7511_CSC_SCALING_4 = 2,
};

/**
 * struct adv7511_video_input_config - Describes adv7511 hardware configuration
 * @id:				Video input format id
 * @input_style:		Video input format style
 * @sync_pulse:			Select the sync pulse
 * @clock_delay:		Clock delay for the input clock
 * @reverse_bitorder:		Reverse video input signal bitorder
 * @bit_justification:		Video input format bit justification
 * @up_conversion_first_order_interpolation:
 * @input_color_depth:		Input video format color depth
 * @tmds_clock_inversion:	Whether to invert the TDMS clock
 * @vsync_polartity_low:	Whether the vsync polarity is low
 * @hsync_polartity_low:	Whether the hsync polarity is low
 * @csc_enable:			Whether to enable color space conversion
 * @csc_scaling_factor:		Color space conversion scaling factor
 * @csc_coefficents:		Color space conversion coefficents
 * @output_format:		Video output format
 * @timing_gen_sequence:
 * @hdmi_mode:			Whether to use HDMI or DVI output mode
 **/
struct adv7511_video_input_config {
	enum adv7511_input_id id;
	enum adv7511_input_style input_style;
	enum adv7511_input_sync_pulse sync_pulse;
	enum adv7511_input_clock_delay clock_delay;
	bool reverse_bitorder;
	enum adv7511_input_bit_justifiction bit_justification;
	bool up_conversion_first_order_interpolation;
	enum adv7511_input_color_depth input_color_depth;
	bool tmds_clock_inversion;

	bool vsync_polartity_low;
	bool hsync_polartity_low;

	bool csc_enable;
	enum adv7511_csc_scaling csc_scaling_factor;
	uint16_t *csc_coefficents;

	enum adv7511_output_format output_format;
	int timing_gen_sequence;
	bool hdmi_mode;
};

#endif
