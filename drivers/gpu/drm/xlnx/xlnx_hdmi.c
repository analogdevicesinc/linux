// SPDX-License-Identifier: GPL-2.0
/*
 * Xilinx FPGA HDMI TX Subsystem Driver
 *
 * Copyright (C) 2021 Xilinx, Inc.
 *
 * Author: Venkateshwar Rao G <vgannava.xilinx.com>
 */

#include <drm/display/drm_dp_helper.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_edid.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_of.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_sysfs.h>

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/delay.h>
#include <linux/hdmi.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/mfd/syscon.h>
#include <linux/of_device.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>
#include <linux/xlnx/xlnx_timer.h>
#include <uapi/linux/media-bus-format.h>

#include "hdcp/xlnx_hdcp_tx.h"
#include "xlnx_bridge.h"

/* Parallel Interface registers */
#define HDMI_TX_PIO_ID				0x40
#define HDMI_TX_PIO_CTRL			0x44
#define HDMI_TX_PIO_CTRL_IE			BIT(1)
#define HDMI_TX_PIO_CTRL_RUN			BIT(0)
#define HDMI_TX_PIO_CTRL_SET			0x48
#define HDMI_TX_PIO_CTRL_CLR			0x4c
#define HDMI_TX_PIO_STA				0x50
#define HDMI_TX_PIO_STA_EVT			BIT(1)
#define HDMI_TX_PIO_STA_IRQ			BIT(0)
#define HDMI_TX_PIO_OUT				0x54
#define HDMI_TX_PIO_OUT_GCP_AVMUTE		BIT(31)
#define HDMI_TX_PIO_OUT_BRIDGE_PIXEL		BIT(30)
#define HDMI_TX_PIO_OUT_BRIDGE_YUV420		BIT(29)
#define HDMI_TX_PIO_OUT_GCP_CLEARAVMUTE		BIT(28)
#define HDMI_TX_PIO_OUT_EXT_SYSRST		BIT(22)
#define HDMI_TX_PIO_OUT_EXT_VRST		BIT(21)
#define HDMI_TX_PIO_OUT_INT_LRST		BIT(20)
#define HDMI_TX_PIO_OUT_SCRM			BIT(12)
#define HDMI_TX_PIO_OUT_CS			GENMASK(11, 10)
#define HDMI_TX_PIO_OUT_SR			GENMASK(9, 8)
#define HDMI_TX_PIO_OUT_PR			GENMASK(7, 6)
#define HDMI_TX_PIO_OUT_CD			GENMASK(5, 4)
#define HDMI_TX_PIO_OUT_CD_SHIFT		4
#define HDMI_TX_PIO_OUT_PR_SHIFT		6
#define HDMI_TX_PIO_OUT_SR_SHIFT		8
#define HDMI_TX_PIO_OUT_CS_SHIFT		10
#define HDMI_TX_PIO_OUT_MODE			BIT(3)
#define HDMI_TX_PIO_OUT_INT_VRST		BIT(0)
#define HDMI_TX_PIO_OUT_SET			0x58
#define HDMI_TX_PIO_OUT_CLR			0x5c
#define HDMI_TX_PIO_OUT_MSK			0x60
#define HDMI_TX_PIO_IN				0x64
#define HDMI_TX_PIO_IN_BRIDGE_UFLOW		BIT(11)
#define HDMI_TX_PIO_IN_BRIDGE_OFLOW		BIT(10)
#define HDMI_TX_PIO_IN_BRIDGE_LOCKED		BIT(9)
#define HDMI_TX_PIO_IN_HPD_TOGGLE		BIT(8)
#define HDMI_TX_PIO_IN_PPP			GENMASK(7, 5)
#define HDMI_TX_PIO_IN_ERR			BIT(4)
#define HDMI_TX_PIO_IN_VS			BIT(3)
#define HDMI_TX_PIO_IN_HPD_CONNECT		BIT(2)
#define HDMI_TX_PIO_IN_VID_RDY			BIT(1)
#define HDMI_TX_PIO_IN_LNK_RDY			BIT(0)
#define HDMI_TX_PIO_LNK_VID_RDY_MASK		(HDMI_TX_PIO_IN_VID_RDY | \
						HDMI_TX_PIO_IN_LNK_RDY)
#define HDMI_TX_PIO_IN_EVT			0x68
#define HDMI_TX_PIO_IN_EVT_RE			0x6c
#define HDMI_TX_PIO_IN_EVT_FE			0x70
#define HDMI_TX_HPD_TIMEGRID			0x74
#define HDMI_TX_HPD_TOGGLE_CONF			0x78
#define HDMI_TX_HPD_CONNECT_CONF		0x7c

/* Display Data Channel registers */
#define HDMI_HDCP_DDC_BASE_OFFSET		0x3a
#define HDMI_TX_DDC_ID				0x80
#define HDMI_TX_DDC_CTRL			0x84
#define HDMI_TX_DDC_CTRL_CLK_DIV		GENMASK(31, 16)
#define HDMI_TX_DDC_CTRL_CLK_DIV_SHIFT		16
#define HDMI_TX_DDC_CTRL_TO_STOP		BIT(2)
#define HDMI_TX_DDC_CTRL_IE			BIT(1)
#define HDMI_TX_DDC_CTRL_RUN			BIT(0)
#define HDMI_TX_DDC_CTRL_SET			0x88
#define HDMI_TX_DDC_CTRL_CLR			0x8c
#define HDMI_TX_DDC_STA				0x90
#define HDMI_TX_DDC_STA_DAT_USED_WRDS		GENMASK(31, 24)
#define HDMI_TX_DDC_STA_CMD_FREE_WRDS		GENMASK(23, 16)
#define HDMI_TX_DDC_STA_DAT_EMPTY		BIT(9)
#define HDMI_TX_DDC_STA_CMD_FULL		BIT(8)
#define HDMI_TX_DDC_STA_SDA			BIT(7)
#define HDMI_TX_DDC_STA_SCL			BIT(6)
#define HDMI_TX_DDC_STA_ACK			BIT(5)
#define HDMI_TX_DDC_STA_TO			BIT(4)
#define HDMI_TX_DDC_STA_DONE			BIT(3)
#define HDMI_TX_DDC_STA_BUSY			BIT(2)
#define HDMI_TX_DDC_STA_EVT			BIT(1)
#define HDMI_TX_DDC_STA_IRQ			BIT(0)
#define HDMI_TX_DDC_CMD				0x94
#define HDMI_TX_DDC_DAT				0x98

/* Auxiliary peripheral registers */
#define HDMI_TX_AUX_ID				0xc0
#define HDMI_TX_AUX_CTRL			0xc4
#define HDMI_TX_AUX_CTRL_IE			BIT(1)
#define HDMI_TX_AUX_CTRL_RUN			BIT(0)
#define HDMI_TX_AUX_CTRL_SET			0xc8
#define HDMI_TX_AUX_CTRL_CLR			0xcc
#define HDMI_TX_AUX_STA				0xd0
#define HDMI_TX_AUX_STA_FREE_PKTS		GENMASK(7, 4)
#define HDMI_TX_AUX_STA_PKT_RDY			BIT(3)
#define HDMI_TX_AUX_STA_FL			BIT(2)
#define HDMI_TX_AUX_STA_EP			BIT(1)
#define HDMI_TX_AUX_STA_IRQ			BIT(0)
#define HDMI_TX_AUX_DAT				0xd4

/* Audio peripheral registers */
#define HDMI_TX_AUD_ID				0x100
#define HDMI_TX_AUD_CTRL			0x104
#define HDMI_TX_AUD_CTRL_AUD_CLK_RATIO		GENMASK(15, 12)
#define HDMI_TX_AUD_CTRL_TMDS_LNKCLK_RATIO	GENMASK(11, 8)
#define HDMI_TX_AUD_CTRL_ACR_SEL		BIT(7)
#define HDMI_TX_AUD_CTRL_ACR_EN			BIT(6)
#define HDMI_TX_AUD_CTRL_AUD_RESET		BIT(5)
#define HDMI_TX_AUD_CTRL_AUD_FMT		BIT(4)
#define HDMI_TX_AUD_CTRL_CH			GENMASK(3, 2)
#define HDMI_TX_AUD_CTRL_IE			BIT(1)
#define HDMI_TX_AUD_CTRL_RUN			BIT(0)
#define HDMI_TX_AUD_CTRL_SET			0x108
#define HDMI_TX_AUD_CTRL_CLR			0x10c
#define HDMI_TX_AUD_STA				0x110
#define HDMI_TX_AUD_ACR_N			0x114
#define HDMI_TX_AUD_ACR_CTS			0x118
#define HDMI_TX_AUD_ACR_CTS_ACR_CTS		GENMASK(19, 0)
#define HDMI_TX_AUD_ACR_CTS_VLD			BIT(31)

/* Video mask peripheral registers */
#define HDMI_TX_VID_MSK_ID			0x140
#define HDMI_TX_VID_MSK_CTRL			0x144
#define HDMI_TX_VID_MSK_CTRL_IE			BIT(1)
#define HDMI_TX_VID_MSK_CTRL_RUN		BIT(0)
#define HDMI_TX_VID_MSK_CTRL_SET		0x148
#define HDMI_TX_VID_MSK_CTRL_CLR		0x14c
#define HDMI_TX_VID_MSK_STA			0x150
#define HDMI_TX_VID_MSK_COMP_RED		0x154
#define HDMI_TX_VID_MSK_COMP_GREEN		0x158
#define HDMI_TX_VID_MSK_COMP_BLUE		0x15c

/* FRL registers */
#define HDMI_TX_FRL_ID				0x180
#define HDMI_TX_FRL_CTRL			0x184
#define HDMI_TX_FRL_CTRL_FRL_VCKE_EXT		BIT(24)
#define HDMI_TX_FRL_CTRL_FRL_LTP3_REQ		GENMASK(23, 20)
#define HDMI_TX_FRL_CTRL_FRL_LTP2_REQ		GENMASK(19, 16)
#define HDMI_TX_FRL_CTRL_FRL_LTP1_REQ		GENMASK(15, 12)
#define HDMI_TX_FRL_CTRL_FRL_LTP0_REQ		GENMASK(11, 8)
#define HDMI_TX_FRL_CTRL_FRL_REQ_MASK		0xF
#define HDMI_TX_FRL_CTRL_FRL_LTP0_SHIFT		8
#define HDMI_TX_FRL_CTRL_FRL_LTP1_SHIFT		12
#define HDMI_TX_FRL_CTRL_FRL_LTP2_SHIFT		16
#define HDMI_TX_FRL_CTRL_FRL_LTP3_SHIFT		20
#define HDMI_TX_FRL_CTRL_FRL_ACT		BIT(7)
#define HDMI_TX_FRL_CTRL_TST_RC_DISABLE		BIT(5)
#define HDMI_TX_FRL_CTRL_EXEC			BIT(4)
#define HDMI_TX_FRL_CTRL_FRL_LN_OP		BIT(3)
#define HDMI_TX_FRL_CTRL_OP_MODE		BIT(2)
#define HDMI_TX_FRL_CTRL_IE			BIT(1)
#define HDMI_TX_FRL_CTRL_RST			BIT(0)
#define HDMI_TX_FRL_CTRL_SET			0x188
#define HDMI_TX_FRL_CTRL_CLR			0x18c
#define HDMI_TX_FRL_STA				0x190
#define HDMI_TX_FRL_STA_GB_SYNC_ERR		BIT(8)
#define HDMI_TX_FRL_STA_GB_EP			BIT(7)
#define HDMI_TX_FRL_STA_VID_CLK_OOS		BIT(6)
#define HDMI_TX_FRL_STA_LNK_CLK_OOS		BIT(5)
#define HDMI_TX_FRL_STA_TRIB_RST		BIT(4)
#define HDMI_TX_FRL_STA_FRL_RST			BIT(3)
#define HDMI_TX_FRL_STA_TMR_ZERO		BIT(2)
#define HDMI_TX_FRL_STA_TMR_EVT			BIT(1)
#define HDMI_TX_FRL_STA_IRQ			BIT(0)
#define HDMI_TX_FRL_TMR				0x194
#define HDMI_TX_FRL_LNK_CLK			0x198
#define HDMI_TX_FRL_VID_CLK			0x19c
#define HDMI_TX_FRL_VP_FIFO_THRD		0x1a0
#define HDMI_TX_DISP_ERR_INJ			0x1a4
#define HDMI_TX_DISP_ERR_INJ_NUM_ERR_CB		GENMASK(31, 16)
#define HDMI_TX_DISP_ERR_INJ_NUM_ERR_CHAR	GENMASK(15, 8)
#define HDMI_TX_DISP_ERR_INJ_ERR_TYPE		GENMASK(6, 4)
#define HDMI_TX_DISP_ERR_INJ_DISP_ERR_INJ_EN	BIT(0)
#define HDMI_TX_FEC_ERR_INJ			0x1a8
#define HDMI_TX_FEC_ERR_INJ_ERR_CB_LOC		GENMASK(25, 16)
#define HDMI_TX_FEC_ERR_INJ_NUM_ERR_CB		GENMASK(15, 8)
#define HDMI_TX_FEC_ERR_INJ_NUM_ERR_CHAR	GENMASK(7, 4)
#define HDMI_TX_FEC_ERR_INJ_FEC_ERR_INJ_EN	BIT(0)
/* VTC register offsets and bit masks */
#define HDMI_TX_VTC_CTL				0x000
#define HDMI_TX_VTC_CTL_MASK			GENMASK(18, 8)
#define HDMI_TX_VTC_RST				BIT(31)
#define HDMI_TX_VTC_CTL_GE			BIT(2)
#define HDMI_TX_VTC_CTL_RU			BIT(1)

#define HDMI_TX_VTC_GASIZE_F0			0x060
#define HDMI_TX_VTC_ACTIVE_SIZE_MASK		GENMASK(13, 0)

#define HDMI_TX_VTC_GFENC			0x068
#define HDMI_TX_VTC_GFENC_MASK			BIT(6)

#define HDMI_TX_VTC_GPOL			0x06c
#define HDMI_TX_VTC_GPOL_FIELD_ID_POL		BIT(6)
#define HDMI_TX_VTC_ACTIVE_CHROMA_POL		BIT(5)
#define HDMI_TX_VTC_ACTIVE_VIDEO_POL		BIT(4)
#define HDMI_TX_VTC_HSYNC_POL			BIT(3)
#define HDMI_TX_VTC_VSYNC_POL			BIT(2)
#define HDMI_TX_VTC_HBLANK_POL			BIT(1)
#define HDMI_TX_VTC_VBLANK_POL			BIT(0)
#define HDMI_TX_VTC_GPOL_MASK		(HDMI_TX_VTC_VBLANK_POL |\
					 HDMI_TX_VTC_HBLANK_POL |\
					 HDMI_TX_VTC_VSYNC_POL |\
					 HDMI_TX_VTC_HSYNC_POL |\
					 HDMI_TX_VTC_ACTIVE_VIDEO_POL |\
					 HDMI_TX_VTC_ACTIVE_CHROMA_POL)

#define HDMI_TX_VTC_INT_GPOL_MASK	(HDMI_TX_VTC_GPOL_FIELD_ID_POL |\
					 HDMI_TX_VTC_ACTIVE_CHROMA_POL |\
					 HDMI_TX_VTC_ACTIVE_VIDEO_POL)

#define HDMI_TX_VTC_GHSIZE			0x070
#define HDMI_TX_VTC_GHSIZE_FRAME_HSIZE		GENMASK(13, 0)

#define HDMI_TX_VTC_GVSIZE			0x074
#define HDMI_TX_VTC_FIELD1_VSIZE_SHIFT		16
#define HDMI_TX_VTC_GVSIZE_FRAME_VSIZE		GENMASK(13, 0)

#define HDMI_TX_VTC_GHSYNC			0x078
#define HDMI_TX_VTC_GH1BPSTART_SHIFT		16
#define HDMI_TX_VTC_GHSYNC_END_MASK		GENMASK(29, 16)
#define HDMI_TX_VTC_GHSYNC_START_MASK		GENMASK(13, 0)

#define HDMI_TX_VTC_GVBHOFF			0x07c
#define HDMI_TX_VTC_F0VSYNC_HEND_SHIFT		16
#define HDMI_TX_VTC_F0VBLANK_HEND_MASK		GENMASK(29, 16)
#define HDMI_TX_VTC_F0VBLANK_HSTART_MASK	GENMASK(13, 0)

#define HDMI_TX_VTC_GVSYNC			0x080
#define HDMI_TX_VTC_F0_VSYNC_VEND_MASK		GENMASK(29, 16)
#define HDMI_TX_VTC_F0_VSYNC_VSTART_MASK	GENMASK(13, 0)

#define HDMI_TX_VTC_GVSHOFF			0x084
#define HDMI_TX_VTC_GVBHOFF_F1			0x088
#define HDMI_TX_VTC_GVSYNC_F1			0x08c
#define HDMI_TX_VTC_GVSHOFF_F1			0x090
#define HDMI_TX_VTC_GASIZE_F1			0x094

#define HDMI_TX_VTC_BASE			0x10000
#define HDMI_MAX_LANES				4

#define HDMI_TX_3_4_GBPS			340000000
#define HDMI_TX_SCRAMBLER_OFFSET		0x20
#define HDMI_TX_TIMEGRID_VAL			0x18696
#define HDMI_TX_TOGGLE_CONF_VAL			0x630032
#define HDMI_TX_CONNECT_CONF_VAL		0xA0064
#define HDMI_TX_DDC_SLAVEADDR			0x54
#define HDMI_TX_DDC_CLKDIV			100000
#define HDMI_TX_DDC_EDID_LENGTH			512
#define HDMI_TX_DDC_EDID_SINK_BW		187
#define HDMI_TX_DDC_EDID_BW_SHIFT		4
#define HDMI_TX_DDC_ADDR			0x50
#define HDMI_TX_DDC_READ_DIR			1
#define HDMI_TX_DDC_DATA_MSK			0xFF
#define HDMI_TX_DDC_CMD_MSK			0xFE
#define HDMI_TX_DDC_CFG_1_FFE_LVLS_MASK		0xF
#define HDMI_TX_DDC_CFG_1_FFE_LVLS_SHIFT	4
#define HDMI_TX_DDC_CFG_1_FRL_RATE_MASK		0xF
#define HDMI_TX_DDC_SINK_VER_REG		0x01
#define HDMI_TX_DDC_UPDATE_FLGS_REG		0x10
#define HDMI_TX_DDC_CED_REG			0x50
#define HDMI_TX_DDC_STCR_REG			0x35
#define HDMI_TX_DDC_STAT_FLGS_REG		0x40
#define HDMI_TX_DDC_STAT_FLGS_LN01_REG		0x41
#define HDMI_TX_DDC_STAT_FLGS_LN23_REG		0x42
#define HDMI_TX_DDC_UPDATE_FLGS_CED_UPDATE_MASK	0x02
#define HDMI_TX_DDC_UPDATE_FLGS_STUPDATE_MASK	0x08
#define HDMI_TX_DDC_UPDATE_FLGS_FRL_START_MASK	0x10
#define HDMI_TX_DDC_UPDATE_FLGS_FLT_UPDATE_MASK	0x20
#define HDMI_TX_DDC_STCR_FLT_NO_TIMEOUT_MASK	0x20
#define HDMI_TX_DDC_STAT_FLGS_FLT_RDY_MASK	0x40
#define HDMI_TX_DDC_STAT_FLGS_LN01_LN0_MASK	0x0F
#define HDMI_TX_DDC_STAT_FLGS_LN01_LN1_SHIFT	4
#define HDMI_TX_DDC_STAT_FLGS_LN23_LN2_MASK	0x0F
#define HDMI_TX_DDC_STAT_FLGS_LN23_LN3_MASK	0x0F
#define HDMI_TX_DDC_STAT_FLGS_LN23_LN3_SHIFT	4

#define HDMI_TX_FRL_CLK_CYCLES			0x3E7
#define HDMI_TX_HDCP2x_ENABLE			0x404
#define HDMI_TX_HDCP2x_ENABLE_BYPASS_DISABLE_MASK	BIT(0)
#define HDMI_TX_PIXEL_MAXRATE			2376000
#define HDMI_TX_PIXELRATE_GBPS			((u64)1e9)

#define HDMI_TX_DDC_CMD_STR_TOKEN		0x100
#define HDMI_TX_DDC_CMD_STP_TOKEN		0x101
#define HDMI_TX_DDC_CMD_RD_TOKEN		0x102
#define HDMI_TX_DDC_CMD_WR_TOKEN		0x103

#define hdmi_mutex_lock(x)	mutex_lock(x)
#define hdmi_mutex_unlock(x)	mutex_unlock(x)

#define TIMEOUT_2MS		2
#define TIMEOUT_5MS		5
#define TIMEOUT_100MS		100
#define TIMEOUT_200MS		200
#define TIMEOUT_250MS		250
#define TIMEOUT_10US		10
/* TODO: Fix this delay in the future */
#define HDMI_TX_LNK_VID_RDY_DELAY	10000

#define HDMI_TX_MAX_FRL_RATE	6
#define HDMI_TX_SCDC_MASK	0xFF
#define HDMI_TX_DEF_TMDS_CLK	148500000

#define HDMI_HDCP_DPCD_READ	0x00
#define HDMI_HDCP_DPCD_WRITE	BIT(0)
#define HDMI_HDCP_STATUS	BIT(1)
#define HDMI_HDCP_TIMER_OFFSET	0x30000
#define HDMI_HDCP2X_OFFSET	0x40000
#define HDMI_HDCP1X_OFFSET	0x20000
#define HDMI_HDCP_MAX_KEYS	800

#define HDMI_MIN_WIDTH	640
#define HDMI_MIN_HEIGHT	480
#define HDMI_MAX_WIDTH	10240
#define HDMI_MAX_HEIGHT	4320

#define XHDMI_AUX_PKT_HEADER_SIZE	4
#define XHDMI_AUX_PKT_DATA_SIZE		32
#define XHDMI_AUX_PKT_SIZE		(XHDMI_AUX_PKT_HEADER_SIZE + \
					 XHDMI_AUX_PKT_DATA_SIZE)

/**
 * enum hdmi_state - Stream state
 * @HDMI_TX_STATE_STREAM_DOWN: stream down
 * @HDMI_TX_STATE_STREAM_UP: stream up
 */
enum hdmi_state {
	HDMI_TX_STATE_STREAM_DOWN = 0,
	HDMI_TX_STATE_STREAM_UP = 1
};

enum color_formats {
	HDMI_TX_CSF_RGB = 0,
	HDMI_TX_CSF_YCRCB_444 = 1,
	HDMI_TX_CSF_YCRCB_422 = 2,
	HDMI_TX_CSF_YCRCB_420 = 3
};

enum color_depths {
	HDMI_TX_BPC_8 = 8,
	HDMI_TX_BPC_10 = 10,
	HDMI_TX_BPC_12 = 12,
	HDMI_TX_BPC_16 = 16
};

enum config_ppc {
	HDMI_TX_PPC_1 = 1,
	HDMI_TX_PPC_2 = 2,
	HDMI_TX_PPC_4 = 4,
	HDMI_TX_PPC_8 = 8
};

enum vid_interface {
	HDMI_TX_AXI_STREAM = 0,
	HDMI_TX_NATIVE = 1,
	HDMI_TX_NATIVE_IDE = 2
};

/* FRL Training States */
enum frl_train_state {
	HDMI_TX_FRLSTATE_LTS_L = 0,
	HDMI_TX_FRLSTATE_LTS_1 = 1,
	HDMI_TX_FRLSTATE_LTS_2 = 2,
	HDMI_TX_FRLSTATE_LTS_3_ARM = 3,
	HDMI_TX_FRLSTATE_LTS_3 = 4,
	HDMI_TX_FRLSTATE_LTS_4 = 5,
	HDMI_TX_FRLSTATE_LTS_P_ARM = 6,
	HDMI_TX_FRLSTATE_LTS_P = 7,
	HDMI_TX_FRLSTATE_LTS_P_FRL_RDY = 8
};

/* LTP type */
enum frl_ltp_type {
	HDMI_TX_LTP_NO_LTP = 0,
	HDMI_TX_LTP_ALL_ONES = 1,
	HDMI_TX_LTP_ALL_ZEROES = 2,
	HDMI_TX_LTP_NYQUIST_CLOCK = 3,
	HDMI_TX_LTP_TXDDE_COMPLIANCE = 4,
	HDMI_TX_LTP_LFSR0 = 5,
	HDMI_TX_LTP_LFSR1 = 6,
	HDMI_TX_LTP_LFSR2 = 7,
	HDMI_TX_LTP_LFSR3 = 8
};

enum frl_active_mode {
	HDMI_TX_FRL_ACTIVE_MODE_GAP_ONLY = 0,
	HDMI_TX_FRL_ACTIVE_MODE_FULL_STREAM = 1
};

/* HDMI TX SCDC Fields */
enum xlnx_hdmi_scdc_fields {
	HDMI_TX_SCDC_FIELD_SOURCE_VER = 0,
	HDMI_TX_SCDC_FIELD_SNK_CFG0 = 1,
	HDMI_TX_SCDC_FIELD_SNK_CFG1 = 2,
	HDMI_TX_SCDC_FIELD_SNK_STU = 3,
	HDMI_TX_SCDC_FIELD_CED_UPDATE = 4,
	HDMI_TX_SCDC_FIELD_FRL_START = 5,
	HDMI_TX_SCDC_FIELD_FLT_UPDATE = 6,
	HDMI_TX_SCDC_FIELD_FLT_NO_RETRAIN = 7,
	HDMI_TX_SCDC_FIELD_SIZE = 8
};

struct xlnx_hdmi_scdc_field {
	u8 offset;
	u8 msk;
	u8 shift;
};

static const struct
xlnx_hdmi_scdc_field scdc_field[HDMI_TX_SCDC_FIELD_SIZE] = {
	{0x02, 0xFF, 0},/* HDMI_TX_SCDCFIELD_SOURCE_VER */
	{0x30, 0xFF, 0},/* HDMI_TX_SCDCFIELD_SNK_CFG0 */
	{0x31, 0xFF, 0},/* HDMI_TX_SCDCFIELD_SNK_CFG1 */
	{0x10, 0x01, 3},/* HDMI_TX_SCDCFIELD_SNK_STU */
	{0x10, 0xFF, 1},/* HDMI_TX_SCDCFIELD_CED_UPDATE */
	{0x10, 0xFF, 4},/* HDMI_TX_SCDCFIELD_FRL_START */
	{0x10, 0xFF, 5},/* HDMI_TX_SCDCFIELD_FLT_UPDATE */
	{0x30, 0x01, 1}	/* HDMI_TX_SCDCFIELD_FLT_NO_RETRAIN */
};

struct xlnx_hdmi_frlrate {
	u8 lanes;
	u8 linerate;
};

static const struct
xlnx_hdmi_frlrate rate_table[] = {
	{3, 0},	/* XHDMIC_MAXFRLRATE_NOT_SUPPORTED */
	{3, 3},	/* XHDMIC_MAXFRLRATE_3X3GBITSPS */
	{3, 6},	/* XHDMIC_MAXFRLRATE_3X6GBITSPS */
	{4, 6},	/* XHDMIC_MAXFRLRATE_4X6GBITSPS */
	{4, 8},	/* XHDMIC_MAXFRLRATE_4X8GBITSPS */
	{4, 10},/* XHDMIC_MAXFRLRATE_4X10GBITSPS */
	{4, 12},/* XHDMIC_MAXFRLRATE_4X12GBITSPS */
};

/**
 * struct xlnx_hdmi_frl_config - FRL config structure
 * @max_frl_rate: maximum supported FRL rate
 * @frl_rate: current FRL rate
 * @max_linerate: maximum supported linerate
 * @linerate: current linerate
 * @max_lanes: maximum supported lanes
 * @lanes: current lanes
 * @timer_cnt: frl timer
 * @timer_event: flag for timer event
 * @flt_no_timeout: flag for no timeout
 * @frl_train_states: indicates the frl training state
 */
struct xlnx_hdmi_frl_config {
	u8 max_frl_rate;
	u8 frl_rate;
	u8 max_linerate;
	u8 linerate;
	u8 max_lanes;
	u8 lanes;
	u16 timer_cnt;
	u8 timer_event;
	u8 flt_no_timeout;
	enum frl_train_state frl_train_states;
};

/**
 * struct xlnx_hdmi_config - Configuration of HDMI
 * @bpc: Bits per component
 * @ppc: Pixels per component
 * @vid_interface: AXI_stream or Native interface
 * @max_frl_rate: maximum frl rate supported by hardware
 * @htiming_div_fact: factor used in calculating htimings
 * @hdcp2x_enable: flag to indicate hdcp22-enable property in device tree
 * @hdcp1x_enable: flag to indicate hdcp-enable property in device tree
 */
struct xlnx_hdmi_config {
	enum color_depths bpc;
	enum config_ppc ppc;
	enum vid_interface vid_interface;
	u8 max_frl_rate;
	u8 htiming_div_fact;
	bool hdcp2x_enable;
	bool hdcp1x_enable;
};

/**
 * struct xlnx_hdmi_stream - Stream status
 * @frl_config: frl config structure
 * @is_frl: flag indicates frl or tmds
 * @tmds_clock_ratio: tmds clock ratio
 * @is_hdmi: flag indicates dvi or hdmi
 * @is_scrambled: scrambled enabled status;
 * @sink_max_linerate: maximum linerate supported by sink in Gbps
 * @sink_max_lanes: maximum lanes supported by sink
 * @state: enum reflects the stream is up or down
 */
struct xlnx_hdmi_stream {
	struct xlnx_hdmi_frl_config frl_config;
	u8 is_frl;
	u8 tmds_clock_ratio;
	u8 is_hdmi;
	u8 is_scrambled;
	u8 sink_max_linerate;
	u8 sink_max_lanes;
	enum hdmi_state state;
};

/**
 * struct xlnx_hdmi - Xilinx HDMI core
 * @dev: device structure
 * @encoder: the drm encoder structure
 * @connector: the drm connector structure
 * @base: device I/O memory for register access
 * @irq: hdmi subsystem irq
 * @phy: PHY handle for hdmi lanes
 * @hdcp1x_keymgmt_base: HDCP Key management address
 * @hdmi_mutex: mutex to lock hdmi structure
 * @irq_lock: to lock irq handler
 * @cable_connected: flag to indicate cable state
 * @hdmi_stream_up: flag to inidcate video stream state
 * @is_hdmi_20_sink: flag to indicate if sink is hdmi2.0 capable
 * @dpms: current dpms state
 * @xvidc_colorfmt: hdmi ip internal colorformat representation
 * @xvidc_colordepth: color depth
 * @config: IP configuration structure
 * @stream: stream properties
 * @intr_status: Flag to indicate irq status
 * @frl_status: Flag to indicate FRL interrupt status
 * @wait_for_streamup: Flag for stream up
 * @tmds_clk: TMDS clock
 * @wait_event: Wait event
 * @bridge: bridge structure
 * @height_out: configurable bridge output height parameter
 * @saved_adjusted_mode: Copy of @drm_crtc_state.adjusted_mode
 * @height_out_prop_val: configurable bridge output height parameter value
 * @width_out: configurable bridge output width parameter
 * @width_out_prop_val: configurable bridge output width parameter value
 * @in_fmt: configurable bridge input media format
 * @in_fmt_prop_val: configurable media bus format value
 * @out_fmt: configurable bridge output media format
 * @out_fmt_prop_val: configurable media bus format value
 * @txhdcp: Hdcp configuration
 * @hdcp_cp_irq_work: hdcp cp irq interrupt detection worker
 * @hdcp2x_timer_irq: hdcp2x timer interrupt
 * @hdcp1x_timer_irq: HDCP1X timer interrupt
 * @hdcp_irq: HDCP1.4 protocol interrupt
 * @aux_buffer: Aux packet buffer
 * @iframe: AVI infroframe structure
 */
struct xlnx_hdmi {
	struct device *dev;
	struct drm_encoder encoder;
	struct drm_connector connector;
	void __iomem *base;
	int irq;
	struct phy *phy[HDMI_MAX_LANES];
	struct regmap *hdcp1x_keymgmt_base;
	/* to lock hdmi */
	struct mutex hdmi_mutex;
	/* to lock irq */
	spinlock_t irq_lock;
	bool cable_connected;
	bool hdmi_stream_up;
	bool is_hdmi_20_sink;
	int dpms;
	enum color_formats xvidc_colorfmt;
	enum color_depths xvidc_colordepth;
	struct xlnx_hdmi_config config;
	struct xlnx_hdmi_stream stream;
	u32 intr_status;
	u32 frl_status;
	u32 wait_for_streamup:1;
	u64 tmds_clk;
	wait_queue_head_t wait_event;
	struct xlnx_bridge *bridge;
	struct drm_property *height_out;
	u32 height_out_prop_val;
	struct drm_property *width_out;
	struct drm_display_mode saved_adjusted_mode;
	u32 width_out_prop_val;
	struct drm_property *in_fmt;
	u32 in_fmt_prop_val;
	struct drm_property *out_fmt;
	u32 out_fmt_prop_val;
	struct delayed_work hdcp_cp_irq_work;
	struct xlnx_hdcptx txhdcp;
	int hdcp2x_timer_irq;
	int hdcp1x_timer_irq;
	int hdcp_irq;
	u32 aux_buffer[XHDMI_AUX_PKT_SIZE / sizeof(u32)];
	struct hdmi_avi_infoframe iframe;
};

enum xlnx_hdmitx_clks {
	S_AXI_CPU_ACLK = 0,
	LINK_CLK = 1,
	VIDEO_CLK = 2,
	FRL_CLK = 3,
	S_AXIS_VIDEO_ACLK = 4,
};

static struct clk_bulk_data hdmitx_clks[] = {
	{ .id = "s_axi_cpu_aclk" },
	{ .id = "link_clk" },
	{ .id = "video_clk" },
	{ .id = "frl_clk" },
	{ .id = "s_axis_video_aclk" },
};

/* Parallel Interface */
#define xlnx_hdmi_piointr_disable(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_PIO_CTRL_CLR,\
			 HDMI_TX_PIO_CTRL_IE)

#define xlnx_hdmi_piointr_clear(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_PIO_STA,\
			 HDMI_TX_PIO_STA_IRQ)

#define xlnx_hdmi_piointr_ie_enable(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_PIO_CTRL_SET,\
			 HDMI_TX_PIO_CTRL_IE)

#define xlnx_hdmi_piointr_run_enable(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_PIO_CTRL_SET,\
			 HDMI_TX_PIO_CTRL_RUN)

#define xlnx_hdmi_pio_set_sr(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_PIO_OUT_MSK,\
			 HDMI_TX_PIO_OUT_SR)

#define xlnx_hdmi_pio_set_pr(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_PIO_OUT_MSK,\
			 HDMI_TX_PIO_OUT_PR)

#define xlnx_hdmi_pio_set_cs(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_PIO_OUT_MSK,\
			 HDMI_TX_PIO_OUT_CS)

#define xlnx_hdmi_pio_set_cd(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_PIO_OUT_MSK,\
			 HDMI_TX_PIO_OUT_CD)

#define xlnx_pioout_bridge_yuv_clr(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_PIO_OUT_CLR,\
			 HDMI_TX_PIO_OUT_BRIDGE_YUV420)

#define xlnx_pioout_bridge_pixel_clr(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_PIO_OUT_CLR,\
			 HDMI_TX_PIO_OUT_BRIDGE_PIXEL)

#define xlnx_pioout_bridge_pixel_set(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_PIO_OUT_SET,\
			 HDMI_TX_PIO_OUT_BRIDGE_PIXEL)

#define xlnx_pioout_bridge_yuv_set(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_PIO_OUT_SET,\
			 HDMI_TX_PIO_OUT_BRIDGE_YUV420)

#define xlnx_hdmi_auxintr_enable(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_AUX_CTRL_SET,\
			 HDMI_TX_AUD_CTRL_IE)

/* Data Display Channel */
#define xlnx_hdmi_ddc_disable(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_DDC_CTRL_CLR,\
			 HDMI_TX_DDC_CTRL_RUN)

#define xlnx_hdmi_ddc_intr_clear(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_DDC_STA,\
			 HDMI_TX_DDC_STA_IRQ)

#define xlnx_hdmi_ddc_set_done(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_DDC_STA,\
			 HDMI_TX_DDC_STA_DONE)

#define xlnx_hdmi_ddc_set_timeout(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_DDC_STA,\
			 HDMI_TX_DDC_STA_TO)

#define xlnx_hdmi_ddc_intr_disable(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_DDC_CTRL_CLR,\
			 HDMI_TX_DDC_CTRL_IE)

#define xlnx_hdmi_ddc_intr_enable(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_DDC_CTRL_SET,\
			 HDMI_TX_DDC_CTRL_IE)

#define xlnx_hdmi_ddc_stop_cmd(hdmi) \
	xlnx_hdmi_ddcwrite_cmd(hdmi, HDMI_TX_DDC_CMD_STP_TOKEN)

#define xlnx_hdmi_ddc_rdtoken_cmd(hdmi) \
	xlnx_hdmi_ddcwrite_cmd(hdmi, HDMI_TX_DDC_CMD_RD_TOKEN)

#define xlnx_hdmi_ddc_rd_data(hdmi) \
	xlnx_hdmi_readl(hdmi, HDMI_TX_DDC_DAT)

#define xlnx_hdmi_ddc_run_enable(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_DDC_CTRL_SET,\
			 HDMI_TX_DDC_CTRL_RUN)

/* Audio */
#define xlnx_hdmi_audio_disable(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_AUD_CTRL_CLR,\
			 HDMI_TX_AUD_CTRL_RUN)
/* Aux communication */
#define xlnx_hdmi_aux_disable(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_AUX_CTRL_CLR,\
			 HDMI_TX_AUX_CTRL_RUN)

#define xlnx_hdmi_aux_enable(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_AUX_CTRL_SET,\
			 HDMI_TX_AUX_CTRL_RUN)

#define xlnx_hdmi_auxintr_enable(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_AUX_CTRL_SET,\
			 HDMI_TX_AUD_CTRL_IE)

#define xlnx_hdmi_auxintr_disable(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_AUX_CTRL_CLR,\
			 HDMI_TX_AUD_CTRL_IE)

/* Fixed Rate Link */
#define xlnx_hdmi_frl_intr_disable(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_FRL_CTRL_CLR,\
			 HDMI_TX_FRL_CTRL_IE)

#define xlnx_hdmi_frl_intr_enable(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_FRL_CTRL_SET,\
			 HDMI_TX_FRL_CTRL_IE)

#define xlnx_hdmi_frl_clear(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_FRL_CTRL_CLR,\
			 HDMI_TX_FRL_CTRL_RST)

#define xlnx_hdmi_frl_ext_vidsrc(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_FRL_CTRL_SET,\
			 HDMI_TX_FRL_CTRL_FRL_VCKE_EXT)

#define xlnx_hdmi_frl_reset(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_FRL_CTRL_SET,\
			 HDMI_TX_FRL_CTRL_RST)

#define xlnx_hdmi_frl_reset_assert(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_FRL_CTRL_CLR,\
			 HDMI_TX_FRL_CTRL_RST)

#define xlnx_hdmi_frl_reset_deassert(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_FRL_CTRL_SET,\
			 HDMI_TX_FRL_CTRL_RST)

#define xlnx_hdmi_frl_sleep(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_FRL_CTRL,\
			 HDMI_TX_FRL_CTRL_RST |\
			 HDMI_TX_FRL_CTRL_IE |\
			 HDMI_TX_FRL_CTRL_EXEC)

#define xlnx_hdmi_frl_mode_enable(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_FRL_CTRL_SET,\
			 HDMI_TX_FRL_CTRL_OP_MODE)

#define xlnx_hdmi_frl_mode_disable(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_FRL_CTRL_CLR,\
			 HDMI_TX_FRL_CTRL_OP_MODE)

#define xlnx_hdmi_frl_execute(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_FRL_CTRL_SET,\
			 HDMI_TX_FRL_CTRL_EXEC)

#define xlnx_hdmi_set_hdmi_mode(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_PIO_OUT_SET,\
			 HDMI_TX_PIO_OUT_MODE)

/* assert VID_IN bridge resets */
#define xlnx_hdmi_ext_sysrst_assert(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_PIO_OUT_CLR,\
			 HDMI_TX_PIO_OUT_EXT_SYSRST)

#define xlnx_hdmi_ext_vrst_assert(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_PIO_OUT_CLR,\
			 HDMI_TX_PIO_OUT_EXT_VRST)

#define xlnx_hdmi_int_lrst_assert(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_PIO_OUT_CLR,\
			 HDMI_TX_PIO_OUT_INT_LRST)

#define xlnx_hdmi_int_vrst_assert(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_PIO_OUT_CLR,\
			 HDMI_TX_PIO_OUT_INT_VRST)

#define xlnx_hdmi_ext_sysrst_deassert(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_PIO_OUT_SET,\
			 HDMI_TX_PIO_OUT_EXT_SYSRST)

#define xlnx_hdmi_ext_vrst_deassert(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_PIO_OUT_SET,\
			 HDMI_TX_PIO_OUT_EXT_VRST)

#define xlnx_hdmi_int_lrst_deassert(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_PIO_OUT_SET,\
			 HDMI_TX_PIO_OUT_INT_LRST)

#define xlnx_hdmi_int_vrst_deassert(hdmi) \
	xlnx_hdmi_writel(hdmi, HDMI_TX_PIO_OUT_SET,\
			 HDMI_TX_PIO_OUT_INT_VRST)

/* video timing controller */
#define xlnx_hdmi_vtc_enable(hdmi) \
	xlnx_hdmi_vtc_writel(hdmi, HDMI_TX_VTC_CTL, HDMI_TX_VTC_CTL_GE)

#define xlnx_hdmi_vtc_disable(hdmi) \
	xlnx_hdmi_vtc_clr(hdmi, HDMI_TX_VTC_CTL, HDMI_TX_VTC_CTL_GE)

static ssize_t xlnx_hdcp_key_store(struct device *sysfs_dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int ret = -EINVAL;
	struct xlnx_hdmi *hdmi = (struct xlnx_hdmi *)dev_get_drvdata(sysfs_dev);
	struct xlnx_hdcptx *xhdcp = &hdmi->txhdcp;

	if (!(hdmi->config.hdcp2x_enable || hdmi->config.hdcp1x_enable))
		return ret;

	if (IS_ERR(xhdcp->xhdcp2x)) {
		dev_err(hdmi->dev, "No HDCP2X module is Registered\n");
		return PTR_ERR(xhdcp->xhdcp2x);
	}

	ret = xlnx_hdcp_tx_set_keys(xhdcp, (const u8 *)buf);
	if (ret) {
		dev_err(xhdcp->dev, "failed to send HDCP key from Sysfs to common layer");
		return ret;
	}

	if (hdmi->config.hdcp2x_enable || hdmi->config.hdcp1x_enable) {
		ret = xlnx_start_hdcp_engine(&hdmi->txhdcp,
					     HDMI_MAX_LANES);
		if (ret < 0) {
			dev_err(hdmi->dev, "Failed to Start HDCP engine\n");
			return ret;
		}
	}

	return count;
}

static DEVICE_ATTR(xlnx_hdcp_key, 0600/*S_IRUSR | S_IWUSR*/, NULL, xlnx_hdcp_key_store);

static struct attribute *xlnx_hdcp_key_attrs[] = {
	&dev_attr_xlnx_hdcp_key.attr,
	NULL,
};

static struct attribute_group xlnx_hdcp_key_attr_group = {
	.attrs = xlnx_hdcp_key_attrs,
};

static inline void
xlnx_hdmi_writel(struct xlnx_hdmi *hdmi, u32 offset, u32 val)
{
	writel(val, hdmi->base + offset);
}

static inline u32 xlnx_hdmi_readl(struct xlnx_hdmi *hdmi, int offset)
{
	return readl(hdmi->base + offset);
}

static void xlnx_hdmi_clr(struct xlnx_hdmi *hdmi, int offset, u32 clr)
{
	xlnx_hdmi_writel(hdmi, offset, xlnx_hdmi_readl(hdmi, offset) & ~clr);
}

static inline void
xlnx_hdmi_vtc_writel(struct xlnx_hdmi *hdmi, u32 offset, u32 val)
{
	writel(val, hdmi->base + HDMI_TX_VTC_BASE + offset);
}

static inline u32
xlnx_hdmi_vtc_readl(struct xlnx_hdmi *hdmi, u32 offset)
{
	return readl(hdmi->base + HDMI_TX_VTC_BASE + offset);
}

static inline void
xlnx_hdmi_vtc_clr(struct xlnx_hdmi *hdmi, u32 offset, u32 clr)
{
	xlnx_hdmi_vtc_writel(hdmi, offset,
			     xlnx_hdmi_vtc_readl(hdmi, offset) & ~clr);
}

static inline void
xlnx_set_frl_link_clk(struct xlnx_hdmi *hdmi, u32 val)
{
	xlnx_hdmi_writel(hdmi, HDMI_TX_FRL_LNK_CLK, val);
}

static inline void
xlnx_set_frl_vid_clk(struct xlnx_hdmi *hdmi, u32 val)
{
	xlnx_hdmi_writel(hdmi, HDMI_TX_FRL_VID_CLK, val);
}

static inline struct
xlnx_hdmi *encoder_to_hdmi(struct drm_encoder *encoder)
{
	return container_of(encoder, struct xlnx_hdmi, encoder);
}

static inline
struct xlnx_hdmi *connector_to_hdmi(struct drm_connector *connector)
{
	return container_of(connector, struct xlnx_hdmi, connector);
}

static bool xlnx_hdmi_is_lnk_vid_rdy(struct xlnx_hdmi *hdmi)
{
	u32 reg_val;

	reg_val = xlnx_hdmi_readl(hdmi, HDMI_TX_PIO_IN);
	reg_val = FIELD_GET(HDMI_TX_PIO_LNK_VID_RDY_MASK, reg_val);
	if (reg_val == HDMI_TX_PIO_LNK_VID_RDY_MASK)
		return true;

	return false;
}

static int xlnx_hdmi_phy_configure(struct xlnx_hdmi *hdmi,
				   union phy_configure_opts *opts)
{
	int ret = 0, i;

	for (i = 0; i < HDMI_MAX_LANES; i++) {
		ret = phy_configure(hdmi->phy[i], opts);
		if (ret) {
			dev_err(hdmi->dev, "phy_configure error %d\n", ret);
			return ret;
		}
	}

	return ret;
}

/**
 * xlnx_hdmi_vtc_set_timing - configure video timing parameters
 * @hdmi: hdmi tx instance
 * @mode: drm display mode
 *
 * Program timing parameters into video timing controller
 * registers.
 *
 * @return: None
 */
static void xlnx_hdmi_vtc_set_timing(struct xlnx_hdmi *hdmi,
				     struct drm_display_mode *mode)
{
	u32 reg;
	u32 htotal, hactive, hsync_start, hbackporch_start;
	u32 vtotal, vactive, vsync_start, vbackporch_start;
	u32 hsync_len, hfront_porch, hback_porch;
	u32 vsync_len, vfront_porch, vback_porch;

	/* vtc reset */
	xlnx_hdmi_vtc_writel(hdmi, HDMI_TX_VTC_CTL, HDMI_TX_VTC_RST);
	reg = xlnx_hdmi_vtc_readl(hdmi, HDMI_TX_VTC_CTL);
	xlnx_hdmi_vtc_writel(hdmi, HDMI_TX_VTC_CTL, reg | HDMI_TX_VTC_CTL_RU);

	hactive = mode->hdisplay / hdmi->config.htiming_div_fact;
	hfront_porch = (mode->hsync_start - mode->hdisplay) /
		hdmi->config.htiming_div_fact;
	hback_porch = (mode->htotal - mode->hsync_end) /
		hdmi->config.htiming_div_fact;
	hsync_len = (mode->hsync_end - mode->hsync_start) /
		hdmi->config.htiming_div_fact;
	if (hdmi->xvidc_colorfmt == HDMI_TX_CSF_YCRCB_420) {
		if (hactive & 0x1 || hfront_porch & 0x1 ||
		    hback_porch & 0x1 || hsync_len & 0x1)
			dev_dbg(hdmi->dev, "VTC does not support this timing\n");

		hactive = hactive / 2;
		hfront_porch = hfront_porch / 2;
		hback_porch = hback_porch / 2;
		hsync_len = hsync_len / 2;
	}
	htotal = hactive + hfront_porch + hsync_len + hback_porch;
	hsync_start = hactive + hfront_porch;
	hbackporch_start = hsync_start + hsync_len;

	vactive = mode->vdisplay;
	vfront_porch = mode->vsync_start - mode->vdisplay;
	vback_porch = mode->vtotal - mode->vsync_end;
	vsync_len = mode->vsync_end - mode->vsync_start;
	vtotal = vactive + vfront_porch + vsync_len + vback_porch;
	vsync_start = vactive + vfront_porch;
	vbackporch_start = vsync_start + vsync_len;

	xlnx_hdmi_vtc_writel(hdmi, HDMI_TX_VTC_CTL, reg & ~HDMI_TX_VTC_CTL_RU);

	reg = htotal & HDMI_TX_VTC_GHSIZE_FRAME_HSIZE;
	xlnx_hdmi_vtc_writel(hdmi, HDMI_TX_VTC_GHSIZE, reg);

	reg = vtotal & HDMI_TX_VTC_GVSIZE_FRAME_VSIZE;
	reg |= reg << HDMI_TX_VTC_FIELD1_VSIZE_SHIFT;
	xlnx_hdmi_vtc_writel(hdmi, HDMI_TX_VTC_GVSIZE, reg);

	reg = hactive & HDMI_TX_VTC_ACTIVE_SIZE_MASK;
	reg |= (vactive & HDMI_TX_VTC_ACTIVE_SIZE_MASK) <<
		HDMI_TX_VTC_FIELD1_VSIZE_SHIFT;
	xlnx_hdmi_vtc_writel(hdmi, HDMI_TX_VTC_GASIZE_F0, reg);

	reg = hsync_start & HDMI_TX_VTC_GHSYNC_START_MASK;
	reg |= (hbackporch_start << HDMI_TX_VTC_GH1BPSTART_SHIFT) &
		HDMI_TX_VTC_GHSYNC_END_MASK;
	xlnx_hdmi_vtc_writel(hdmi, HDMI_TX_VTC_GHSYNC, reg);

	reg = (vsync_start - 1) & HDMI_TX_VTC_F0_VSYNC_VSTART_MASK;
	reg |= ((vbackporch_start - 1) << HDMI_TX_VTC_FIELD1_VSIZE_SHIFT) &
		HDMI_TX_VTC_F0_VSYNC_VEND_MASK;
	xlnx_hdmi_vtc_writel(hdmi, HDMI_TX_VTC_GVSYNC, reg);
	xlnx_hdmi_clr(hdmi, HDMI_TX_VTC_BASE + HDMI_TX_VTC_GFENC,
		      HDMI_TX_VTC_GFENC_MASK);

	/* Calculate and uppdate Generator VBlank Hori field 0 */
	reg = hactive & HDMI_TX_VTC_F0VBLANK_HSTART_MASK;
	reg |= (hactive << HDMI_TX_VTC_F0VSYNC_HEND_SHIFT) &
		HDMI_TX_VTC_F0VBLANK_HEND_MASK;
	xlnx_hdmi_vtc_writel(hdmi, HDMI_TX_VTC_GVBHOFF, reg);

	/* Calculate and update Generator VSync Hori field 0 */
	reg = hsync_start & HDMI_TX_VTC_F0VBLANK_HSTART_MASK;
	reg |= (hsync_start << HDMI_TX_VTC_F0VSYNC_HEND_SHIFT) &
		HDMI_TX_VTC_F0VBLANK_HEND_MASK;
	xlnx_hdmi_vtc_writel(hdmi, HDMI_TX_VTC_GVSHOFF, reg);

	/* sets all polarities as active high */
	xlnx_hdmi_vtc_writel(hdmi, HDMI_TX_VTC_GPOL, HDMI_TX_VTC_GPOL_MASK);
	/* configure timing source */
	xlnx_hdmi_vtc_writel(hdmi, HDMI_TX_VTC_CTL, HDMI_TX_VTC_CTL_MASK |
			     HDMI_TX_VTC_CTL_RU);
}

/**
 * xlnx_hdmi_ddc_getack: get acknowledge
 * @hdmi: pointer to HDMI TX core instance
 *
 * Returns: ddc transaction acknowledgment
 */
static bool xlnx_hdmi_ddc_getack(struct xlnx_hdmi *hdmi)
{
	u32 status;

	status = xlnx_hdmi_readl(hdmi, HDMI_TX_DDC_STA);

	return (status & HDMI_TX_DDC_STA_ACK);
}

/**
 * xlnx_hdmi_ddcwaitfordone - wait for the ddc done flag to be set
 * @hdmi: pointer to HDMI TX core instance
 *
 * Returns: 0 on success, 1 on timeout error
 */
static bool xlnx_hdmi_ddcwaitfordone(struct xlnx_hdmi *hdmi)
{
	u32 data;
	bool status, val = false;

	do {
		data = xlnx_hdmi_readl(hdmi, HDMI_TX_DDC_CTRL);
		if (data & HDMI_TX_DDC_CTRL_RUN) {
			data = xlnx_hdmi_readl(hdmi, HDMI_TX_DDC_STA);
			if (data & HDMI_TX_DDC_STA_DONE) {
				xlnx_hdmi_ddc_set_done(hdmi);
				val = true;
				status = false;
			} else if (data & HDMI_TX_DDC_STA_TO) {
				xlnx_hdmi_ddc_set_timeout(hdmi);
				val = true;
				status = true;
			}
		} else {
			status = true;
			val = true;
		}
	} while (!val);

	return status;
}

/**
 * xlnx_hdmi_ddcwrite_cmd - writes data into FIFO
 * @hdmi: pointer to HDMI TX core instance
 * @cmd: command to be written
 *
 * Returns: 0 on success, 1 if fifo full error
 */
static u32 xlnx_hdmi_ddcwrite_cmd(struct xlnx_hdmi *hdmi, u32 cmd)
{
	int tries = 0;
	u32 status;
	bool val = false;

	do {
		status = xlnx_hdmi_readl(hdmi, HDMI_TX_DDC_CTRL);
		if (status & HDMI_TX_DDC_CTRL_RUN) {
			status = xlnx_hdmi_readl(hdmi, HDMI_TX_DDC_STA);
			status &= HDMI_TX_DDC_STA_CMD_FULL;
			if (!status) {
				xlnx_hdmi_writel(hdmi, HDMI_TX_DDC_CMD, cmd);
				status = 0;
				val = true;
			} else {
				usleep_range(100, 200);
				if (tries++ > 10) {
					xlnx_hdmi_ddc_disable(hdmi);
					status = 1;
					val = true;
				}
			}
		} else {
			status = 1;
			val = true;
		}
	} while (!val);

	return status;
}

/**
 * xlnx_hdmi_ddcwrite - ddc write
 * @hdmi: pointer to HDMI TX core instance
 * @slave: slave address
 * @length: length of the data to be written
 * @buffer: data buffer
 * @stop: stop flag
 *
 * Returns: 0 if write is successful, 1 on failure.
 */
static u32 xlnx_hdmi_ddcwrite(struct xlnx_hdmi *hdmi, u8 slave,
			      u16 length, u8 *buffer, u8 stop)
{
	u32 data, index, status;

	/* ddc enable */
	xlnx_hdmi_ddc_run_enable(hdmi);
	xlnx_hdmi_ddc_intr_disable(hdmi);

	status = xlnx_hdmi_ddcwrite_cmd(hdmi, HDMI_TX_DDC_CMD_STR_TOKEN);
	if (status)
		return status;

	status = xlnx_hdmi_ddcwrite_cmd(hdmi, HDMI_TX_DDC_CMD_WR_TOKEN);
	if (status)
		return status;

	status = xlnx_hdmi_ddcwrite_cmd(hdmi, 0);
	if (status)
		return status;

	status = xlnx_hdmi_ddcwrite_cmd(hdmi, 1);
	if (status)
		return status;

	data = slave << 1;
	data &= HDMI_TX_DDC_CMD_MSK;
	status = xlnx_hdmi_ddcwrite_cmd(hdmi, data);
	if (status)
		return status;

	/* Wait for done flag */
	if (xlnx_hdmi_ddcwaitfordone(hdmi))
		return 1;

	if (xlnx_hdmi_ddc_getack(hdmi)) {
		status = xlnx_hdmi_ddcwrite_cmd(hdmi, HDMI_TX_DDC_CMD_WR_TOKEN);
		if (status)
			return status;

		data = (length >> 8) & HDMI_TX_DDC_DATA_MSK;
		status = xlnx_hdmi_ddcwrite_cmd(hdmi, data);
		if (status)
			return status;

		data = length & HDMI_TX_DDC_DATA_MSK;
		status = xlnx_hdmi_ddcwrite_cmd(hdmi, data);
		if (status)
			return status;

		for (index = 0; index < length; index++) {
			status = xlnx_hdmi_ddcwrite_cmd(hdmi, *buffer++);
			if (status)
				return status;
		}
		if (!xlnx_hdmi_ddcwaitfordone(hdmi)) {
			if (xlnx_hdmi_ddc_getack(hdmi)) {
				if (stop) {
					status = xlnx_hdmi_ddc_stop_cmd(hdmi);
					if (status)
						return status;

					xlnx_hdmi_ddcwaitfordone(hdmi);
				}
				status = 0;
			}
		}
	}
	xlnx_hdmi_ddc_disable(hdmi);

	return status;
}

/**
 * xlnx_hdmi_ddcreaddata - read data
 * @hdmi: pointer to HDMI TX core instance
 *
 * Returns: Byte data from ddc
 */
static u8 xlnx_hdmi_ddcreaddata(struct xlnx_hdmi *hdmi)
{
	u32 status;
	int tries = 0;
	u8 data;
	bool val = false;

	do {
		data = xlnx_hdmi_readl(hdmi, HDMI_TX_DDC_CTRL);
		if (data & HDMI_TX_DDC_CTRL_RUN) {
			status = xlnx_hdmi_readl(hdmi, HDMI_TX_DDC_STA);
			status &= HDMI_TX_DDC_STA_DAT_EMPTY;
			if (!status) {
				data = xlnx_hdmi_ddc_rd_data(hdmi);
				val = true;
			} else {
				usleep_range(1000, 1100);
				if (tries++ > 10) {
					xlnx_hdmi_ddc_disable(hdmi);
					val = true;
					data = 0;
				}
			}
		} else {
			val = true;
			data = 0;
		}
	} while (!val);

	return data;
}

/**
 * xlnx_hdmi_ddcread - read bulk data from ddc
 * @hdmi: pointer to HDMI TX core instance
 * @slave: slave address
 * @length: length of data to be read
 * @buffer: destination buffer address
 * @stop: stop flag
 *
 * Returns: 0 on success, 1 on timeout errors
 */
static u32 xlnx_hdmi_ddcread(struct xlnx_hdmi *hdmi, u8 slave,
			     u16 length, u8 *buffer, u8 stop)
{
	u32 data, index, status;

	/* ddc enable */
	xlnx_hdmi_ddc_run_enable(hdmi);
	xlnx_hdmi_ddc_intr_disable(hdmi);

	status = xlnx_hdmi_ddcwrite_cmd(hdmi, HDMI_TX_DDC_CMD_STR_TOKEN);
	if (status)
		return status;

	status = xlnx_hdmi_ddcwrite_cmd(hdmi, HDMI_TX_DDC_CMD_WR_TOKEN);
	if (status)
		return status;

	status = xlnx_hdmi_ddcwrite_cmd(hdmi, 0);
	if (status)
		return status;

	status = xlnx_hdmi_ddcwrite_cmd(hdmi, 1);
	if (status)
		return status;

	data = slave << 1;
	/* set read bit */
	data |= HDMI_TX_DDC_READ_DIR;

	status = xlnx_hdmi_ddcwrite_cmd(hdmi, data);
	if (status)
		return status;

	/* Wait for done flag */
	if (!xlnx_hdmi_ddcwaitfordone(hdmi)) {
		if (xlnx_hdmi_ddc_getack(hdmi)) {
			status = xlnx_hdmi_ddc_rdtoken_cmd(hdmi);
			if (status)
				return status;

			data = (length >> 8) & HDMI_TX_DDC_DATA_MSK;
			status = xlnx_hdmi_ddcwrite_cmd(hdmi, data);
			if (status)
				return status;

			data = length & HDMI_TX_DDC_DATA_MSK;
			status = xlnx_hdmi_ddcwrite_cmd(hdmi, data);
			if (status)
				return status;

			/* read data */
			for (index = 0; index < length; index++)
				*buffer++ = xlnx_hdmi_ddcreaddata(hdmi);
			if (!xlnx_hdmi_ddcwaitfordone(hdmi)) {
				if (stop) {
					status = xlnx_hdmi_ddc_stop_cmd(hdmi);
					if (status)
						return(status);

					xlnx_hdmi_ddcwaitfordone(hdmi);
				}
				status = 0;
			}
		}
	}
	xlnx_hdmi_ddc_disable(hdmi);

	return status;
}

/**
 * xlnx_hdmi_ddc_readreg - read register from ddc
 * @hdmi: pointer to HDMI TX core instance
 * @slave: slave address
 * @length: length of the data to be read
 * @reg_addr: register address
 * @buffer: destination buffer address
 *
 * Returns: 0 on success, non-zero value if ddc transaction fails.
 */
static int
xlnx_hdmi_ddc_readreg(struct xlnx_hdmi *hdmi, u8 slave, u16 length,
		      u8 reg_addr, u8 *buffer)
{
	int status;

	/* Set the register to be read */
	status = xlnx_hdmi_ddcwrite(hdmi, slave, 1, (u8 *)&reg_addr, false);
	if (!status)
		status = xlnx_hdmi_ddcread(hdmi, slave, length,
					   (u8 *)buffer, true);

	return status;
}

/**
 * xlnx_hdmi_ddcwrite_field - writes specified SCDC field
 * @hdmi: HDMI TX core instance structure
 * @field: field from SCDC channel to be written
 * @val: value to be written
 *
 * Returns: 0 on success, non-zero value if ddc transaction fails.
 */
static int xlnx_hdmi_ddcwrite_field(struct xlnx_hdmi *hdmi,
				    enum xlnx_hdmi_scdc_fields field, u8 val)
{
	u32 status;
	u8 ddc_buf[2] = {0};
	u8 offset = scdc_field[field].offset;

	if (scdc_field[field].msk != HDMI_TX_SCDC_MASK) {
		status = xlnx_hdmi_ddcwrite(hdmi, HDMI_TX_DDC_SLAVEADDR, 1,
					    (u8 *)&offset, false);
		if (status)
			return status;

		status = xlnx_hdmi_ddcread(hdmi, HDMI_TX_DDC_SLAVEADDR, 1,
					   (u8 *)&ddc_buf, true);
		if (status)
			return status;

		ddc_buf[0] &= ~(scdc_field[field].msk <<
				scdc_field[field].shift);
	} else {
		ddc_buf[0] |= ((val & scdc_field[field].msk) <<
			       scdc_field[field].shift);
	}

	ddc_buf[1] = ddc_buf[0];
	ddc_buf[0] = offset;
	return xlnx_hdmi_ddcwrite(hdmi, HDMI_TX_DDC_SLAVEADDR, 2,
				  (u8 *)&ddc_buf, true);
}

/**
 * xlnx_hdmi_set_samplerate - set sample rate
 * @hdmi: pointer to HDMI TX core instance
 * @samplerate: sample rate value
 *
 * Returns: None
 */
static void
xlnx_hdmi_set_samplerate(struct xlnx_hdmi *hdmi, unsigned int samplerate)
{
	u32 regvalue;

	xlnx_hdmi_pio_set_sr(hdmi);

	switch (samplerate) {
	case 2:
		regvalue = 2;
		break;
	case 3:
		regvalue = 1;
		break;
	case 5:
		regvalue = 3;
		break;
	default:
		regvalue = 0;
		break;
	}

	/* set sample rate */
	xlnx_hdmi_writel(hdmi, HDMI_TX_PIO_OUT,
			 (regvalue << (HDMI_TX_PIO_OUT_SR_SHIFT)));
}

/**
 * xlnx_hdmi_set_ppc - set pixel per clock
 * @hdmi: pointer to HDMI TX core instance
 *
 * Returns: None
 */
static void xlnx_hdmi_set_ppc(struct xlnx_hdmi *hdmi)
{
	u32 regvalue;

	/* Mask PIO Out Mask register */
	xlnx_hdmi_pio_set_pr(hdmi);

	/* Check for pixel width */
	switch (hdmi->config.ppc) {
	case HDMI_TX_PPC_2:
		regvalue = 1;
		break;
	case HDMI_TX_PPC_4:
		regvalue = 2;
		break;
	case HDMI_TX_PPC_8:
		regvalue = 3;
		break;
	default:
		regvalue = 0;
		break;
	}
	/* Write pixel rate into PIO Out register */
	xlnx_hdmi_writel(hdmi, HDMI_TX_PIO_OUT,
			 (regvalue << HDMI_TX_PIO_OUT_PR_SHIFT));
}

/**
 * xlnx_hdmi_set_colorfmt - set color format
 * @hdmi: pointer to HDMI TX core instance
 *
 * Returns: None
 */
static void xlnx_hdmi_set_colorfmt(struct xlnx_hdmi *hdmi)
{
	u32 regvalue;

	/* Mask PIO Out Mask register */
	xlnx_hdmi_pio_set_cs(hdmi);

	/* Check for color format */
	switch (hdmi->xvidc_colorfmt) {
	case HDMI_TX_CSF_YCRCB_444:
		regvalue = 1;
		break;
	case HDMI_TX_CSF_YCRCB_422:
		regvalue = 2;
		break;
	case HDMI_TX_CSF_YCRCB_420:
		regvalue = 3;
		break;
	default:
		regvalue = 0;
		break;
	}
	/* Write color space into PIO Out register */
	xlnx_hdmi_writel(hdmi, HDMI_TX_PIO_OUT,
			 (regvalue << HDMI_TX_PIO_OUT_CS_SHIFT));
}

/**
 * xlnx_hdmi_set_colordepth - set color depth
 * @hdmi: pointer to HDMI TX core instance
 *
 * Returns: None
 */
static void xlnx_hdmi_set_colordepth(struct xlnx_hdmi *hdmi)
{
	u32 regvalue;

	/* Mask PIO Out Mask register */
	xlnx_hdmi_pio_set_cd(hdmi);

	switch (hdmi->config.bpc) {
	case HDMI_TX_BPC_10:
		regvalue = 1;
		break;
	case HDMI_TX_BPC_12:
		regvalue = 2;
		break;
	case HDMI_TX_BPC_16:
		regvalue = 3;
		break;
	default:
		regvalue = 0;
		break;
	}
	/* Write color depth into PIO Out register */
	xlnx_hdmi_writel(hdmi, HDMI_TX_PIO_OUT,
			 regvalue << HDMI_TX_PIO_OUT_CD_SHIFT);
}

/**
 * xlnx_hdmi_clkratio - set clock ratio
 * @hdmi: pointer to HDMI TX core instance
 *
 * Returns: 0 on success, error if ddc write fails.
 */
static u32
xlnx_hdmi_clkratio(struct xlnx_hdmi *hdmi)
{
	u32 status;
	u8 ddc_buf[2];

	ddc_buf[0] = HDMI_TX_SCRAMBLER_OFFSET;
	status = xlnx_hdmi_ddcwrite(hdmi, HDMI_TX_DDC_SLAVEADDR, 1,
				    (u8 *)&ddc_buf, false);
	if (status)
		return status;

	/* Read TMDS configuration */
	status = xlnx_hdmi_ddcread(hdmi, HDMI_TX_DDC_SLAVEADDR, 1,
				   (u8 *)&ddc_buf, true);
	ddc_buf[0] &= 0xfd;

	if (hdmi->stream.tmds_clock_ratio)
		ddc_buf[0] |= 0x02;
	ddc_buf[1] = ddc_buf[0];
	ddc_buf[0] = HDMI_TX_SCRAMBLER_OFFSET;

	status = xlnx_hdmi_ddcwrite(hdmi, HDMI_TX_DDC_SLAVEADDR, 2,
				    (u8 *)&ddc_buf, true);

	return status;
}

static void xlnx_hdmi_avi_infoframe_colorspace(struct hdmi_avi_infoframe *frame,
					       enum color_formats fmt)
{
	switch (fmt) {
	case HDMI_TX_CSF_RGB:
		frame->colorspace = HDMI_COLORSPACE_RGB;
		break;
	case HDMI_TX_CSF_YCRCB_420:
		frame->colorspace = HDMI_COLORSPACE_YUV420;
		break;
	case HDMI_TX_CSF_YCRCB_422:
		frame->colorspace = HDMI_COLORSPACE_YUV422;
		break;
	case HDMI_TX_CSF_YCRCB_444:
		frame->colorspace = HDMI_COLORSPACE_YUV444;
		break;
	default:
		break;
	}
}

static void xlnx_hdmi_aux_write(struct xlnx_hdmi *hdmi)
{
	int index;
	u32 readval;

	readval = xlnx_hdmi_readl(hdmi, HDMI_TX_AUX_STA);

	if ((readval & (HDMI_TX_AUX_STA_PKT_RDY | HDMI_TX_AUX_STA_FL))) {
		if (readval & HDMI_TX_AUX_STA_FL) {
			dev_dbg(hdmi->dev, "HDMI TX AUX FIFO full\n");
		} else {
			for (index = 0; index < (XHDMI_AUX_PKT_SIZE / sizeof(u32)); index++) {
				xlnx_hdmi_writel(hdmi, HDMI_TX_AUX_DAT,
						 hdmi->aux_buffer[index]);
			}
		}
	}
}

static ssize_t xlnx_hdmi_send_avi_infoframe(struct xlnx_hdmi *hdmi)
{
	struct hdmi_avi_infoframe *frame = &hdmi->iframe;
	u8 *ptr = (u8 *)hdmi->aux_buffer;
	u8 buffer[HDMI_INFOFRAME_SIZE(AVI)] = {0};
	int ret;
	ssize_t err;

	ret = drm_hdmi_avi_infoframe_from_display_mode(frame,
						       &hdmi->connector,
						       &hdmi->saved_adjusted_mode);
	if (ret < 0) {
		dev_err(hdmi->dev, "couldn't fill AVI infoframe\n");
		return ret;
	}

	xlnx_hdmi_avi_infoframe_colorspace(frame, hdmi->xvidc_colorfmt);
	err = hdmi_avi_infoframe_pack(frame, buffer, HDMI_INFOFRAME_SIZE(AVI));
	if (err < 0) {
		dev_err(hdmi->dev, "Failed to pack AVI infoframe: %zd\n", err);
		return err;
	}

	/*
	 * As per the Table 8-1 in HDMI 1.4b specification, packetization of
	 * AVI Infoframe is like: HB0 HB1 HB2 PB0 PB1 .....PB27.
	 */

	/* Setting AVI Infoframe packet header from HB0 to HB2 */
	ptr[0] = buffer[0];
	ptr[1] = buffer[1];
	ptr[2] = buffer[2];
	/* Checksum (this will be calculated by the HDMI TX IP) */
	ptr[3] = 0x0;

	/* Copying PB0 - PB27 from offset 4 in the buffer */
	memcpy((void *)(&ptr[4]), (void *)(&buffer[3]),
	       (HDMI_INFOFRAME_SIZE(AVI) - HDMI_INFOFRAME_HEADER_SIZE));

	xlnx_hdmi_aux_write(hdmi);

	return 0;
}

static void xlnx_hdmi_send_infoframes(struct xlnx_hdmi *hdmi)
{
	xlnx_hdmi_send_avi_infoframe(hdmi);
}

static void xlnx_hdmi_vsync_event_handler(struct xlnx_hdmi *hdmi)
{
	xlnx_hdmi_send_infoframes(hdmi);
}

/**
 * xlnx_hdmi_stream_start - set core parameters
 * @hdmi: pointer to HDMI TX core instance
 *
 * Returns: 0 on success, 1 if ddc transaction fails
 */
static u32 xlnx_hdmi_stream_start(struct xlnx_hdmi *hdmi)
{
	u8 ddc_buf[2];
	int status;

	xlnx_hdmi_set_ppc(hdmi);
	xlnx_hdmi_set_colorfmt(hdmi);
	xlnx_hdmi_set_colordepth(hdmi);

	/*
	 * Set the TMDS clock ratio bit if the data rate is higher
	 * than 3.4Gb/s
	 */
	if (hdmi->tmds_clk > HDMI_TX_3_4_GBPS) {
		hdmi->stream.is_scrambled = true;
		hdmi->stream.tmds_clock_ratio = true;
	} else {
		hdmi->stream.is_scrambled = false;
		hdmi->stream.tmds_clock_ratio = false;
	}

	/* set scrambler */
	xlnx_hdmi_writel(hdmi, HDMI_TX_PIO_OUT_CLR, HDMI_TX_PIO_OUT_SCRM);

	ddc_buf[0] = HDMI_TX_SCRAMBLER_OFFSET;
	status = xlnx_hdmi_ddcread(hdmi, HDMI_TX_DDC_SLAVEADDR, 1,
				   (u8 *)&ddc_buf, false);
	if (!status)
		return status;

	status = xlnx_hdmi_ddcwrite(hdmi, HDMI_TX_DDC_SLAVEADDR, 1,
				    (u8 *)&ddc_buf, true);
	if (status) {
		ddc_buf[1] = ddc_buf[0] & HDMI_TX_DDC_CMD_MSK;
		ddc_buf[1] |= hdmi->stream.is_scrambled;

		status = xlnx_hdmi_ddcwrite(hdmi, HDMI_TX_DDC_SLAVEADDR, 2,
					    (u8 *)&ddc_buf, true);
	}
	/* set clock ratio */
	xlnx_hdmi_clkratio(hdmi);
	return status;
}

/**
 * xlnx_hdmi_set_frl_active: sets active FRL mode.
 *
 * @hdmi: HDMI TX core instance
 * @mode: Mode specifies the active FRL mode.
 * 0 = FRL transmission only includes GAP characters
 * 1 = FRL transmission includes video, audio and control packets
 */
static void
xlnx_hdmi_set_frl_active(struct xlnx_hdmi *hdmi, enum frl_active_mode mode)
{
	if (mode)
		xlnx_hdmi_writel(hdmi, HDMI_TX_FRL_CTRL_SET,
				 HDMI_TX_FRL_CTRL_FRL_ACT);
	else
		xlnx_hdmi_writel(hdmi, HDMI_TX_FRL_CTRL_CLR,
				 HDMI_TX_FRL_CTRL_FRL_ACT);
}

/**
 * xlnx_hdmi_set_frl_ltp: sets the link training pattern for the selected lane.
 *
 * @hdmi: pointer to HDMI TX instance
 * @lane: lane number
 * @ltp_type: link training pattern type
 */
static void
xlnx_hdmi_set_frl_ltp(struct xlnx_hdmi *hdmi, u8 lane, u8 ltp_type)
{
	u32 value = ltp_type;
	u32 data;

	data = xlnx_hdmi_readl(hdmi, HDMI_TX_FRL_CTRL);

	switch (lane) {
	case 0:
		data = data & ~((u32)(HDMI_TX_FRL_CTRL_FRL_REQ_MASK <<
				      HDMI_TX_FRL_CTRL_FRL_LTP0_SHIFT));
		data = data | ((value & HDMI_TX_FRL_CTRL_FRL_REQ_MASK) <<
			       HDMI_TX_FRL_CTRL_FRL_LTP0_SHIFT);
		break;
	case 1:
		data = data & ~((u32)(HDMI_TX_FRL_CTRL_FRL_REQ_MASK <<
				      HDMI_TX_FRL_CTRL_FRL_LTP1_SHIFT));
		data = data | ((value & HDMI_TX_FRL_CTRL_FRL_REQ_MASK) <<
			       HDMI_TX_FRL_CTRL_FRL_LTP1_SHIFT);
		break;
	case 2:
		data = data & ~((u32)(HDMI_TX_FRL_CTRL_FRL_REQ_MASK <<
				      HDMI_TX_FRL_CTRL_FRL_LTP2_SHIFT));
		data = data | ((value & HDMI_TX_FRL_CTRL_FRL_REQ_MASK) <<
			       HDMI_TX_FRL_CTRL_FRL_LTP2_SHIFT);
		break;
	case 3:
		data = data & ~((u32)(HDMI_TX_FRL_CTRL_FRL_REQ_MASK <<
				      HDMI_TX_FRL_CTRL_FRL_LTP3_SHIFT));
		data = data | ((value & HDMI_TX_FRL_CTRL_FRL_REQ_MASK) <<
			       HDMI_TX_FRL_CTRL_FRL_LTP3_SHIFT);
		break;
	default:
		dev_dbg(hdmi->dev, "Wrong lane is selected!\n");
		break;
	}

	xlnx_hdmi_writel(hdmi, HDMI_TX_FRL_CTRL, data);
}

static void xlnx_hdmi_streamup_callback(struct xlnx_hdmi *hdmi)
{
	union phy_configure_opts phy_cfg = {0};
	int ret;

	if (hdmi->stream.is_frl)
		xlnx_hdmi_frl_mode_enable(hdmi);
	else
		xlnx_hdmi_frl_mode_disable(hdmi);

	phy_cfg.hdmi.get_samplerate = 1;
	ret = xlnx_hdmi_phy_configure(hdmi, &phy_cfg);
	if (ret) {
		dev_err(hdmi->dev, "phy_cfg: get_samplerate err %d\n", ret);
		return;
	}
	/* Set the sample rate got from HMDI-PHY */
	xlnx_hdmi_set_samplerate(hdmi,
				 phy_cfg.hdmi.samplerate);
	xlnx_hdmi_stream_start(hdmi);

	phy_cfg.hdmi.clkout1_obuftds = 1;
	phy_cfg.hdmi.clkout1_obuftds_en = true;
	ret = xlnx_hdmi_phy_configure(hdmi, &phy_cfg);
	if (ret) {
		dev_err(hdmi->dev, "phy_cfg: obuftds_en err %d\n", ret);
		return;
	}

	/* release vid_in bridge resets */
	xlnx_hdmi_ext_sysrst_deassert(hdmi);
	xlnx_hdmi_ext_vrst_deassert(hdmi);
	/* release tx core resets */
	xlnx_hdmi_int_lrst_deassert(hdmi);
	xlnx_hdmi_int_vrst_deassert(hdmi);

	if (hdmi->xvidc_colorfmt == HDMI_TX_CSF_YCRCB_420) {
		xlnx_pioout_bridge_yuv_set(hdmi);
		xlnx_pioout_bridge_pixel_clr(hdmi);
	} else {
		if (hdmi->iframe.pixel_repeat) {
			xlnx_pioout_bridge_yuv_clr(hdmi);
			xlnx_pioout_bridge_pixel_set(hdmi);
		} else {
			xlnx_pioout_bridge_yuv_clr(hdmi);
			xlnx_pioout_bridge_pixel_clr(hdmi);
		}
	}

	hdmi->wait_for_streamup =
		xlnx_hdmi_is_lnk_vid_rdy(hdmi) ? 1 : 0;
	wake_up(&hdmi->wait_event);
}

/**
 * xlnx_hdmi_set_frl_timer: sets the frl timer value
 * @hdmi: pinter to HDMI TX core instance
 * @timer_val: timer value in milliseconds
 */
static void xlnx_hdmi_set_frl_timer(struct xlnx_hdmi *hdmi, u32 timer_val)
{
	u32 clk_cycles = 0;
	unsigned long clkrate;

	clkrate = clk_get_rate(hdmitx_clks[S_AXI_CPU_ACLK].clk);
	if (timer_val == TIMEOUT_10US)
		clk_cycles = div_u64(clkrate, 100000);
	else if (timer_val > 0)
		clk_cycles = div_u64(clkrate * timer_val, 1000);

	xlnx_hdmi_writel(hdmi, HDMI_TX_FRL_TMR, clk_cycles);
}

/**
 * xlnx_hdmi_clear_frl_ltp - stops sending link training patterns
 * @hdmi: pointer to HDMI TX core instance
 */
static void xlnx_hdmi_clear_frl_ltp(struct xlnx_hdmi *hdmi)
{
	u32 index;

	for (index = 0; index < HDMI_MAX_LANES; index++)
		xlnx_hdmi_set_frl_ltp(hdmi, index, HDMI_TX_LTP_NO_LTP);
}

static int xlnx_hdmi_set_frl_rate(struct xlnx_hdmi *hdmi, u8 frlrate)
{
	if (!frlrate) {
		dev_err(hdmi->dev, "frl_rate %d not supported\n", frlrate);
		return 1;
	}
	hdmi->stream.frl_config.frl_rate = frlrate;
	hdmi->stream.frl_config.lanes = rate_table[frlrate].lanes;
	hdmi->stream.frl_config.linerate = rate_table[frlrate].linerate;

	dev_dbg(hdmi->dev, "Setting FRL rate @%d Gbps\n", rate_table[frlrate].linerate);
	/* Set lanes */
	if (hdmi->stream.frl_config.lanes == 4)
		xlnx_hdmi_writel(hdmi, HDMI_TX_FRL_CTRL_SET,
				 HDMI_TX_FRL_CTRL_FRL_LN_OP);
	else
		xlnx_hdmi_writel(hdmi, HDMI_TX_FRL_CTRL_CLR,
				 HDMI_TX_FRL_CTRL_FRL_LN_OP);
	/*TODO: FFE levels needs to set here */
	return xlnx_hdmi_ddcwrite_field(hdmi, HDMI_TX_SCDC_FIELD_SNK_CFG1,
					frlrate);
}

/**
 * xlnx_hdmi_reset - Reset the core and bridge
 * @hdmi: HDMI core structure
 *
 * Returns: None
 */
static void xlnx_hdmi_reset(struct xlnx_hdmi *hdmi)
{
	/* hdmi core reset - assert */
	xlnx_hdmi_int_lrst_assert(hdmi);
	xlnx_hdmi_int_vrst_assert(hdmi);

	/* vid out bridge reset */
	xlnx_hdmi_ext_sysrst_assert(hdmi);
	xlnx_hdmi_ext_vrst_assert(hdmi);

	/* release vid in bridge resets */
	xlnx_hdmi_ext_sysrst_deassert(hdmi);
	xlnx_hdmi_ext_vrst_deassert(hdmi);

	/* release hdmi tx core resets */
	xlnx_hdmi_int_lrst_deassert(hdmi);
	xlnx_hdmi_int_vrst_deassert(hdmi);
}

static void xlnx_hdmi_streamdown_callback(struct xlnx_hdmi *hdmi)
{
	xlnx_hdmi_reset(hdmi);
	xlnx_hdmi_ddc_disable(hdmi);
}

static void xlnx_hdmi_tmdsconfig(struct xlnx_hdmi *hdmi)
{
	union phy_configure_opts phy_cfg = {0};
	int ret;

	phy_cfg.hdmi.ibufds = 1;
	phy_cfg.hdmi.ibufds_en = true;
	ret = xlnx_hdmi_phy_configure(hdmi, &phy_cfg);
	if (ret) {
		dev_err(hdmi->dev, "phy_cfg: Ibufds err %d\n", ret);
		return;
	}

	phy_cfg.hdmi.clkout1_obuftds = 1;
	phy_cfg.hdmi.clkout1_obuftds_en = false;
	ret = xlnx_hdmi_phy_configure(hdmi, &phy_cfg);
	if (ret) {
		dev_err(hdmi->dev, "phy_cfg: obuftds_en err %d\n", ret);
		return;
	}

	phy_cfg.hdmi.config_hdmi20 = 1;
	ret = xlnx_hdmi_phy_configure(hdmi, &phy_cfg);
	if (ret) {
		dev_err(hdmi->dev, "phy_cfg: hdmi20 err %d\n", ret);
		return;
	}

	xlnx_set_frl_link_clk(hdmi, 0);
	xlnx_set_frl_vid_clk(hdmi, 0);
	xlnx_hdmi_set_hdmi_mode(hdmi);
}

/**
 * xlnx_hdmi_hdcp_reset - Reset hdcp module
 * @hdmi: HDMI IP core structure
 *
 * This function resets HDCP cipher engine,
 * protocol state machine and its internal parameters.
 *
 * Return: 0 on success, or the error code returned
 * from the callee functions.
 */
static int xlnx_hdmi_hdcp_reset(struct xlnx_hdmi *hdmi)
{
	struct xlnx_hdcptx *xhdcp = &hdmi->txhdcp;
	int ret;

	cancel_delayed_work(&hdmi->hdcp_cp_irq_work);
	ret = xlnx_hdcp_tx_reset(xhdcp);
	if (ret < 0) {
		dev_err(xhdcp->dev, "failed to reset HDCP");
		return ret;
	}

	return 0;
}

static void xlnx_hdmi_connect_callback(struct xlnx_hdmi *hdmi)
{
	union phy_configure_opts phy_cfg = {0};
	int ret;

	if (hdmi->cable_connected) {
		xlnx_hdmi_ddc_disable(hdmi);

		hdmi->tmds_clk = HDMI_TX_DEF_TMDS_CLK;
		xlnx_hdmi_stream_start(hdmi);
		phy_cfg.hdmi.tx_params = 1;
		phy_cfg.hdmi.ppc = hdmi->config.ppc;
		phy_cfg.hdmi.bpc = hdmi->config.bpc;
		phy_cfg.hdmi.fmt = hdmi->xvidc_colorfmt;
		phy_cfg.hdmi.tx_tmdsclk = hdmi->tmds_clk;
		ret = xlnx_hdmi_phy_configure(hdmi, &phy_cfg);
		if (ret) {
			dev_err(hdmi->dev, "phy_cfg: txparams error %d\n", ret);
			return;
		}

		xlnx_hdmi_tmdsconfig(hdmi);
	} else {
		struct xlnx_hdcptx *xhdcp = &hdmi->txhdcp;

		if (xhdcp->hdcp2xenable || xhdcp->hdcp1xenable) {
			ret = xlnx_hdmi_hdcp_reset(hdmi);
			if (ret < 0) {
				dev_err(hdmi->dev, "failed to reset HDCP %d\n", ret);
				return;
			}
		}
		phy_cfg.hdmi.ibufds = 1;
		phy_cfg.hdmi.ibufds_en = true;
		ret = xlnx_hdmi_phy_configure(hdmi, &phy_cfg);
		if (ret) {
			dev_err(hdmi->dev, "phy_cfg: Ibufds err %d\n", ret);
			return;
		}
	}
}

static void xlnx_hdmi_frl_config(struct xlnx_hdmi *hdmi)
{
	union phy_configure_opts phy_cfg = {0};
	int ret;

	/* Enable HDMI 2.1 config */
	phy_cfg.hdmi.linerate =
		(u64)(hdmi->stream.frl_config.linerate * HDMI_TX_PIXELRATE_GBPS);
	phy_cfg.hdmi.nchannels = hdmi->stream.frl_config.lanes;
	phy_cfg.hdmi.config_hdmi21 = 1;
	ret = xlnx_hdmi_phy_configure(hdmi, &phy_cfg);
	if (ret) {
		dev_err(hdmi->dev, "phy_cfg: hdmi21 config failed\n");
		return;
	}

	/* set FRL mode */
	hdmi->stream.is_frl = 1;
}

static int xlnx_hdmi_sink_max_frl(struct xlnx_hdmi *hdmi)
{
	struct drm_connector *connector = &hdmi->connector;
	int max_lanes, rate_per_lane, max_frl_rate;
	int sink_max_frl_bw;

	max_lanes = connector->display_info.hdmi.max_lanes;
	rate_per_lane = connector->display_info.hdmi.max_frl_rate_per_lane;
	max_frl_rate = max_lanes * rate_per_lane;

	switch (max_frl_rate) {
	case 9:
		sink_max_frl_bw = DP_PCON_ENABLE_MAX_BW_9GBPS;
		break;
	case 18:
		sink_max_frl_bw = DP_PCON_ENABLE_MAX_BW_18GBPS;
		break;
	case 24:
		sink_max_frl_bw = DP_PCON_ENABLE_MAX_BW_24GBPS;
		break;
	case 32:
		sink_max_frl_bw = DP_PCON_ENABLE_MAX_BW_32GBPS;
		break;
	case 40:
		sink_max_frl_bw = DP_PCON_ENABLE_MAX_BW_40GBPS;
		break;
	case 48:
		sink_max_frl_bw = DP_PCON_ENABLE_MAX_BW_48GBPS;
		break;
	case 0:
		sink_max_frl_bw = DP_PCON_ENABLE_MAX_BW_0GBPS;
		break;
	default:
		return -EINVAL;
	}

	return sink_max_frl_bw;
}

/**
 * xlnx_hdmi_frl_train_init: Initializes sink's SCDC for training.
 * @hdmi: Pointer to HDMI TX core instance
 *
 * Returns: 0 on success, non-zero value if ddc transaction fails.
 */
static int xlnx_hdmi_frl_train_init(struct xlnx_hdmi *hdmi)
{
	int status = 1;
	int source_max_frl_bw, sink_max_frl_bw, max_frl_bw;

	xlnx_hdmi_clear_frl_ltp(hdmi);
	/*
	 * Initialize the FRL module to send out GAP characters only for
	 * link training
	 */
	xlnx_hdmi_set_frl_active(hdmi, HDMI_TX_FRL_ACTIVE_MODE_GAP_ONLY);

	source_max_frl_bw = hdmi->config.max_frl_rate;
	sink_max_frl_bw = xlnx_hdmi_sink_max_frl(hdmi);
	max_frl_bw = min(sink_max_frl_bw, source_max_frl_bw);
	if (max_frl_bw <= 0)
		return 1;

	/* Initialize the core to operate in FRL mode */
	xlnx_hdmi_frl_mode_enable(hdmi);
	status = xlnx_hdmi_set_frl_rate(hdmi, max_frl_bw);
	if (status)
		return status;

	return xlnx_hdmi_ddcwrite_field(hdmi, HDMI_TX_SCDC_FIELD_SNK_CFG0, 0);
}

/**
 * xlnx_hdmi_exec_frl_state_ltsl - executes legacy training state
 * @hdmi: pointer to HDMI TX core instance
 *
 * Returns: 0 on success, non-zero value if ddc transaction fails.
 */
static int xlnx_hdmi_exec_frl_state_ltsl(struct xlnx_hdmi *hdmi)
{
	int status;
	u8 ddc_buf;

	xlnx_hdmi_set_frl_timer(hdmi, 0);
	xlnx_hdmi_frl_reset_assert(hdmi);
	xlnx_hdmi_frl_reset_deassert(hdmi);
	xlnx_hdmi_frl_mode_disable(hdmi);
	hdmi->stream.is_frl = 0;

	status = xlnx_hdmi_ddc_readreg(hdmi, HDMI_TX_DDC_SLAVEADDR, 1,
				       HDMI_TX_DDC_UPDATE_FLGS_REG,
				       (u8 *)&ddc_buf);
	if (status)
		return status;

	if (ddc_buf & HDMI_TX_DDC_UPDATE_FLGS_FLT_UPDATE_MASK)
		status = xlnx_hdmi_ddcwrite_field(hdmi,
						  HDMI_TX_SCDC_FIELD_FLT_UPDATE,
						  1);
	if (!status)
		xlnx_hdmi_frl_execute(hdmi);

	return status;
}

/**
 * xlnx_hdmi_exec_frl_state_lts1 - executes FRL LTS1 training state
 * @hdmi: pointer to HDMI TX core instance
 *
 * Returns: 0 on success, non-zero value if ddc transaction fails.
 */
static int xlnx_hdmi_exec_frl_state_lts1(struct xlnx_hdmi *hdmi)
{
	int status;
	u8 ddc_buf;

	/* Read sink version */
	status = xlnx_hdmi_ddc_readreg(hdmi, HDMI_TX_DDC_SLAVEADDR, 1,
				       HDMI_TX_DDC_SINK_VER_REG,
				       (u8 *)&ddc_buf);

	if (!status && ddc_buf != 0) {
		status = xlnx_hdmi_ddcwrite_field(hdmi,
						  HDMI_TX_SCDC_FIELD_SOURCE_VER,
						  1);

		if (!status) {
			hdmi->stream.frl_config.frl_train_states =
					HDMI_TX_FRLSTATE_LTS_2;
			hdmi->stream.frl_config.timer_cnt = 0;
		}
	} else {
		hdmi->stream.frl_config.frl_train_states =
			HDMI_TX_FRLSTATE_LTS_L;
		status = 1;
	}

	xlnx_hdmi_set_frl_timer(hdmi, TIMEOUT_10US);

	return status;
}

/**
 * xlnx_hdmi_exec_frl_state_lts2: executes FRL LTS2 training state
 * @hdmi: pointer to Hdmi Tx core instance
 *
 * Returns: 0 on success, non-zero value if ddc transaction or
 * phy configure call fails.
 */
static int xlnx_hdmi_exec_frl_state_lts2(struct xlnx_hdmi *hdmi)
{
	union phy_configure_opts phy_cfg = {0};
	int status = 1, ret, i;
	u8 ddc_buf, index;

	hdmi->stream.frl_config.timer_cnt += TIMEOUT_5MS;
	status = xlnx_hdmi_ddc_readreg(hdmi, HDMI_TX_DDC_SLAVEADDR, 1,
				       HDMI_TX_DDC_STCR_REG, (u8 *)&ddc_buf);

	/* Reset GTPLL before starting FRL Training */
	phy_cfg.hdmi.resetgtpll = 1;
	for (i = 0; i < HDMI_MAX_LANES; i++) {
		ret = phy_configure(hdmi->phy[i], &phy_cfg);
		if (ret) {
			dev_err(hdmi->dev, "phy_cfg: resetgtpll config failed\n");
			return ret;
		}
	}

	if (!status) {
		if (ddc_buf & HDMI_TX_DDC_STCR_FLT_NO_TIMEOUT_MASK)
			hdmi->stream.frl_config.flt_no_timeout = true;
		else
			hdmi->stream.frl_config.flt_no_timeout = false;

		xlnx_hdmi_ddcwrite_field(hdmi, HDMI_TX_SCDC_FIELD_SNK_STU, 1);
	}

	/* Read FLT_NO_UPDATE SCDC Register */
	if (!status && (hdmi->stream.frl_config.flt_no_timeout ||
			hdmi->stream.frl_config.timer_cnt < TIMEOUT_100MS)) {
		status = xlnx_hdmi_ddc_readreg(hdmi, HDMI_TX_DDC_SLAVEADDR, 1,
					       HDMI_TX_DDC_STAT_FLGS_REG,
					       (u8 *)&ddc_buf);
		if (status)
			return status;

		if (ddc_buf & HDMI_TX_DDC_STAT_FLGS_FLT_RDY_MASK) {
			/* Set the training state as LTS_3_ARM */
			xlnx_hdmi_set_frl_timer(hdmi, 0);
			hdmi->stream.frl_config.timer_cnt = 0;
			hdmi->stream.frl_config.frl_train_states =
					HDMI_TX_FRLSTATE_LTS_3_ARM;

			xlnx_hdmi_frl_config(hdmi);

			/* set Nyquist Clock as link training pattern */
			for (index = 0; index < HDMI_MAX_LANES; index++) {
				xlnx_hdmi_set_frl_ltp(hdmi, index,
						      HDMI_TX_LTP_NYQUIST_CLOCK);
			}

			xlnx_hdmi_frl_execute(hdmi);
		}
	} else {
		/* Timeout, fallback to LTS:L training state */
		hdmi->stream.frl_config.frl_train_states =
			HDMI_TX_FRLSTATE_LTS_L;
		xlnx_hdmi_set_frl_timer(hdmi, TIMEOUT_10US);
	}

	return status;
}

/**
 * xlnx_hdmi_exec_frl_state_lts2_ratewr: executes FRL LTS2-wr training state
 * @hdmi: pointer to Hdmi Tx core instance
 *
 * Returns: 0 on success, non zero value on failure
 */
static int xlnx_hdmi_exec_frl_state_lts2_ratewr(struct xlnx_hdmi *hdmi)
{
	int status;

	status = xlnx_hdmi_frl_train_init(hdmi);
	if (status) {
		dev_err(hdmi->dev, "lts2 train init failed\n");
		hdmi->stream.frl_config.frl_train_states =
			HDMI_TX_FRLSTATE_LTS_L;
		return status;
	}

	xlnx_hdmi_frl_execute(hdmi);
	hdmi->stream.frl_config.frl_train_states = HDMI_TX_FRLSTATE_LTS_3;
	xlnx_hdmi_set_frl_timer(hdmi, TIMEOUT_10US);

	return status;
}

/**
 * xlnx_hdmi_exec_frl_state_lts3: executes FRL LTS3 training state
 * @hdmi: pointer to HDMI TX core instance
 *
 * Returns: 0 on success, non-zero value if ddc transaction fails.
 */
static int xlnx_hdmi_exec_frl_state_lts3(struct xlnx_hdmi *hdmi)
{
	int status;
	u8 ddc_buf[4], ln;

	/* If timeout is 200ms, fallback to LTS:L */
	if (hdmi->stream.frl_config.timer_cnt > TIMEOUT_200MS &&
	    !hdmi->stream.frl_config.flt_no_timeout) {
		hdmi->stream.frl_config.timer_cnt = 0;
		hdmi->stream.frl_config.frl_train_states =
			HDMI_TX_FRLSTATE_LTS_L;
		xlnx_hdmi_set_frl_timer(hdmi, TIMEOUT_10US);
		return 1;
	}

	xlnx_hdmi_set_frl_timer(hdmi, TIMEOUT_2MS);
	hdmi->stream.frl_config.timer_cnt += TIMEOUT_2MS;

	status = xlnx_hdmi_ddc_readreg(hdmi, HDMI_TX_DDC_SLAVEADDR, 1,
				       HDMI_TX_DDC_UPDATE_FLGS_REG,
				       (u8 *)&ddc_buf);

	if (status || (ddc_buf[0] & HDMI_TX_DDC_UPDATE_FLGS_FLT_UPDATE_MASK) !=
	    HDMI_TX_DDC_UPDATE_FLGS_FLT_UPDATE_MASK)
		return 1;

	if (ddc_buf[0] & HDMI_TX_DDC_UPDATE_FLGS_STUPDATE_MASK) {
		status = xlnx_hdmi_ddc_readreg(hdmi, HDMI_TX_DDC_SLAVEADDR, 1,
					       HDMI_TX_DDC_STCR_REG,
					       (u8 *)&ddc_buf);
		if (status)
			return status;

		if (ddc_buf[0] & HDMI_TX_DDC_STCR_FLT_NO_TIMEOUT_MASK)
			hdmi->stream.frl_config.flt_no_timeout = true;
		else
			hdmi->stream.frl_config.flt_no_timeout = false;

		status = xlnx_hdmi_ddcwrite_field(hdmi,
						  HDMI_TX_SCDC_FIELD_SNK_STU,
						  1);
	}

	status = xlnx_hdmi_ddc_readreg(hdmi, HDMI_TX_DDC_SLAVEADDR, 2,
				       HDMI_TX_DDC_STAT_FLGS_LN01_REG,
				       (u8 *)&ddc_buf);
	if (status)
		return status;

	ddc_buf[3] = ddc_buf[1] >> HDMI_TX_DDC_STAT_FLGS_LN23_LN3_SHIFT;
	ddc_buf[2] = ddc_buf[1] & HDMI_TX_DDC_STAT_FLGS_LN23_LN2_MASK;
	ddc_buf[1] = ddc_buf[0] >> HDMI_TX_DDC_STAT_FLGS_LN01_LN1_SHIFT;
	ddc_buf[0] = ddc_buf[0] & HDMI_TX_DDC_STAT_FLGS_LN01_LN0_MASK;

	/* link training is successful, if ddc status flag value is 0x0 */
	if (ddc_buf[0] == 0x0 && ddc_buf[1] == 0x0 &&
	    ddc_buf[2] == 0x0 && ddc_buf[3] == 0x0) {
		hdmi->stream.frl_config.timer_cnt = 0;
		hdmi->stream.frl_config.frl_train_states =
			HDMI_TX_FRLSTATE_LTS_P_ARM;
		xlnx_hdmi_set_frl_timer(hdmi, TIMEOUT_10US);
		return 0;
	} else if (ddc_buf[0] == 0xF && ddc_buf[1] == 0xF &&
		   ddc_buf[2] == 0xF && ddc_buf[3] == 0xF) {
		/* 0xF means a request to drop FRL rate */
		hdmi->config.max_frl_rate = hdmi->config.max_frl_rate - 1;
		hdmi->stream.frl_config.timer_cnt = 0;
		hdmi->stream.frl_config.frl_train_states =
			HDMI_TX_FRLSTATE_LTS_4;
		xlnx_hdmi_set_frl_timer(hdmi, TIMEOUT_10US);
	} else {
		for (ln = 0; ln < 4; ln++) {
			/*
			 * 0x1 to 0x8 means specific link training pattern is
			 * requested. Each of the lane need to be set to output
			 * the link training pattern as requested.
			 */
			if (ddc_buf[ln] >= 1 && ddc_buf[ln] <= 8) {
				if (ddc_buf[ln] != 3 ||
				    hdmi->stream.frl_config.flt_no_timeout)
					xlnx_hdmi_set_frl_ltp(hdmi, ln,
							      ddc_buf[ln]);
			}
		}
		xlnx_hdmi_frl_execute(hdmi);
	}

	return xlnx_hdmi_ddcwrite_field(hdmi, HDMI_TX_SCDC_FIELD_FLT_UPDATE, 1);
}

/**
 * xlnx_hdmi_exec_frl_state_lts4: executes FRL LTS4 training state
 * @hdmi: Pointer to HDMI TX core instance
 *
 * Returns: 0 on success, non-zero value if ddc transaction fails.
 */
static int xlnx_hdmi_exec_frl_state_lts4(struct xlnx_hdmi *hdmi)
{
	int status = 0;

	xlnx_hdmi_set_frl_timer(hdmi, 0);
	xlnx_hdmi_clear_frl_ltp(hdmi);

	if (hdmi->stream.frl_config.max_frl_rate > 1) {
		hdmi->config.max_frl_rate = hdmi->config.max_frl_rate - 1;
		hdmi->stream.sink_max_linerate =
			rate_table[hdmi->config.max_frl_rate].linerate;
		hdmi->stream.sink_max_lanes = rate_table[hdmi->config.max_frl_rate].lanes;
		status = 0;
	} else {
		status = 1;
	}

	if (!status) {
		status = xlnx_hdmi_ddcwrite_field(hdmi,
						HDMI_TX_SCDC_FIELD_FLT_UPDATE, 1);
		if (!status) {
			hdmi->stream.frl_config.timer_cnt = 0;
			hdmi->stream.frl_config.frl_train_states =
				HDMI_TX_FRLSTATE_LTS_3_ARM;
			xlnx_hdmi_frl_config(hdmi);
		}
	} else {
		hdmi->stream.frl_config.timer_cnt = 0;
		hdmi->stream.frl_config.frl_train_states = HDMI_TX_FRLSTATE_LTS_L;
		xlnx_hdmi_set_frl_timer(hdmi, TIMEOUT_10US);
	}

	xlnx_hdmi_frl_execute(hdmi);

	return status;
}

/**
 * xlnx_hdmi_exec_frl_state_ltsp_arm: executes FRL LTSP-arm training state
 * @hdmi: pointer to HDMI TX core instance.
 *
 * Returns: 0 on success, non-zero value if ddc transaction fails.
 */
static int xlnx_hdmi_exec_frl_state_ltsp_arm(struct xlnx_hdmi *hdmi)
{
	int status;

	xlnx_hdmi_clear_frl_ltp(hdmi);
	/* Send GAP characters */
	xlnx_hdmi_set_frl_active(hdmi, HDMI_TX_FRL_ACTIVE_MODE_GAP_ONLY);
	status = xlnx_hdmi_ddcwrite_field(hdmi,
					  HDMI_TX_SCDC_FIELD_FLT_UPDATE, 1);
	hdmi->stream.frl_config.frl_train_states = HDMI_TX_FRLSTATE_LTS_P;

	return status;
}

/**
 * xlnx_hdmi_exec_frl_state_ltsp: executes FRL LTS-P training state
 * @hdmi: pointer to HDMI TX core instance.
 *
 * Returns: 0 on success, non-zero value if ddc transaction fails.
 */
static int xlnx_hdmi_exec_frl_state_ltsp(struct xlnx_hdmi *hdmi)
{
	int status;
	u8 ddc_buf;

	if (hdmi->stream.frl_config.frl_train_states !=
	    HDMI_TX_FRLSTATE_LTS_P_FRL_RDY)
		xlnx_hdmi_set_frl_timer(hdmi, TIMEOUT_2MS);
	else
		xlnx_hdmi_set_frl_timer(hdmi, TIMEOUT_250MS);

	status = xlnx_hdmi_ddc_readreg(hdmi, HDMI_TX_DDC_SLAVEADDR, 1,
				       HDMI_TX_DDC_UPDATE_FLGS_REG,
				       (u8 *)&ddc_buf);
	if (status)
		return status;

	if (hdmi->stream.frl_config.frl_train_states ==
	    HDMI_TX_FRLSTATE_LTS_P) {
		if (ddc_buf & HDMI_TX_DDC_UPDATE_FLGS_FRL_START_MASK) {
			xlnx_hdmi_set_frl_timer(hdmi, TIMEOUT_250MS);
			status = xlnx_hdmi_ddcwrite_field(hdmi,
							  HDMI_TX_SCDC_FIELD_FRL_START,
							  1);
			if (!status) {
				hdmi->stream.frl_config.frl_train_states =
					HDMI_TX_FRLSTATE_LTS_P_FRL_RDY;
				hdmi->wait_for_streamup =
					xlnx_hdmi_is_lnk_vid_rdy(hdmi) ? 1 : 0;
				wake_up(&hdmi->wait_event);
			}
		}
	}

	if (ddc_buf & HDMI_TX_DDC_UPDATE_FLGS_FLT_UPDATE_MASK) {
		/* Stops transmitting link training pattern */
		xlnx_hdmi_clear_frl_ltp(hdmi);
		/* Stops transmitting video, audio and control packets */
		xlnx_hdmi_set_frl_active(hdmi,
					 HDMI_TX_FRL_ACTIVE_MODE_GAP_ONLY);
		hdmi->stream.frl_config.timer_cnt = 0;
		hdmi->stream.frl_config.frl_train_states =
			HDMI_TX_FRLSTATE_LTS_3;
		xlnx_hdmi_set_frl_timer(hdmi, TIMEOUT_10US);
	} else if (ddc_buf & HDMI_TX_DDC_UPDATE_FLGS_CED_UPDATE_MASK) {
		xlnx_hdmi_set_frl_timer(hdmi, 0);
	}

	return status;
}

/**
 * xlnx_hdmi_exec_frl_state: executes states of FRL.
 * @hdmi: pointer to HDMI TX core instance
 *
 * Returns: 0 on success, non-zero value on failure
 */
static int xlnx_hdmi_exec_frl_state(struct xlnx_hdmi *hdmi)
{
	int status = 1;

	xlnx_hdmi_set_frl_timer(hdmi, 0);
	xlnx_hdmi_frl_intr_enable(hdmi);
	xlnx_hdmi_frl_execute(hdmi);

	switch (hdmi->stream.frl_config.frl_train_states) {
	case HDMI_TX_FRLSTATE_LTS_L:
		status = xlnx_hdmi_exec_frl_state_ltsl(hdmi);
		break;
	case HDMI_TX_FRLSTATE_LTS_1:
		status = xlnx_hdmi_exec_frl_state_lts1(hdmi);
		break;
	case HDMI_TX_FRLSTATE_LTS_2:
		status = xlnx_hdmi_exec_frl_state_lts2(hdmi);
		break;
	case HDMI_TX_FRLSTATE_LTS_3_ARM:
		status = xlnx_hdmi_exec_frl_state_lts2_ratewr(hdmi);
		break;
	case HDMI_TX_FRLSTATE_LTS_3:
		status = xlnx_hdmi_exec_frl_state_lts3(hdmi);
		break;
	case HDMI_TX_FRLSTATE_LTS_4:
		status = xlnx_hdmi_exec_frl_state_lts4(hdmi);
		break;
	case HDMI_TX_FRLSTATE_LTS_P_ARM:
		status = xlnx_hdmi_exec_frl_state_ltsp_arm(hdmi);
		if (!status)
			status = xlnx_hdmi_exec_frl_state_ltsp(hdmi);
		break;
	case HDMI_TX_FRLSTATE_LTS_P:
		status = xlnx_hdmi_exec_frl_state_ltsp(hdmi);
		break;
	case HDMI_TX_FRLSTATE_LTS_P_FRL_RDY:
		status = xlnx_hdmi_exec_frl_state_ltsp(hdmi);
		break;
	default:
		dev_dbg(hdmi->dev, "TX:S:FRL_INVALID_STATE!\n");
		break;
	}
	/* Clear timer event flag */
	hdmi->stream.frl_config.timer_event = false;

	return status;
}

/**
 * xlnx_hdmi_start_frl_train - starts the Fixed Rate Link Training.
 * @hdmi: pointer to the HDMI Tx core instance.
 *
 * Returns: 0 on success, 1 on failure.
 */
static int
xlnx_hdmi_start_frl_train(struct xlnx_hdmi *hdmi)
{
	int status;

	hdmi->stream.frl_config.frl_train_states = HDMI_TX_FRLSTATE_LTS_1;
	hdmi->stream.frl_config.timer_event = false;

	status = xlnx_hdmi_exec_frl_state(hdmi);

	return status;
}

/**
 * xlnx_hdmi_piointr_handler - HDMI TX peripheral interrupt handler.
 * @hdmi: pointer to HDMI TX core instance
 *
 * Returns: None.
 *
 * This handler reads corresponding event interrupt from the PIO_IN_EVT
 * register. It determines the source of the interrupt
 */
static void xlnx_hdmi_piointr_handler(struct xlnx_hdmi *hdmi)
{
	u32 event, data;

	/* Read PIO IN Event register */
	event = xlnx_hdmi_readl(hdmi, HDMI_TX_PIO_IN_EVT);

	/* Clear event flags */
	xlnx_hdmi_writel(hdmi, HDMI_TX_PIO_IN_EVT, event);

	/* Read data */
	data = xlnx_hdmi_readl(hdmi, HDMI_TX_PIO_IN);

	/* HPD event has occurred */
	if (event & HDMI_TX_PIO_IN_HPD_TOGGLE)
		xlnx_hdmi_stream_start(hdmi);

	/* HPD event has occurred */
	if (event & HDMI_TX_PIO_IN_HPD_CONNECT) {
		/* Check the HPD status */
		if (data & HDMI_TX_PIO_IN_HPD_CONNECT) {
			hdmi->cable_connected = 1;
			hdmi->connector.status = connector_status_connected;
		} else {
			hdmi->cable_connected = 0;
			hdmi->connector.status = connector_status_disconnected;
			dev_info(hdmi->dev, "stream is not connected\n");
			xlnx_hdmi_streamdown_callback(hdmi);
		}
		xlnx_hdmi_connect_callback(hdmi);
		if (hdmi->connector.dev)
			drm_sysfs_hotplug_event(hdmi->connector.dev);
		else
			dev_dbg(hdmi->dev, "Not sending HOTPLUG.\n");
	}

	/* Bridge Unlocked event has occurred */
	if (event & HDMI_TX_PIO_IN_BRIDGE_LOCKED) {
		dev_dbg(hdmi->dev, "PIO IN status = 0x%x\n",
			xlnx_hdmi_readl(hdmi, HDMI_TX_PIO_IN));
		if (data & HDMI_TX_PIO_IN_BRIDGE_LOCKED) {
			dev_dbg(hdmi->dev, "Bridge locked\n");
		} else {
			dev_dbg(hdmi->dev, "Bridge unlocked\n");
			/* Clear interrupt and FRL */
			xlnx_hdmi_set_frl_timer(hdmi, 0);
			xlnx_hdmi_frl_reset_assert(hdmi);
			xlnx_hdmi_frl_reset_deassert(hdmi);
			xlnx_hdmi_frl_mode_disable(hdmi);
			hdmi->stream.is_frl = 0;
			xlnx_hdmi_ddcwrite_field(hdmi,
						 HDMI_TX_SCDC_FIELD_FLT_UPDATE,
						 1);
			xlnx_hdmi_piointr_clear(hdmi);
		}
	}

	/* Bridge Overflow event has occurred */
	if (event & HDMI_TX_PIO_IN_BRIDGE_OFLOW)
		dev_err_ratelimited(hdmi->dev, "Overflow interrupt\n");

	/* Bridge Underflow event has occurred */
	if (event & HDMI_TX_PIO_IN_BRIDGE_UFLOW)
		dev_err_ratelimited(hdmi->dev, "Underflow interrupt\n");

	/* vsync event has occurred */
	if (event & HDMI_TX_PIO_IN_VS) {
		dev_dbg_ratelimited(hdmi->dev, "Vsync interrupt\n");
		xlnx_hdmi_vsync_event_handler(hdmi);
	}
	/* Link ready event has occurred */
	if (event & HDMI_TX_PIO_IN_LNK_RDY) {
		/* Check the link status */
		if (data & HDMI_TX_PIO_IN_LNK_RDY) {
			if (hdmi->stream.is_frl) {
				hdmi->stream.state = HDMI_TX_STATE_STREAM_UP;
				if (hdmi->stream.frl_config.frl_train_states ==
				    HDMI_TX_FRLSTATE_LTS_3_ARM) {
					/* Execute state machine */
					xlnx_hdmi_exec_frl_state(hdmi);
				}
			} else {
				xlnx_hdmi_streamup_callback(hdmi);
			}
			xlnx_hdmi_aux_enable(hdmi);
			xlnx_hdmi_auxintr_enable(hdmi);
		} else {
			/* Set stream status to down */
			hdmi->stream.state = HDMI_TX_STATE_STREAM_DOWN;
			/* Disable AUX */
			xlnx_hdmi_aux_disable(hdmi);
		}
	}
}

/**
 * xlnx_hdmi_frlintr_handler - HDMI TX FRL interrupt handler.
 * @hdmi: pointer to HDMI TX core instance
 */
static void xlnx_hdmi_frlintr_handler(struct xlnx_hdmi *hdmi)
{
	u32 data;

	/* Read FRL Status register */
	data = xlnx_hdmi_readl(hdmi, HDMI_TX_FRL_STA);

	/* Check FRL timer event */
	if ((data) & (HDMI_TX_FRL_STA_TMR_EVT)) {
		xlnx_hdmi_writel(hdmi, HDMI_TX_FRL_STA,
				 HDMI_TX_FRL_STA_TMR_EVT);
		/* Set Timer event flag */
		hdmi->stream.frl_config.timer_event = true;

		/* Execute state machine */
		xlnx_hdmi_exec_frl_state(hdmi);
	}
}

static irqreturn_t hdmitx_irq_handler(int irq, void *dev_id)
{
	struct xlnx_hdmi *hdmi = (struct xlnx_hdmi *)dev_id;
	unsigned long flags;

	/* read status registers */
	hdmi->intr_status = xlnx_hdmi_readl(hdmi, HDMI_TX_PIO_STA);
	hdmi->intr_status &= HDMI_TX_PIO_STA_IRQ;
	if (hdmi->stream.is_frl) {
		hdmi->frl_status = xlnx_hdmi_readl(hdmi, HDMI_TX_FRL_STA);
		hdmi->frl_status &= HDMI_TX_FRL_STA_IRQ;
	}

	spin_lock_irqsave(&hdmi->irq_lock, flags);
	xlnx_hdmi_piointr_disable(hdmi);
	if (hdmi->frl_status) {
		xlnx_hdmi_frl_intr_disable(hdmi);
		xlnx_hdmi_frl_execute(hdmi);
	}
	spin_unlock_irqrestore(&hdmi->irq_lock, flags);

	return IRQ_WAKE_THREAD;
}

static irqreturn_t hdmitx_irq_thread(int irq, void *data)
{
	struct xlnx_hdmi *hdmi = (struct xlnx_hdmi *)data;
	unsigned long flags;

	if (!hdmi)
		return IRQ_HANDLED;

	hdmi_mutex_lock(&hdmi->hdmi_mutex);

	if (hdmi->intr_status)
		xlnx_hdmi_piointr_handler(hdmi);

	if (hdmi->frl_status && hdmi->stream.is_frl)
		xlnx_hdmi_frlintr_handler(hdmi);

	hdmi->cable_connected = 1;

	hdmi_mutex_unlock(&hdmi->hdmi_mutex);

	spin_lock_irqsave(&hdmi->irq_lock, flags);
	xlnx_hdmi_piointr_ie_enable(hdmi);
	spin_unlock_irqrestore(&hdmi->irq_lock, flags);

	return IRQ_HANDLED;
}

/* DRM connector functions */
static enum drm_connector_status
xlnx_hdmi_connector_detect(struct drm_connector *connector, bool force)
{
	/* it takes HDMI 50 ms to detect connection on init */
	static int first_time_ms = 50;
	struct xlnx_hdmi *hdmi = connector_to_hdmi(connector);

	/* first time; wait 50 ms max until cable connected */
	while (first_time_ms && !hdmi->cable_connected) {
		msleep(20);
		first_time_ms--;
	}

	/* connected in less than 50 ms? */
	if (first_time_ms) {
		/* after first time, report immediately */
		dev_info(hdmi->dev, "detect() waited %d ms until connect.\n",
			 50 - first_time_ms);
		first_time_ms = 0;
	}

	hdmi_mutex_lock(&hdmi->hdmi_mutex);
	if (hdmi->cable_connected) {
		hdmi_mutex_unlock(&hdmi->hdmi_mutex);
		dev_dbg(hdmi->dev, "hdmi_connector_detect() = connected\n");
		return connector_status_connected;
	}

	hdmi_mutex_unlock(&hdmi->hdmi_mutex);
	dev_dbg(hdmi->dev, "hdmi_connector_detect() = disconnected\n");

	return connector_status_disconnected;
}

static void xlnx_hdmi_connector_destroy(struct drm_connector *connector)
{
	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
	connector->dev = NULL;
}

static int xlnx_hdmi_set_property(struct drm_connector *connector,
				  struct drm_connector_state *state,
				  struct drm_property *property,
				  uint64_t val)
{
	struct xlnx_hdmi *hdmi = connector_to_hdmi(connector);

	if (property == hdmi->height_out)
		hdmi->height_out_prop_val = (u32)val;
	else if (property == hdmi->width_out)
		hdmi->width_out_prop_val = (u32)val;
	else if (property == hdmi->in_fmt)
		hdmi->in_fmt_prop_val = (u32)val;
	else if (property == hdmi->out_fmt)
		hdmi->out_fmt_prop_val = (u32)val;
	else
		return -EINVAL;

	return 0;
}

static int xlnx_hdmi_get_property(struct drm_connector *connector,
				  const struct drm_connector_state *state,
				  struct drm_property *property,
				  uint64_t *val)
{
	struct xlnx_hdmi *hdmi = connector_to_hdmi(connector);

	if (property == hdmi->height_out)
		*val = hdmi->height_out_prop_val;
	else if (property == hdmi->width_out)
		*val = hdmi->width_out_prop_val;
	else if (property == hdmi->in_fmt)
		*val = hdmi->in_fmt_prop_val;
	else if (property == hdmi->out_fmt)
		*val = hdmi->out_fmt_prop_val;
	else
		return -EINVAL;

	return 0;
}

static const struct drm_connector_funcs xlnx_hdmi_connector_funcs = {
	.dpms			= drm_helper_connector_dpms,
	.detect			= xlnx_hdmi_connector_detect,
	.fill_modes		= drm_helper_probe_single_connector_modes,
	.destroy		= xlnx_hdmi_connector_destroy,
	.atomic_duplicate_state	= drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_connector_destroy_state,
	.reset			= drm_atomic_helper_connector_reset,
	.atomic_set_property	= xlnx_hdmi_set_property,
	.atomic_get_property	= xlnx_hdmi_get_property,
};

/* DRM connector helper functions */
static int
xlnx_hdmi_connector_mode_valid(struct drm_connector *connector,
			       struct drm_display_mode *mode)
{
	struct xlnx_hdmi *hdmi = connector_to_hdmi(connector);
	enum drm_mode_status status = MODE_OK;

	if (mode->flags & DRM_MODE_FLAG_INTERLACE) {
		mode->vdisplay = mode->vdisplay / 2;
		dev_dbg(hdmi->dev, "INTERLACE, mode->vdisplay %d\n",
			mode->vdisplay);
	}

	if ((mode->flags & DRM_MODE_FLAG_DBLCLK) &&
	    (mode->flags & DRM_MODE_FLAG_INTERLACE)) {
		mode->clock *= 2;
		dev_dbg(hdmi->dev, "clock = %d, refresh rate = %d\n",
			mode->clock, drm_mode_vrefresh(mode));
	}

	drm_mode_debug_printmodeline(mode);
	hdmi_mutex_lock(&hdmi->hdmi_mutex);

	/* pixel clock too high for sink? */
	if (mode->clock > HDMI_TX_PIXEL_MAXRATE)
		status = MODE_CLOCK_HIGH;
	hdmi_mutex_unlock(&hdmi->hdmi_mutex);

	return status;
}

/**
 * xlnx_hdmi_get_edid_block - callback function for drm_do_get_edid() used in
 * get_modes through drm_do_get_edid() from drm/drm_edid.c.
 *
 * @data: pointer to hdmi instance
 * @buf: buffer pointer to copy edid data
 * @block: edid block
 * @len: length of the data to be read
 *
 * @return: 0 on success, error code otherwise
 */
static int
xlnx_hdmi_get_edid_block(void *data, u8 *buf, unsigned int block,
			 size_t len)
{
	u8 *buffer;
	struct xlnx_hdmi *hdmi = data;
	int ret = 0;

	/* out of bounds? */
	if (((block * 128) + len) > HDMI_TX_DDC_EDID_LENGTH)
		return -EINVAL;

	buffer = kzalloc(HDMI_TX_DDC_EDID_LENGTH, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	/* first obtain edid in local buffer */
	*buffer = 0;
	ret = xlnx_hdmi_ddcwrite(hdmi, HDMI_TX_DDC_ADDR, 1, buffer, false);

	if (!ret) {
		ret = xlnx_hdmi_ddcread(hdmi, HDMI_TX_DDC_ADDR,
					HDMI_TX_DDC_EDID_LENGTH, buffer, true);
	} else {
		kfree(buffer);
		dev_err(hdmi->dev, "failed reading EDID\n");
		return -EINVAL;
	}

	memcpy(buf, buffer + block * 128, len);

	kfree(buffer);
	return 0;
}

/**
 * xlnx_hdmi_set_frl_tmds_mode - Function sets the supported mode (FRL/TMDS)
 * by the connectd sink device. Also gets the max_frl_rate supportd by sink.
 *
 * @connector: pointer to drm connector instance
 */
static void
xlnx_hdmi_set_frl_tmds_mode(struct drm_connector *connector)
{
	struct xlnx_hdmi *hdmi = connector_to_hdmi(connector);

	if (connector->display_info.hdmi.max_lanes != 0 &&
	    connector->display_info.hdmi.max_frl_rate_per_lane != 0) {
		hdmi->stream.is_frl = 1;
	} else {
		hdmi->stream.is_frl = 0;
	}
}

static int xlnx_hdmi_connector_get_modes(struct drm_connector *connector)
{
	struct xlnx_hdmi *hdmi = connector_to_hdmi(connector);
	const struct drm_edid *drm_edid;
	const struct edid *edid;
	int ret;
	bool is_hdmi_sink;

	hdmi_mutex_lock(&hdmi->hdmi_mutex);

	drm_edid = drm_edid_read_custom(connector, xlnx_hdmi_get_edid_block, hdmi);

	/* Set HDMI FRL or TMDS Mode */
	xlnx_hdmi_set_frl_tmds_mode(connector);

	hdmi_mutex_unlock(&hdmi->hdmi_mutex);
	drm_edid_connector_update(connector, drm_edid);

	if (!drm_edid) {
		dev_info(hdmi->dev, "no edid, assume <= 1024x768 works\n");
		return 0;
	}

	/*
	 * FIXME: This should use connector->display_info.is_hdmi from a
	 * path that has read the EDID and called
	 * drm_edid_connector_update().
	 */
	edid = drm_edid_raw(drm_edid);

	/* If the sink is non HDMI, set the stream type to DVI else HDMI */
	is_hdmi_sink = drm_detect_hdmi_monitor(edid);
	if (is_hdmi_sink) {
		dev_dbg(hdmi->dev, "setting stream type to HDMI\n");
		xlnx_hdmi_set_hdmi_mode(hdmi);
		hdmi->stream.is_hdmi = true;
		if (hdmi->stream.is_hdmi)
			xlnx_hdmi_aux_enable(hdmi);
	} else {
		dev_dbg(hdmi->dev, "setting stream type to DVI\n");
	}

	ret = drm_edid_connector_add_modes(connector);
	kfree(edid);

	return ret;
}

static struct drm_encoder *
xlnx_hdmi_connector_best_encoder(struct drm_connector *connector)
{
	struct xlnx_hdmi *hdmi = connector_to_hdmi(connector);

	return &hdmi->encoder;
}

static struct
drm_connector_helper_funcs xlnx_hdmi_connector_helper_funcs = {
	.get_modes = xlnx_hdmi_connector_get_modes,
	.best_encoder = xlnx_hdmi_connector_best_encoder,
	.mode_valid = xlnx_hdmi_connector_mode_valid,
};

/* DRM encoder functions */
static void xlnx_hdmi_encoder_dpms(struct drm_encoder *encoder, int dpms)
{
	struct xlnx_hdmi *hdmi = encoder_to_hdmi(encoder);

	hdmi_mutex_lock(&hdmi->hdmi_mutex);
	hdmi->dpms = dpms;
	hdmi_mutex_unlock(&hdmi->hdmi_mutex);
}

static void xlnx_hdmi_encoder_enable(struct drm_encoder *encoder)
{
	struct xlnx_hdmi *hdmi = encoder_to_hdmi(encoder);
	struct xlnx_hdmi_config *config = &hdmi->config;

	xlnx_hdmi_encoder_dpms(encoder, DRM_MODE_DPMS_ON);
	if (xlnx_hdmi_is_lnk_vid_rdy(hdmi)) {
		if (!config->vid_interface)
			xlnx_hdmi_vtc_enable(hdmi);
	} else {
		dev_err(hdmi->dev, "No video/link clock! failed to enable vtc\n");
	}

	xlnx_hdmi_ext_sysrst_deassert(hdmi);
}

static void xlnx_hdmi_encoder_disable(struct drm_encoder *encoder)
{
	struct xlnx_hdmi *hdmi = encoder_to_hdmi(encoder);
	struct xlnx_hdmi_config *config = &hdmi->config;

	if (hdmi->bridge)
		xlnx_bridge_disable(hdmi->bridge);

	xlnx_hdmi_encoder_dpms(encoder, DRM_MODE_DPMS_OFF);

	/* Disable the EXT VRST which actually starts the bridge */
	xlnx_hdmi_ext_sysrst_assert(hdmi);
	if (xlnx_hdmi_is_lnk_vid_rdy(hdmi)) {
		if (!config->vid_interface)
			xlnx_hdmi_vtc_disable(hdmi);
	} else {
		dev_err(hdmi->dev, "No video/link clock! failed to disable vtc\n");
	}
}

/**
 * xlnx_hdmi_find_media_bus - finds drm_fourcc equivalent format
 * @hdmi: pointer to HDMI TX core instance
 * @drm_fourcc: drm fourcc code
 *
 * Returns: equivalent media bus format
 */
static enum
color_formats xlnx_hdmi_find_media_bus(struct xlnx_hdmi *hdmi,
				       u32 drm_fourcc)
{
	switch (drm_fourcc) {
	case DRM_FORMAT_XBGR8888:
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_BGR888:
	case DRM_FORMAT_RGB888:
	case DRM_FORMAT_ARGB8888:
	case DRM_FORMAT_ABGR8888:
	case MEDIA_BUS_FMT_RGB888_1X24:
		hdmi->xvidc_colordepth = HDMI_TX_BPC_8;
		return HDMI_TX_CSF_RGB;
	case DRM_FORMAT_XBGR2101010:
		hdmi->xvidc_colordepth = HDMI_TX_BPC_10;
		return HDMI_TX_CSF_RGB;
	case DRM_FORMAT_VUY888:
	case DRM_FORMAT_XVUY8888:
	case DRM_FORMAT_Y8:
	case MEDIA_BUS_FMT_VUY8_1X24:
		hdmi->xvidc_colordepth = HDMI_TX_BPC_8;
		return HDMI_TX_CSF_YCRCB_444;
	case DRM_FORMAT_XVUY2101010:
	case DRM_FORMAT_Y10:
		hdmi->xvidc_colordepth = HDMI_TX_BPC_10;
		return HDMI_TX_CSF_YCRCB_444;
	case DRM_FORMAT_YUYV:
	case DRM_FORMAT_UYVY:
	case DRM_FORMAT_NV16:
	case MEDIA_BUS_FMT_UYVY8_1X16:
		hdmi->xvidc_colordepth = HDMI_TX_BPC_8;
		return HDMI_TX_CSF_YCRCB_422;
	case DRM_FORMAT_XV20:
		hdmi->xvidc_colordepth = HDMI_TX_BPC_10;
		return HDMI_TX_CSF_YCRCB_422;
	case DRM_FORMAT_NV12:
	case MEDIA_BUS_FMT_VYYUYY8_1X24:
		hdmi->xvidc_colordepth = HDMI_TX_BPC_8;
		return HDMI_TX_CSF_YCRCB_420;
	case DRM_FORMAT_XV15:
		hdmi->xvidc_colordepth = HDMI_TX_BPC_10;
		return HDMI_TX_CSF_YCRCB_420;
	default:
		dev_err(hdmi->dev, "Unknown drm fmt: %d\n", drm_fourcc);
		hdmi->xvidc_colordepth = HDMI_TX_BPC_8;
		return HDMI_TX_CSF_RGB;
	}
}

static u64 xlnx_hdmi_get_tmdsclk(struct xlnx_hdmi *hdmi, struct drm_display_mode *adjusted_mode)
{
	u64 tmdsclk;

	tmdsclk = adjusted_mode->clock * 1000;
	if (hdmi->xvidc_colorfmt == HDMI_TX_CSF_YCRCB_420)
		tmdsclk = tmdsclk >> 1;

	if (hdmi->xvidc_colorfmt != HDMI_TX_CSF_YCRCB_422) {
		switch (hdmi->config.bpc) {
		case HDMI_TX_BPC_10:
			tmdsclk = (tmdsclk * 5) >> 2;
			break;
		case HDMI_TX_BPC_12:
			tmdsclk = (tmdsclk * 3) >> 1;
			break;
		case HDMI_TX_BPC_16:
			tmdsclk = tmdsclk << 1;
			break;
		default:
			break;
		}
	}

	return tmdsclk;
}

/**
 * xlnx_hdmi_encoder_atomic_mode_set - drive the HDMI timing parameters
 *
 * @encoder: pointer to Xilinx DRM encoder
 * @crtc_state: DRM crtc state
 * @connector_state: DRM connector state
 *
 * This function derives the HDMI IP timing parameters from the timing
 * values given to timing module.
 */
static void
xlnx_hdmi_encoder_atomic_mode_set(struct drm_encoder *encoder,
				  struct drm_crtc_state *crtc_state,
				  struct drm_connector_state *connector_state)
{
	struct xlnx_hdmi *hdmi = encoder_to_hdmi(encoder);
	struct drm_connector *connector = &hdmi->connector;
	struct xlnx_hdmi_config *config = &hdmi->config;
	struct drm_display_mode *mode = &crtc_state->mode;
	struct drm_display_mode *adjusted_mode = &crtc_state->adjusted_mode;
	union phy_configure_opts phy_cfg = {0};
	int ret;
	int source_max_frl_bw, sink_max_frl_bw, max_frl_bw;
	u32 drm_fourcc, pixelrate = 0;
	u64 lnk_clk = 0, vid_clk = 0;

	drm_mode_copy(&hdmi->saved_adjusted_mode, &crtc_state->adjusted_mode);
	dev_dbg(hdmi->dev, "mode->clock = %d\n", mode->clock * 1000);
	dev_dbg(hdmi->dev, "mode->crtc_clock = %d\n", mode->crtc_clock * 1000);
	dev_dbg(hdmi->dev, "mode->pvsync = %d\n",
		!!(mode->flags & DRM_MODE_FLAG_PVSYNC));
	dev_dbg(hdmi->dev, "mode->phsync = %d\n",
		!!(mode->flags & DRM_MODE_FLAG_PHSYNC));
	dev_dbg(hdmi->dev, "mode->hsync_end = %d\n", mode->hsync_end);
	dev_dbg(hdmi->dev, "mode->hsync_start = %d\n", mode->hsync_start);
	dev_dbg(hdmi->dev, "mode->vsync_end = %d\n", mode->vsync_end);
	dev_dbg(hdmi->dev, "mode->vsync_start = %d\n", mode->vsync_start);
	dev_dbg(hdmi->dev, "mode->hdisplay = %d\n", mode->hdisplay);
	dev_dbg(hdmi->dev, "mode->vdisplay = %d\n", mode->vdisplay);
	dev_dbg(hdmi->dev, "mode->htotal = %d\n", mode->htotal);
	dev_dbg(hdmi->dev, "mode->vtotal = %d\n", mode->vtotal);
	dev_dbg(hdmi->dev, "mode->vrefresh = %d\n", drm_mode_vrefresh(mode));
	dev_dbg(hdmi->dev, "mode->flags = %d interlace = %d\n", mode->flags,
		!!(mode->flags & DRM_MODE_FLAG_INTERLACE));

	source_max_frl_bw = hdmi->config.max_frl_rate;
	sink_max_frl_bw = xlnx_hdmi_sink_max_frl(hdmi);
	max_frl_bw = min(sink_max_frl_bw, source_max_frl_bw);
	if (max_frl_bw <= 0) {
		hdmi->stream.is_frl = 0;
		dev_dbg(hdmi->dev, "Connected sink supports TMDS mode\n");
	} else {
		dev_dbg(hdmi->dev, "Connected sink supports FRL mode\n");
		hdmi->stream.is_frl = 1;
	}

	if (hdmi->stream.is_frl) {
		xlnx_hdmi_frl_reset_deassert(hdmi);
		xlnx_hdmi_frl_intr_enable(hdmi);
		xlnx_hdmi_frl_execute(hdmi);

		hdmi->stream.frl_config.lanes = rate_table[max_frl_bw].lanes;
		hdmi->stream.frl_config.linerate = rate_table[max_frl_bw].linerate;
	} else {
		xlnx_hdmi_frl_ext_vidsrc(hdmi);
		xlnx_hdmi_frl_sleep(hdmi);
	}

	drm_fourcc = encoder->crtc->primary->state->fb->format->format;

	if (hdmi->bridge) {
		/*
		 * TODO: Add a check for valid values of width_out,
		 * height_out and out_fmt values based on sink
		 * capabilities.
		 */
		xlnx_bridge_set_input(hdmi->bridge, adjusted_mode->hdisplay,
				      adjusted_mode->vdisplay,
				      hdmi->in_fmt_prop_val);
		xlnx_bridge_set_output(hdmi->bridge, hdmi->width_out_prop_val,
				       hdmi->height_out_prop_val,
				       hdmi->out_fmt_prop_val);
		xlnx_bridge_enable(hdmi->bridge);

		drm_fourcc = hdmi->out_fmt_prop_val;
		adjusted_mode = drm_mode_find_cea(connector->dev,
						  hdmi->width_out_prop_val,
						  hdmi->height_out_prop_val,
						  drm_mode_vrefresh(adjusted_mode),
						  adjusted_mode->flags & DRM_MODE_FLAG_INTERLACE);
		if (!adjusted_mode) {
			dev_err(hdmi->dev, "Invalid CEA mode\n");
			return;
		}
	}

	hdmi->xvidc_colorfmt = xlnx_hdmi_find_media_bus(hdmi, drm_fourcc);
	dev_dbg(hdmi->dev, "xvidc_colorfmt = %d\n", hdmi->xvidc_colorfmt);
	dev_dbg(hdmi->dev, "xvidc_colordepth = %d\n", hdmi->xvidc_colordepth);

	hdmi->tmds_clk = xlnx_hdmi_get_tmdsclk(hdmi, adjusted_mode);
	dev_dbg(hdmi->dev, "tmds_clk = %llu\n", hdmi->tmds_clk);

	if (connector->display_info.is_hdmi)
		xlnx_hdmi_send_infoframes(hdmi);

	if (hdmi->stream.is_frl) {
		phy_cfg.hdmi.clkout1_obuftds = 1;
		phy_cfg.hdmi.clkout1_obuftds_en = false;
		ret = xlnx_hdmi_phy_configure(hdmi, &phy_cfg);
		if (ret) {
			dev_err(hdmi->dev, "phy_cfg:10bufds_en err %d\n", ret);
			return;
		}

		ret = xlnx_hdmi_start_frl_train(hdmi);
		if (ret) {
			dev_err(hdmi->dev, "FRL training is failed.switch to TMDS mode \r\n");
			xlnx_hdmi_set_hdmi_mode(hdmi);
			hdmi->stream.is_hdmi = true;
			hdmi->stream.is_frl = false;
			xlnx_hdmi_auxintr_enable(hdmi);
		} else {
			dev_dbg(hdmi->dev, "FRL training passed !!\n");
		}
	}

	xlnx_hdmi_stream_start(hdmi);
	/* get tmds clock from phy */
	if (!hdmi->stream.is_frl) {
		xlnx_hdmi_clkratio(hdmi);

		/* Assert VID_IN bridge resets */
		xlnx_hdmi_ext_sysrst_assert(hdmi);
		xlnx_hdmi_ext_vrst_assert(hdmi);

		/* Assert HDMI TXCore resets */
		xlnx_hdmi_int_lrst_assert(hdmi);

		phy_cfg.hdmi.tx_params = 1;
		phy_cfg.hdmi.ppc = config->ppc;
		phy_cfg.hdmi.bpc = config->bpc;
		phy_cfg.hdmi.fmt = hdmi->xvidc_colorfmt;
		phy_cfg.hdmi.tx_tmdsclk = hdmi->tmds_clk;
		ret = xlnx_hdmi_phy_configure(hdmi, &phy_cfg);
		if (ret) {
			dev_err(hdmi->dev, "phy_config: set txparams error %d\n", ret);
			return;
		}
	} else {
		if (hdmi->xvidc_colorfmt == HDMI_TX_CSF_YCRCB_422) {
			vid_clk = div_u64(div_u64(hdmi->tmds_clk, config->ppc),
					  1000);
			lnk_clk = vid_clk;
		} else {
			pixelrate = div_u64(hdmi->tmds_clk * 8, config->bpc * 1000);
			vid_clk = div_u64(pixelrate, config->ppc);
			lnk_clk = div_u64(vid_clk * config->bpc, 8);
		}

		xlnx_set_frl_link_clk(hdmi, lnk_clk);
		xlnx_set_frl_vid_clk(hdmi, vid_clk);

		xlnx_hdmi_streamup_callback(hdmi);
	}

	dev_dbg(hdmi->dev, "tmds_clk = %llu Hz\n", hdmi->tmds_clk);

	hdmi->wait_for_streamup = 0;
	wait_event_timeout(hdmi->wait_event, hdmi->wait_for_streamup,
			   msecs_to_jiffies(HDMI_TX_LNK_VID_RDY_DELAY));
	if (!hdmi->wait_for_streamup)
		dev_err(hdmi->dev, "wait_for_streamup timeout\n");

	if (xlnx_hdmi_is_lnk_vid_rdy(hdmi)) {
		dev_dbg(hdmi->dev, "TX: Video ready interrupt received\n");
		if (!config->vid_interface)
			xlnx_hdmi_vtc_set_timing(hdmi, adjusted_mode);
		if (hdmi->stream.is_frl)
			xlnx_hdmi_vtc_writel(hdmi, HDMI_TX_VTC_CTL,
					     HDMI_TX_VTC_CTL_GE);
	} else {
		dev_dbg(hdmi->dev, "Video/Link clock is not ready\n");
	}
	if (hdmi->config.hdcp2x_enable || hdmi->config.hdcp1x_enable) {
		ret = xlnx_start_hdcp_engine(&hdmi->txhdcp,
					     HDMI_MAX_LANES);
		if (ret < 0) {
			dev_err(hdmi->dev, "Failed to Start HDCP engine\n");
			return;
		}
	}
	if (hdmi->stream.is_frl)
		xlnx_hdmi_set_frl_active(hdmi,
					 HDMI_TX_FRL_ACTIVE_MODE_FULL_STREAM);
	else
		xlnx_hdmi_ext_sysrst_assert(hdmi);
}

static const struct drm_encoder_funcs xlnx_hdmi_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static const struct
drm_encoder_helper_funcs xlnx_hdmi_encoder_helper_funcs = {
	.dpms = xlnx_hdmi_encoder_dpms,
	.enable = xlnx_hdmi_encoder_enable,
	.disable = xlnx_hdmi_encoder_disable,
	.atomic_mode_set = xlnx_hdmi_encoder_atomic_mode_set,
};

static void
xlnx_hdmi_create_connector_property(struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;
	struct xlnx_hdmi *hdmi = connector_to_hdmi(connector);

	hdmi->height_out = drm_property_create_range(dev, 0, "height_out",
						     HDMI_MIN_HEIGHT,
						     HDMI_MAX_HEIGHT);
	hdmi->width_out = drm_property_create_range(dev, 0, "width_out",
						    HDMI_MIN_WIDTH,
						    HDMI_MAX_WIDTH);
	hdmi->in_fmt = drm_property_create_range(dev, 0, "in_fmt",
						 MEDIA_BUS_FMT_RGB888_1X24,
						 MEDIA_BUS_FMT_VYYUYY8_1X24);
	hdmi->out_fmt = drm_property_create_range(dev, 0, "out_fmt",
						  MEDIA_BUS_FMT_RGB888_1X24,
						  MEDIA_BUS_FMT_VYYUYY8_1X24);
}

static void
xlnx_hdmi_attach_connector_property(struct drm_connector *connector)
{
	struct xlnx_hdmi *hdmi = connector_to_hdmi(connector);
	struct drm_mode_object *obj = &connector->base;

	if (hdmi->height_out)
		drm_object_attach_property(obj, hdmi->height_out, 0);
	if (hdmi->width_out)
		drm_object_attach_property(obj, hdmi->width_out, 0);
	if (hdmi->in_fmt)
		drm_object_attach_property(obj, hdmi->in_fmt, 0);
	if (hdmi->out_fmt)
		drm_object_attach_property(obj, hdmi->out_fmt, 0);
}

static int xlnx_hdmi_create_connector(struct drm_encoder *encoder)
{
	struct xlnx_hdmi *hdmi = encoder_to_hdmi(encoder);
	struct drm_connector *connector = &hdmi->connector;
	int ret;

	connector->polled = DRM_CONNECTOR_POLL_HPD;
	connector->interlace_allowed = true;

	ret = drm_connector_init(encoder->dev, connector,
				 &xlnx_hdmi_connector_funcs,
				 DRM_MODE_CONNECTOR_HDMIA);
	if (ret) {
		dev_err(hdmi->dev, "Failed to initialize connector with drm\n");
		return ret;
	}

	drm_connector_helper_add(connector, &xlnx_hdmi_connector_helper_funcs);
	ret = drm_connector_register(connector);
	if (ret) {
		dev_err(hdmi->dev,
			"Failed to register connector (ret=%d)\n", ret);
		return ret;
	}
	ret = drm_connector_attach_encoder(connector, encoder);
	if (ret) {
		dev_err(hdmi->dev,
			"Failed to attach connector (ret=%d)\n", ret);
		return ret;
	}

	xlnx_hdmi_create_connector_property(connector);
	xlnx_hdmi_attach_connector_property(connector);

	return 0;
}

static int xlnx_hdmi_bind(struct device *dev, struct device *master,
			  void *data)
{
	struct xlnx_hdmi *hdmi = dev_get_drvdata(dev);
	struct drm_encoder *encoder = &hdmi->encoder;
	struct drm_device *drm_dev = data;
	int ret;

	encoder->possible_crtcs = 1;
	/* initialize encoder */
	drm_encoder_init(drm_dev, encoder, &xlnx_hdmi_encoder_funcs,
			 DRM_MODE_ENCODER_TMDS, NULL);
	drm_encoder_helper_add(encoder, &xlnx_hdmi_encoder_helper_funcs);

	/* create connector */
	ret = xlnx_hdmi_create_connector(encoder);
	if (ret) {
		dev_err(hdmi->dev, "failed create connector, ret = %d\n", ret);
		drm_encoder_cleanup(encoder);
	}

	return ret;
}

static void xlnx_hdmi_unbind(struct device *dev, struct device *master,
			     void *data)
{
	struct xlnx_hdmi *hdmi = dev_get_drvdata(dev);

	if (hdmi->bridge)
		xlnx_bridge_disable(hdmi->bridge);

	xlnx_hdmi_encoder_dpms(&hdmi->encoder, DRM_MODE_DPMS_OFF);
	drm_encoder_cleanup(&hdmi->encoder);
	drm_connector_cleanup(&hdmi->connector);
}

static const struct component_ops xlnx_hdmi_component_ops = {
	.bind	= xlnx_hdmi_bind,
	.unbind	= xlnx_hdmi_unbind
};

/**
 * xlnx_hdmi_exit_phy - Exit the phy
 * @hdmi: HDMI core structure
 *
 * Exit the phy.
 */
static void xlnx_hdmi_exit_phy(struct xlnx_hdmi *hdmi)
{
	unsigned int i;
	int ret;

	for (i = 0; i < HDMI_MAX_LANES; i++) {
		ret = phy_exit(hdmi->phy[i]);
		if (ret)
			dev_err(hdmi->dev, "fail to exit phy(%d) %d\n", i, ret);
		hdmi->phy[i] = NULL;
	}
}

/**
 * xlnx_hdmi_initialize - Initializes the hdmi core
 * @hdmi: HDMI core strcture
 *
 * Return: 0 on success, error code on failure
 */
static int xlnx_hdmi_initialize(struct xlnx_hdmi *hdmi)
{
	union phy_configure_opts phy_cfg = {0};
	int ret;
	unsigned long val, clkrate;

	/* mutex that protects against concurrent access */
	mutex_init(&hdmi->hdmi_mutex);
	spin_lock_init(&hdmi->irq_lock);
	init_waitqueue_head(&hdmi->wait_event);

	/* set default color format to RGB */
	hdmi->xvidc_colorfmt = HDMI_TX_CSF_RGB;

	/* Reset all peripherals */
	xlnx_hdmi_piointr_disable(hdmi);
	xlnx_hdmi_ddc_disable(hdmi);
	xlnx_hdmi_audio_disable(hdmi);
	xlnx_hdmi_aux_disable(hdmi);
	xlnx_hdmi_frl_intr_disable(hdmi);
	xlnx_hdmi_frl_clear(hdmi);
	xlnx_hdmi_piointr_clear(hdmi);
	xlnx_hdmi_ddc_intr_clear(hdmi);

	/* PIO: Set event rising edge masks */
	xlnx_hdmi_writel(hdmi, HDMI_TX_PIO_IN_EVT_RE,
			 HDMI_TX_PIO_IN_BRIDGE_UFLOW |
			 HDMI_TX_PIO_IN_BRIDGE_OFLOW |
			 HDMI_TX_PIO_IN_BRIDGE_LOCKED |
			 HDMI_TX_PIO_IN_HPD_TOGGLE |
			 HDMI_TX_PIO_IN_HPD_CONNECT |
			 HDMI_TX_PIO_IN_VS |
			 HDMI_TX_PIO_IN_LNK_RDY);
	/* PIO: Set event falling edge masks */
	xlnx_hdmi_writel(hdmi, HDMI_TX_PIO_IN_EVT_FE,
			 HDMI_TX_PIO_IN_BRIDGE_LOCKED |
			 HDMI_TX_PIO_IN_HPD_CONNECT |
			 HDMI_TX_PIO_IN_LNK_RDY);

	/* Set the Timegrid for HPD */
	xlnx_hdmi_writel(hdmi, HDMI_TX_HPD_TIMEGRID, HDMI_TX_TIMEGRID_VAL);
	xlnx_hdmi_writel(hdmi, HDMI_TX_HPD_TOGGLE_CONF,
			 HDMI_TX_TOGGLE_CONF_VAL);
	xlnx_hdmi_writel(hdmi, HDMI_TX_HPD_CONNECT_CONF,
			 HDMI_TX_CONNECT_CONF_VAL);

	xlnx_hdmi_set_hdmi_mode(hdmi);
	xlnx_hdmi_aux_enable(hdmi);

	/* ddc init */
	clkrate = clk_get_rate(hdmitx_clks[S_AXI_CPU_ACLK].clk);
	val = div_u64(clkrate, HDMI_TX_DDC_CLKDIV * 2);
	val = (val << HDMI_TX_DDC_CTRL_CLK_DIV_SHIFT) &
		HDMI_TX_DDC_CTRL_CLK_DIV;

	/* Update DDC Control register */
	xlnx_hdmi_writel(hdmi, HDMI_TX_DDC_CTRL, val);

	xlnx_hdmi_frl_reset(hdmi);
	xlnx_hdmi_set_hdmi_mode(hdmi);
	xlnx_hdmi_aux_enable(hdmi);

	xlnx_hdmi_reset(hdmi);

	phy_cfg.hdmi.config_hdmi20 = 1;
	ret = xlnx_hdmi_phy_configure(hdmi, &phy_cfg);
	if (ret) {
		dev_err(hdmi->dev, "phy_cfg: hdmi20 err %d\n", ret);
		return ret;
	}

	/* Enable Interrupts */
	xlnx_hdmi_piointr_ie_enable(hdmi);
	xlnx_hdmi_piointr_run_enable(hdmi);

	return 0;
}

static int xlnx_hdmi_parse_of(struct xlnx_hdmi *hdmi)
{
	struct xlnx_hdmi_config *config = &hdmi->config;
	struct device_node *node = hdmi->dev->of_node;
	int ret;
	u32 ppc, bpc, vid, frl_rate;

	ret = of_property_read_u32(node, "xlnx,input-pixels-per-clock", &ppc);
	if (ret || (ppc != HDMI_TX_PPC_4 && ppc != HDMI_TX_PPC_8)) {
		dev_err(hdmi->dev, "missing or invalid pixels per clock dt prop\n");
		return -EINVAL;
	}
	config->ppc = ppc;

	ret = of_property_read_u32(node, "xlnx,max-bits-per-component", &bpc);
	if (ret || (bpc != HDMI_TX_BPC_8 && bpc != HDMI_TX_BPC_10 &&
		    bpc != HDMI_TX_BPC_12 && bpc != HDMI_TX_BPC_16)) {
		dev_err(hdmi->dev, "missing or invalid max bpc dt prop\n");
		return -EINVAL;
	}
	config->bpc = bpc;

	config->hdcp1x_enable = of_property_read_bool(node, "xlnx,include-hdcp-1-4");
	config->hdcp2x_enable = of_property_read_bool(node, "xlnx,include-hdcp-2-2");

	ret = of_property_read_u32(node, "xlnx,vid-interface", &vid);
	if (ret || (vid != HDMI_TX_AXI_STREAM && vid != HDMI_TX_NATIVE &&
		    vid != HDMI_TX_NATIVE_IDE)) {
		dev_err(hdmi->dev, "missing or unsupported video interface\n");
		return -EINVAL;
	}
	config->vid_interface = vid;

	ret = of_property_read_u32(node, "xlnx,max-frl-rate", &frl_rate);
	if (ret || frl_rate > HDMI_TX_MAX_FRL_RATE) {
		dev_err(hdmi->dev, "missing or unsupported frl rate\n");
		return -EINVAL;
	}
	config->max_frl_rate = frl_rate;

	if (of_device_is_compatible(node, "xlnx,v-hdmi-txss1-1.1")) {
		config->htiming_div_fact = config->ppc;
	} else {
		/* VTC core updated to support arbitrary resolutions */
		config->htiming_div_fact = 1;
		/* Remapper in subsystem will generate 4 ppc */
		config->ppc = HDMI_TX_PPC_4;
	}

	return 0;
}

/**
 * xlnx_hdmi_hdcp_exit - hdcp module de-initialization
 * @hdmi: HDMI IP core structure
 *
 * Return: 0 on success, or the status from called functions
 */
static int xlnx_hdmi_hdcp_exit(struct xlnx_hdmi *hdmi)
{
	struct xlnx_hdcptx *xhdcp = &hdmi->txhdcp;
	int ret;

	if (xhdcp->hdcp2xenable) {
		ret = xlnx_hdmi_hdcp_reset(hdmi);
		if (ret < 0) {
			dev_err(hdmi->dev, "failed to exit HDCP IP module");
			return ret;
		}
	}
	xlnx_hdcp_tx_timer_exit(xhdcp);
	xlnx_hdcp_tx_exit(xhdcp);

	return 0;
}

/**
 * xlnx_hdmi_hdcp_cp_irq_func - Checks for HDCP information
 * whenever CP Irq is detected
 * @work: work structure
 *
 * This function checks for HDCP authentication information via rxstatus register
 * as soon as cp irq interrupt triggers.
 */
static void xlnx_hdmi_hdcp_cp_irq_func(struct work_struct *work)
{
	struct xlnx_hdmi *hdmi;
	struct xlnx_hdcptx *xhdcp;

	hdmi = container_of(work, struct xlnx_hdmi, hdcp_cp_irq_work.work);
	xhdcp = &hdmi->txhdcp;
	xlnx_hdcp_tx_process_cp_irq(xhdcp);
}

/**
 * xlnx_hdmi_hdcp_status_update - hdcp status notification
 * @ref: callback reference pointer
 * @notification: hdcp notification
 */
static void xlnx_hdmi_hdcp_status_update(void *ref, u32 notification)
{
	struct xlnx_hdmi *hdmi = (struct xlnx_hdmi *)ref;

	switch (notification) {
	case XHDCPTX_INCOMPATIBLE_RX:
		dev_dbg(hdmi->dev, "HDCP Tx compatible receiver is not found\n");
		break;
	case XHDCPTX_AUTHENTICATION_BUSY:
		dev_dbg(hdmi->dev, "HDCP Tx Authentication Busy\n");
		break;
	case XHDCPTX_AUTHENTICATED:
		dev_dbg(hdmi->dev, "HDCP Tx Authenticated\n");
		break;
	case XHDCPTX_REAUTHENTICATE_REQUESTED:
		dev_dbg(hdmi->dev, "HDCP Tx Re-authentication Request received\n");
		break;
	case XHDCPTX_DEVICE_IS_REVOKED:
		dev_dbg(hdmi->dev, "HDCP Tx , a device in the hdcp chain is revoked\n");
		break;
	case XHDCPTX_NO_SRM_LOADED:
		dev_dbg(hdmi->dev, "HDCP Tx , no valid srm is loaded\n");
		break;
	case XHDCPTX_UNAUTHENTICATED:
		dev_dbg(hdmi->dev, "HDCP Tx Unauthenticated\n");
		break;
	default:
		dev_dbg(hdmi->dev, "Error, HDCP is not initialized\n");
		break;
	}
}

/**
 * xlnx_hdmi_hdcp_irq_handler - HDCP protocol message interrupt handler
 * @irq: IRQ number of the interrupt being handled
 * @data: Pointer to device structure
 *
 * Return: irq handler status
 */
static irqreturn_t xlnx_hdmi_hdcp_irq_handler(int irq, void *data)
{
	struct xlnx_hdmi *hdmi = (struct xlnx_hdmi *)data;
	struct xlnx_hdcptx *hdmitxhdcp = &hdmi->txhdcp;

	xlnx_hdcp1x_interrupt_handler(hdmitxhdcp);

	return IRQ_HANDLED;
}

/**
 * xlnx_hdmi_timer_irq_handler - hdcp timer interrupt handler
 * @irq: IRQ number of the interrupt being handled
 * @data: Pointer to device structure
 *
 * Return: irq handler status
 */
static irqreturn_t xlnx_hdmi_timer_irq_handler(int irq, void *data)
{
	struct xlnx_hdmi *hdmi = (struct xlnx_hdmi *)data;
	struct xlnx_hdcptx *xhdcp = &hdmi->txhdcp;

	xlnx_hdcp_tmrcntr_interrupt_handler(xhdcp->xhdcptmr);

	return IRQ_HANDLED;
}

static int xlnx_hdmi_hdcp_ddc_callback_write(void *ref, u32 offset,
					     void *buf, u32 buf_size)
{
	struct xlnx_hdmi *hdmi = (struct xlnx_hdmi *)ref;
	int ret;
	bool stop_flag;

	stop_flag = (buf_size > 1) ? true : false;

	ret = xlnx_hdmi_ddcwrite(hdmi, HDMI_HDCP_DDC_BASE_OFFSET, buf_size, (u8 *)buf, stop_flag);
	if (ret < 0) {
		dev_err(hdmi->dev, "DDC write failed");
		return ret;
	}
	return buf_size;
}

static int xlnx_hdmi_hdcp_ddc_callback_read(void *ref, u32 offset,
					    void *buf, u32 buf_size)
{
	struct xlnx_hdmi *hdmi = (struct xlnx_hdmi *)ref;
	int ret;

	if (!buf_size)
		return buf_size;
	ret = xlnx_hdmi_ddc_readreg(hdmi, HDMI_HDCP_DDC_BASE_OFFSET, buf_size, offset, buf);
	if (ret < 0) {
		dev_err(hdmi->dev, "DDC read failed");
		return ret;
	}
	return buf_size;
}

/**
 * xlnx_hdcp_init - hdcp module initialization
 * @hdmi: HDMI IP core structure
 * @pdev: platform structure
 *
 * Return: 0 on success, or return the error code from the called functions.
 */
static int xlnx_hdcp_init(struct xlnx_hdmi *hdmi,
			  struct platform_device *pdev)
{
	struct xlnx_hdcptx *xhdcp = &hdmi->txhdcp;
	int ret;

	xhdcp->dev = hdmi->dev;
	xhdcp->hdcp2xenable = hdmi->config.hdcp2x_enable;
	xhdcp->hdcp1xenable = hdmi->config.hdcp1x_enable;
	xhdcp->is_hdcp_initialized = false;

	if (hdmi->config.hdcp2x_enable) {
		xhdcp->xhdcp2x = xlnx_hdcp_tx_init(&pdev->dev, hdmi, xhdcp,
						   hdmi->base + HDMI_HDCP2X_OFFSET,
						   0, XHDCPTX_HDCP_2X,
						   hdmi->stream.sink_max_lanes,
						   XHDCP2X_TX_HDMI, hdmi->hdcp1x_keymgmt_base);

		if (IS_ERR(xhdcp->xhdcp2x)) {
			dev_err(hdmi->dev, "failed to initialize HDCP2X module\n");
			return PTR_ERR(xhdcp->xhdcp2x);
		}
		hdmi->hdcp2x_timer_irq =
				 platform_get_irq_byname(pdev, "hdcp22timer");
		if (hdmi->hdcp2x_timer_irq < 0) {
			dev_err(hdmi->dev, "failed to get HDCP2X timer irq");
			return -EINVAL;
		}
		ret = devm_request_threaded_irq(hdmi->dev, hdmi->hdcp2x_timer_irq, NULL,
						xlnx_hdmi_timer_irq_handler,
						IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
						"hdcp22timer", hdmi);
		if (ret < 0) {
			dev_err(hdmi->dev, "failed to register HDCP timer irq");
			return ret;
		}
		xhdcp->xhdcptmr =
				xlnx_hdcp_timer_init(&pdev->dev,
						     hdmi->base + HDMI_HDCP2X_OFFSET + 0x10000);
		if (IS_ERR(xhdcp->xhdcptmr)) {
			dev_err(hdmi->dev, "failed to initialize HDCP timer\n");
			return PTR_ERR(xhdcp->xhdcptmr);
		}
	}
	if (hdmi->config.hdcp1x_enable) {
		xhdcp->xhdcp1x = xlnx_hdcp_tx_init(&pdev->dev, hdmi, xhdcp,
						   hdmi->base + HDMI_HDCP1X_OFFSET,
						   0, XHDCPTX_HDCP_1X,
						   hdmi->stream.sink_max_lanes,
						   XHDCP2X_TX_HDMI,
						   hdmi->hdcp1x_keymgmt_base);

		if (IS_ERR(xhdcp->xhdcp1x)) {
			dev_err(hdmi->dev, "failed to initialize HDCP1X module\n");
			return PTR_ERR(xhdcp->xhdcp1x);
		}

		xhdcp->xhdcptmr =
			xlnx_hdcp_timer_init(&pdev->dev, hdmi->base + HDMI_HDCP_TIMER_OFFSET);
		if (IS_ERR(xhdcp->xhdcptmr)) {
			dev_err(hdmi->dev, "failed to initialize HDCP timer\n");
			return PTR_ERR(xhdcp->xhdcptmr);
		}

		hdmi->hdcp1x_timer_irq =
				 platform_get_irq_byname(pdev, "hdcp14timer");
		if (hdmi->hdcp1x_timer_irq < 0) {
			dev_err(hdmi->dev, "failed to get HDCP timer irq ");
			return -EINVAL;
		}

		ret = devm_request_threaded_irq(hdmi->dev, hdmi->hdcp1x_timer_irq, NULL,
						xlnx_hdmi_timer_irq_handler,
						IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
						"hdcp14timer", hdmi);
		if (ret < 0) {
			dev_err(hdmi->dev, "failed to register HDCP timer irq");
			return ret;
		}

		hdmi->hdcp_irq =
				 platform_get_irq_byname(pdev, "hdcp14");
		if (hdmi->hdcp_irq < 0) {
			dev_err(hdmi->dev, "failed to get HDCP irq ");
			return -EINVAL;
		}

		ret = devm_request_threaded_irq(hdmi->dev, hdmi->hdcp_irq, NULL,
						xlnx_hdmi_hdcp_irq_handler,
						IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
						"hdcp14", hdmi);
		if (ret < 0) {
			dev_err(hdmi->dev, "failed to register HDCP interrupt irq");
			return ret;
		}
	}
	xlnx_hdcp_tx_set_callback(xhdcp, HDMI_HDCP_DPCD_WRITE,
				  xlnx_hdmi_hdcp_ddc_callback_write);

	xlnx_hdcp_tx_set_callback(xhdcp, HDMI_HDCP_DPCD_READ,
				  xlnx_hdmi_hdcp_ddc_callback_read);

	xlnx_hdcp_tx_set_callback(xhdcp, HDMI_HDCP_STATUS,
				  xlnx_hdmi_hdcp_status_update);

	INIT_DELAYED_WORK(&hdmi->hdcp_cp_irq_work, xlnx_hdmi_hdcp_cp_irq_func);

	xhdcp->is_hdcp_initialized = true;

	return 0;
}

static int xlnx_hdmi_probe(struct platform_device *pdev)
{
	struct xlnx_hdmi *hdmi;
	struct resource *res;
	struct device_node *vpss_node;
	unsigned int index;
	int ret, num_clks = ARRAY_SIZE(hdmitx_clks);

	hdmi = devm_kzalloc(&pdev->dev, sizeof(*hdmi), GFP_KERNEL);
	if (!hdmi)
		return -ENOMEM;

	hdmi->dpms = DRM_MODE_DPMS_OFF;
	hdmi->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	hdmi->base = devm_ioremap_resource(hdmi->dev, res);
	if (IS_ERR(hdmi->base))
		return PTR_ERR(hdmi->base);

	ret = xlnx_hdmi_parse_of(hdmi);
	if (ret < 0)
		return ret;

	/* VPSS bridge support */
	vpss_node = of_parse_phandle(hdmi->dev->of_node, "xlnx,vpss", 0);
	if (vpss_node) {
		hdmi->bridge = of_xlnx_bridge_get(vpss_node);
		if (!hdmi->bridge) {
			dev_info(hdmi->dev, "Didn't get bridge instance\n");
			return -EPROBE_DEFER;
		}
	}

	ret = clk_bulk_get(&pdev->dev, num_clks, hdmitx_clks);
	if (ret)
		return ret;

	ret = clk_bulk_prepare_enable(num_clks, hdmitx_clks);
	if (ret)
		goto err_clk_put;

	/* acquire hdmi phy lanes */
	for (index = 0; index < HDMI_MAX_LANES; index++) {
		char phy_name[16];

		snprintf(phy_name, sizeof(phy_name), "hdmi-phy%d", index);
		hdmi->phy[index] = devm_phy_get(hdmi->dev, phy_name);
		if (IS_ERR(hdmi->phy[index])) {
			dev_err(hdmi->dev, "failed to get hdmi phy\n");
			ret = PTR_ERR(hdmi->phy[index]);
			goto err_clk_put;
		}
		ret = phy_init(hdmi->phy[index]);
		if (ret) {
			dev_err(hdmi->dev, "failed to init hdmi phy\n");
			goto error_phy;
		}
	}

	dev_dbg(hdmi->dev, "axi_cpu_aclk = %lu Hz\n",
		clk_get_rate(hdmitx_clks[S_AXI_CPU_ACLK].clk));
	dev_dbg(hdmi->dev, "link clk = %lu Hz\n",
		clk_get_rate(hdmitx_clks[LINK_CLK].clk));
	dev_dbg(hdmi->dev, "video clk = %lu Hz\n",
		clk_get_rate(hdmitx_clks[VIDEO_CLK].clk));
	dev_dbg(hdmi->dev, "frl clk = %lu Hz\n",
		clk_get_rate(hdmitx_clks[FRL_CLK].clk));
	dev_dbg(hdmi->dev, "video aclk rate = %lu Hz\n",
		clk_get_rate(hdmitx_clks[S_AXIS_VIDEO_ACLK].clk));

	hdmi->irq = platform_get_irq(pdev, 0);
	if (hdmi->irq < 0) {
		dev_err(hdmi->dev, "platform_get_irq() failed\n");
		ret = hdmi->irq;
		goto error_phy;
	}

	/* Request the interrupt */
	ret = devm_request_threaded_irq(hdmi->dev, hdmi->irq,
					hdmitx_irq_handler, hdmitx_irq_thread,
					IRQF_TRIGGER_HIGH,
					"xilinx-hdmitxss", hdmi/* dev_id */);
	if (ret) {
		dev_err(hdmi->dev, "unable to request IRQ %d\n", hdmi->irq);
		goto error_phy;
	}

	platform_set_drvdata(pdev, hdmi);

	/* initialize hw */
	ret = xlnx_hdmi_initialize(hdmi);
	if (ret) {
		dev_err(hdmi->dev, "hdmi initialization failed\n");
		goto error_phy;
	}
	if (hdmi->config.hdcp2x_enable || hdmi->config.hdcp1x_enable) {
		ret = sysfs_create_group(&hdmi->dev->kobj, &xlnx_hdcp_key_attr_group);
		if (ret) {
			dev_err(hdmi->dev, "\nunable to create sysfs group");
			goto error_phy;
		}
	}

	if (hdmi->config.hdcp1x_enable) {
		hdmi->hdcp1x_keymgmt_base =
			syscon_regmap_lookup_by_phandle(hdmi->dev->of_node,
							"xlnx,hdcp1x-keymgmt");
		if (IS_ERR(hdmi->hdcp1x_keymgmt_base)) {
			dev_err(hdmi->dev, "couldn't map HDCP1X Keymgmt registers\n");
			goto error_phy;
		}
	}

	if (hdmi->config.hdcp1x_enable || hdmi->config.hdcp2x_enable) {
		ret = xlnx_hdcp_init(hdmi, pdev);
		if (ret < 0)
			goto error_hdcp;
	}

	return component_add(hdmi->dev, &xlnx_hdmi_component_ops);

error_hdcp:
	xlnx_hdmi_hdcp_exit(hdmi);
	sysfs_remove_group(&pdev->dev.kobj, &xlnx_hdcp_key_attr_group);
error_phy:
	dev_dbg(hdmi->dev, "probe failed:: error_phy:\n");
	xlnx_hdmi_exit_phy(hdmi);
	clk_bulk_disable_unprepare(num_clks, hdmitx_clks);
err_clk_put:
	clk_bulk_put(num_clks, hdmitx_clks);

	return ret;
}

static void xlnx_hdmi_remove(struct platform_device *pdev)
{
	struct xlnx_hdmi *hdmi = platform_get_drvdata(pdev);
	int num_clks = ARRAY_SIZE(hdmitx_clks);

	if (hdmi->bridge)
		xlnx_bridge_disable(hdmi->bridge);

	xlnx_hdmi_exit_phy(hdmi);
	sysfs_remove_group(&pdev->dev.kobj, &xlnx_hdcp_key_attr_group);
	component_del(&pdev->dev, &xlnx_hdmi_component_ops);
	clk_bulk_disable_unprepare(num_clks, hdmitx_clks);
	clk_bulk_put(num_clks, hdmitx_clks);
}

static const struct of_device_id xlnx_hdmi_of_match[] = {
	{ .compatible = "xlnx,v-hdmi-txss1-1.1" },
	{ .compatible = "xlnx,v-hdmi-txss1-1.2" },
	{ /* end of table */ },
};

MODULE_DEVICE_TABLE(of, xlnx_hdmi_of_match);

static struct platform_driver xlnx_hdmi_driver = {
	.probe			= xlnx_hdmi_probe,
	.remove			= xlnx_hdmi_remove,
	.driver			= {
		.name		= "xlnx-hdmi",
		.of_match_table	= xlnx_hdmi_of_match,
	},
};

module_platform_driver(xlnx_hdmi_driver);

MODULE_AUTHOR("Venkateshwar Rao G <vgannava@xilinx.com>");
MODULE_DESCRIPTION("Xilinx DRM KMS HDMI Driver");
MODULE_LICENSE("GPL");
