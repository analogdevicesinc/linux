/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author: Chris Zhong <zyw@rock-chips.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef CDNS_MHDP_H_
#define CDNS_MHDP_H_

#include <drm/drm_bridge.h>
#include <drm/drm_connector.h>
#include <drm/display/drm_dp_helper.h>
#include <drm/display/drm_dp_mst_helper.h>
#include <media/cec.h>
#include <linux/bitops.h>
#include <sound/hdmi-codec.h>

#define ADDR_IMEM		0x10000
#define ADDR_DMEM		0x20000
#define ADDR_PHY_AFE	0x80000

/* APB CFG addr */
#define APB_CTRL			0
#define XT_INT_CTRL			0x04
#define MAILBOX_FULL_ADDR		0x08
#define MAILBOX_EMPTY_ADDR		0x0c
#define MAILBOX0_WR_DATA		0x10
#define MAILBOX0_RD_DATA		0x14
#define KEEP_ALIVE			0x18
#define VER_L				0x1c
#define VER_H				0x20
#define VER_LIB_L_ADDR			0x24
#define VER_LIB_H_ADDR			0x28
#define SW_DEBUG_L			0x2c
#define SW_DEBUG_H			0x30
#define MAILBOX_INT_MASK		0x34
#define MAILBOX_INT_STATUS		0x38
#define SW_CLK_L			0x3c
#define SW_CLK_H			0x40
#define SW_EVENTS0			0x44
#define SW_EVENTS1			0x48
#define SW_EVENTS2			0x4c
#define SW_EVENTS3			0x50
#define XT_OCD_CTRL			0x60
#define APB_INT_MASK			0x6c
#define APB_STATUS_MASK			0x70

/* audio decoder addr */
#define AUDIO_SRC_CNTL			0x30000
#define AUDIO_SRC_CNFG			0x30004
#define COM_CH_STTS_BITS		0x30008
#define STTS_BIT_CH(x)			(0x3000c + ((x) << 2))
#define SPDIF_CTRL_ADDR			0x3004c
#define SPDIF_CH1_CS_3100_ADDR		0x30050
#define SPDIF_CH1_CS_6332_ADDR		0x30054
#define SPDIF_CH1_CS_9564_ADDR		0x30058
#define SPDIF_CH1_CS_12796_ADDR		0x3005c
#define SPDIF_CH1_CS_159128_ADDR	0x30060
#define SPDIF_CH1_CS_191160_ADDR	0x30064
#define SPDIF_CH2_CS_3100_ADDR		0x30068
#define SPDIF_CH2_CS_6332_ADDR		0x3006c
#define SPDIF_CH2_CS_9564_ADDR		0x30070
#define SPDIF_CH2_CS_12796_ADDR		0x30074
#define SPDIF_CH2_CS_159128_ADDR	0x30078
#define SPDIF_CH2_CS_191160_ADDR	0x3007c
#define SMPL2PKT_CNTL			0x30080
#define SMPL2PKT_CNFG			0x30084
#define FIFO_CNTL			0x30088
#define FIFO_STTS			0x3008c

/* source pif addr */
#define SOURCE_PIF_WR_ADDR		0x30800
#define SOURCE_PIF_WR_REQ		0x30804
#define SOURCE_PIF_RD_ADDR		0x30808
#define SOURCE_PIF_RD_REQ		0x3080c
#define SOURCE_PIF_DATA_WR		0x30810
#define SOURCE_PIF_DATA_RD		0x30814
#define SOURCE_PIF_FIFO1_FLUSH		0x30818
#define SOURCE_PIF_FIFO2_FLUSH		0x3081c
#define SOURCE_PIF_STATUS		0x30820
#define SOURCE_PIF_INTERRUPT_SOURCE	0x30824
#define SOURCE_PIF_INTERRUPT_MASK	0x30828
#define SOURCE_PIF_PKT_ALLOC_REG	0x3082c
#define SOURCE_PIF_PKT_ALLOC_WR_EN	0x30830
#define SOURCE_PIF_SW_RESET		0x30834

/* bellow registers need access by mailbox */
/* source phy comp */
#define PHY_DATA_SEL			0x0818
#define LANES_CONFIG			0x0814

/* source car addr */
#define SOURCE_HDTX_CAR			0x0900
#define SOURCE_DPTX_CAR			0x0904
#define SOURCE_PHY_CAR			0x0908
#define SOURCE_CEC_CAR			0x090c
#define SOURCE_CBUS_CAR			0x0910
#define SOURCE_PKT_CAR			0x0918
#define SOURCE_AIF_CAR			0x091c
#define SOURCE_CIPHER_CAR		0x0920
#define SOURCE_CRYPTO_CAR		0x0924

/* mhdp tx_top_comp */
#define SCHEDULER_H_SIZE		0x1000
#define SCHEDULER_V_SIZE		0x1004
#define HDTX_SIGNAL_FRONT_WIDTH	0x100c
#define HDTX_SIGNAL_SYNC_WIDTH	0x1010
#define HDTX_SIGNAL_BACK_WIDTH	0x1014
#define HDTX_CONTROLLER			0x1018
#define HDTX_HPD				0x1020
#define HDTX_CLOCK_REG_0		0x1024
#define HDTX_CLOCK_REG_1		0x1028

/* clock meters addr */
#define CM_CTRL				0x0a00
#define CM_I2S_CTRL			0x0a04
#define CM_SPDIF_CTRL			0x0a08
#define CM_VID_CTRL			0x0a0c
#define CM_LANE_CTRL			0x0a10
#define I2S_NM_STABLE			0x0a14
#define I2S_NCTS_STABLE			0x0a18
#define SPDIF_NM_STABLE			0x0a1c
#define SPDIF_NCTS_STABLE		0x0a20
#define NMVID_MEAS_STABLE		0x0a24
#define I2S_MEAS			0x0a40
#define SPDIF_MEAS			0x0a80
#define NMVID_MEAS			0x0ac0

/* source vif addr */
#define BND_HSYNC2VSYNC			0x0b00
#define HSYNC2VSYNC_F1_L1		0x0b04
#define HSYNC2VSYNC_F2_L1		0x0b08
#define HSYNC2VSYNC_STATUS		0x0b0c
#define HSYNC2VSYNC_POL_CTRL		0x0b10

/* dptx phy addr */
#define DP_TX_PHY_CONFIG_REG		0x2000
#define DP_TX_PHY_SW_RESET		0x2004
#define DP_TX_PHY_SCRAMBLER_SEED	0x2008
#define DP_TX_PHY_TRAINING_01_04	0x200c
#define DP_TX_PHY_TRAINING_05_08	0x2010
#define DP_TX_PHY_TRAINING_09_10	0x2014
#define TEST_COR			0x23fc

/* dptx hpd addr */
#define HPD_IRQ_DET_MIN_TIMER		0x2100
#define HPD_IRQ_DET_MAX_TIMER		0x2104
#define HPD_UNPLGED_DET_MIN_TIMER	0x2108
#define HPD_STABLE_TIMER		0x210c
#define HPD_FILTER_TIMER		0x2110
#define HPD_EVENT_MASK			0x211c
#define HPD_EVENT_DET			0x2120

/* dpyx framer addr */
#define DP_FRAMER_GLOBAL_CONFIG		0x2200
#define DP_SW_RESET			0x2204
#define DP_FRAMER_TU			0x2208
#define DP_FRAMER_PXL_REPR		0x220c
#define DP_FRAMER_SP			0x2210
#define AUDIO_PACK_CONTROL		0x2214
#define DP_VC_TABLE(x)			(0x2218 + ((x) << 2))
#define DP_VB_ID			0x2258
#define DP_MTPH_LVP_CONTROL		0x225c
#define DP_MTPH_SYMBOL_VALUES		0x2260
#define DP_MTPH_ECF_CONTROL		0x2264
#define DP_MTPH_ACT_CONTROL		0x2268
#define DP_MTPH_STATUS			0x226c
#define DP_INTERRUPT_SOURCE		0x2270
#define DP_INTERRUPT_MASK		0x2274
#define DP_FRONT_BACK_PORCH		0x2278
#define DP_BYTE_COUNT			0x227c

/* dptx stream addr */
#define MSA_HORIZONTAL_0		0x2280
#define MSA_HORIZONTAL_1		0x2284
#define MSA_VERTICAL_0			0x2288
#define MSA_VERTICAL_1			0x228c
#define MSA_MISC			0x2290
#define STREAM_CONFIG			0x2294
#define AUDIO_PACK_STATUS		0x2298
#define VIF_STATUS			0x229c
#define PCK_STUFF_STATUS_0		0x22a0
#define PCK_STUFF_STATUS_1		0x22a4
#define INFO_PACK_STATUS		0x22a8
#define RATE_GOVERNOR_STATUS		0x22ac
#define DP_HORIZONTAL			0x22b0
#define DP_VERTICAL_0			0x22b4
#define DP_VERTICAL_1			0x22b8
#define DP_BLOCK_SDP			0x22bc

/* dptx glbl addr */
#define DPTX_LANE_EN			0x2300
#define DPTX_ENHNCD			0x2304
#define DPTX_INT_MASK			0x2308
#define DPTX_INT_STATUS			0x230c

/* dp aux addr */
#define DP_AUX_HOST_CONTROL		0x2800
#define DP_AUX_INTERRUPT_SOURCE		0x2804
#define DP_AUX_INTERRUPT_MASK		0x2808
#define DP_AUX_SWAP_INVERSION_CONTROL	0x280c
#define DP_AUX_SEND_NACK_TRANSACTION	0x2810
#define DP_AUX_CLEAR_RX			0x2814
#define DP_AUX_CLEAR_TX			0x2818
#define DP_AUX_TIMER_STOP		0x281c
#define DP_AUX_TIMER_CLEAR		0x2820
#define DP_AUX_RESET_SW			0x2824
#define DP_AUX_DIVIDE_2M		0x2828
#define DP_AUX_TX_PREACHARGE_LENGTH	0x282c
#define DP_AUX_FREQUENCY_1M_MAX		0x2830
#define DP_AUX_FREQUENCY_1M_MIN		0x2834
#define DP_AUX_RX_PRE_MIN		0x2838
#define DP_AUX_RX_PRE_MAX		0x283c
#define DP_AUX_TIMER_PRESET		0x2840
#define DP_AUX_NACK_FORMAT		0x2844
#define DP_AUX_TX_DATA			0x2848
#define DP_AUX_RX_DATA			0x284c
#define DP_AUX_TX_STATUS		0x2850
#define DP_AUX_RX_STATUS		0x2854
#define DP_AUX_RX_CYCLE_COUNTER		0x2858
#define DP_AUX_MAIN_STATES		0x285c
#define DP_AUX_MAIN_TIMER		0x2860
#define DP_AUX_AFE_OUT			0x2864

/* crypto addr */
#define CRYPTO_HDCP_REVISION		0x5800
#define HDCP_CRYPTO_CONFIG		0x5804
#define CRYPTO_INTERRUPT_SOURCE		0x5808
#define CRYPTO_INTERRUPT_MASK		0x580c
#define CRYPTO22_CONFIG			0x5818
#define CRYPTO22_STATUS			0x581c
#define SHA_256_DATA_IN			0x583c
#define SHA_256_DATA_OUT_(x)		(0x5850 + ((x) << 2))
#define AES_32_KEY_(x)			(0x5870 + ((x) << 2))
#define AES_32_DATA_IN			0x5880
#define AES_32_DATA_OUT_(x)		(0x5884 + ((x) << 2))
#define CRYPTO14_CONFIG			0x58a0
#define CRYPTO14_STATUS			0x58a4
#define CRYPTO14_PRNM_OUT		0x58a8
#define CRYPTO14_KM_0			0x58ac
#define CRYPTO14_KM_1			0x58b0
#define CRYPTO14_AN_0			0x58b4
#define CRYPTO14_AN_1			0x58b8
#define CRYPTO14_YOUR_KSV_0		0x58bc
#define CRYPTO14_YOUR_KSV_1		0x58c0
#define CRYPTO14_MI_0			0x58c4
#define CRYPTO14_MI_1			0x58c8
#define CRYPTO14_TI_0			0x58cc
#define CRYPTO14_KI_0			0x58d0
#define CRYPTO14_KI_1			0x58d4
#define CRYPTO14_BLOCKS_NUM		0x58d8
#define CRYPTO14_KEY_MEM_DATA_0		0x58dc
#define CRYPTO14_KEY_MEM_DATA_1		0x58e0
#define CRYPTO14_SHA1_MSG_DATA		0x58e4
#define CRYPTO14_SHA1_V_VALUE_(x)	(0x58e8 + ((x) << 2))
#define TRNG_CTRL			0x58fc
#define TRNG_DATA_RDY			0x5900
#define TRNG_DATA			0x5904

/* cipher addr */
#define HDCP_REVISION			0x60000
#define INTERRUPT_SOURCE		0x60004
#define INTERRUPT_MASK			0x60008
#define HDCP_CIPHER_CONFIG		0x6000c
#define AES_128_KEY_0			0x60010
#define AES_128_KEY_1			0x60014
#define AES_128_KEY_2			0x60018
#define AES_128_KEY_3			0x6001c
#define AES_128_RANDOM_0		0x60020
#define AES_128_RANDOM_1		0x60024
#define CIPHER14_KM_0			0x60028
#define CIPHER14_KM_1			0x6002c
#define CIPHER14_STATUS			0x60030
#define CIPHER14_RI_PJ_STATUS		0x60034
#define CIPHER_MODE			0x60038
#define CIPHER14_AN_0			0x6003c
#define CIPHER14_AN_1			0x60040
#define CIPHER22_AUTH			0x60044
#define CIPHER14_R0_DP_STATUS		0x60048
#define CIPHER14_BOOTSTRAP		0x6004c

#define DPTX_FRMR_DATA_CLK_RSTN_EN	BIT(11)
#define DPTX_FRMR_DATA_CLK_EN		BIT(10)
#define DPTX_PHY_DATA_RSTN_EN		BIT(9)
#define DPTX_PHY_DATA_CLK_EN		BIT(8)
#define DPTX_PHY_CHAR_RSTN_EN		BIT(7)
#define DPTX_PHY_CHAR_CLK_EN		BIT(6)
#define SOURCE_AUX_SYS_CLK_RSTN_EN	BIT(5)
#define SOURCE_AUX_SYS_CLK_EN		BIT(4)
#define DPTX_SYS_CLK_RSTN_EN		BIT(3)
#define DPTX_SYS_CLK_EN			BIT(2)
#define CFG_DPTX_VIF_CLK_RSTN_EN	BIT(1)
#define CFG_DPTX_VIF_CLK_EN		BIT(0)

#define SOURCE_PHY_RSTN_EN		BIT(1)
#define SOURCE_PHY_CLK_EN		BIT(0)

#define SOURCE_PKT_SYS_RSTN_EN		BIT(3)
#define SOURCE_PKT_SYS_CLK_EN		BIT(2)
#define SOURCE_PKT_DATA_RSTN_EN		BIT(1)
#define SOURCE_PKT_DATA_CLK_EN		BIT(0)

#define SPDIF_CDR_CLK_RSTN_EN		BIT(5)
#define SPDIF_CDR_CLK_EN		BIT(4)
#define SOURCE_AIF_SYS_RSTN_EN		BIT(3)
#define SOURCE_AIF_SYS_CLK_EN		BIT(2)
#define SOURCE_AIF_CLK_RSTN_EN		BIT(1)
#define SOURCE_AIF_CLK_EN		BIT(0)

#define SOURCE_CIPHER_SYSTEM_CLK_RSTN_EN	BIT(3)
#define SOURCE_CIPHER_SYS_CLK_EN		BIT(2)
#define SOURCE_CIPHER_CHAR_CLK_RSTN_EN		BIT(1)
#define SOURCE_CIPHER_CHAR_CLK_EN		BIT(0)

#define SOURCE_CRYPTO_SYS_CLK_RSTN_EN	BIT(1)
#define SOURCE_CRYPTO_SYS_CLK_EN	BIT(0)

#define APB_IRAM_PATH			BIT(2)
#define APB_DRAM_PATH			BIT(1)
#define APB_XT_RESET			BIT(0)

#define MAILBOX_INT_MASK_BIT		BIT(1)
#define PIF_INT_MASK_BIT		BIT(0)
#define ALL_INT_MASK			3

/* mailbox */
#define MB_OPCODE_ID			0
#define MB_MODULE_ID			1
#define MB_SIZE_MSB_ID			2
#define MB_SIZE_LSB_ID			3
#define MB_DATA_ID			4

#define MB_MODULE_ID_DP_TX			0x01
#define MB_MODULE_ID_HDMI_TX		0x03
#define MB_MODULE_ID_HDCP_TX		0x07
#define MB_MODULE_ID_HDCP_RX		0x08
#define MB_MODULE_ID_HDCP_GENERAL	0x09
#define MB_MODULE_ID_GENERAL		0x0A

/* general opcode */
#define GENERAL_MAIN_CONTROL            0x01
#define GENERAL_TEST_ECHO               0x02
#define GENERAL_BUS_SETTINGS            0x03
#define GENERAL_TEST_ACCESS             0x04
#define GENERAL_WRITE_REGISTER          0x05
#define GENERAL_WRITE_FIELD             0x06
#define GENERAL_READ_REGISTER           0x07
#define GENERAL_GET_HPD_STATE           0x11
#define GENERAL_ASSERT_PHY_BUS_RESET    0x12
#define GENERAL_DEASSERT_PHY_BUS_RESET  0x13
#define GENERAL_LOAD_HDCP_RX            0x20
#define GENERAL_UNLOAD_HDCP_RX          0x21

/* DPTX opcode */
#define DPTX_SET_POWER_MNG			0x00
#define DPTX_SET_HOST_CAPABILITIES		0x01
#define DPTX_GET_EDID				0x02
#define DPTX_READ_DPCD				0x03
#define DPTX_WRITE_DPCD				0x04
#define DPTX_ENABLE_EVENT			0x05
#define DPTX_WRITE_REGISTER			0x06
#define DPTX_READ_REGISTER			0x07
#define DPTX_WRITE_FIELD			0x08
#define DPTX_TRAINING_CONTROL			0x09
#define DPTX_READ_EVENT				0x0a
#define DPTX_READ_LINK_STAT			0x0b
#define DPTX_SET_VIDEO				0x0c
#define DPTX_SET_AUDIO				0x0d
#define DPTX_GET_LAST_AUX_STAUS			0x0e
#define DPTX_SET_LINK_BREAK_POINT		0x0f
#define DPTX_FORCE_LANES			0x10
#define DPTX_HPD_STATE				0x11
#define DPTX_ADJUST_LT				0x12
#define DPTX_I2C_READ              0x15
#define DPTX_I2C_WRITE             0x16
#define DPTX_GET_LAST_I2C_STATUS   0x17



/* HDMI TX opcode */
#define HDMI_TX_READ				0x00
#define HDMI_TX_WRITE				0x01
#define HDMI_TX_UPDATE_READ			0x02
#define HDMI_TX_EDID				0x03
#define HDMI_TX_EVENTS				0x04
#define HDMI_TX_HPD_STATUS			0x05
#define HDMI_TX_DEBUG_ECHO			0xAA
#define HDMI_TX_TEST				0xBB
#define HDMI_TX_EDID_INTERNAL		0xF0

/* HDCP General opcode */
#define HDCP_GENERAL_SET_LC_128		0x00
#define HDCP_GENERAL_SET_SEED		0x01

/* HDCP TX opcode */
#define HDCP_TX_CONFIGURATION				0x00
#define HDCP2_TX_SET_PUBLIC_KEY_PARAMS		0x01
#define HDCP2_TX_SET_DEBUG_RANDOM_NUMBERS	0x02
#define HDCP2_TX_RESPOND_KM					0x03
#define HDCP1_TX_SEND_KEYS					0x04
#define HDCP1_TX_SEND_RANDOM_AN				0x05
#define HDCP_TX_STATUS_CHANGE				0x06
#define HDCP2_TX_IS_KM_STORED				0x07
#define HDCP2_TX_STORE_KM					0x08
#define HDCP_TX_IS_RECEIVER_ID_VALID		0x09
#define HDCP_TX_RESPOND_RECEIVER_ID_VALID	0x0A
#define HDCP_TX_TEST_KEYS					0x0B
#define HDCP2_TX_SET_KM_KEY_PARAMS			0x0C
#define HDCP_TX_SET_CP_IRQ					0x0D
#define HDCP_TX_DO_AUTH_REQ					0x0E

#define FW_STANDBY				0
#define FW_ACTIVE				1

#define MHDP_EVENT_ENABLE_HPD			BIT(0)
#define MHDP_EVENT_ENABLE_TRAINING		BIT(1)

#define LINK_TRAINING_NOT_ACTIVE		0
#define LINK_TRAINING_RUN			1
#define LINK_TRAINING_RESTART			2

#define CONTROL_VIDEO_IDLE			0
#define CONTROL_VIDEO_VALID			1

#define TU_CNT_RST_EN				BIT(15)
#define VIF_BYPASS_INTERLACE			BIT(13)
#define INTERLACE_FMT_DET			BIT(12)
#define INTERLACE_DTCT_WIN			0x20

#define DP_FRAMER_SP_INTERLACE_EN		BIT(2)
#define DP_FRAMER_SP_HSP			BIT(1)
#define DP_FRAMER_SP_VSP			BIT(0)

/* capability */
#define AUX_HOST_INVERT				3
#define	FAST_LT_SUPPORT				1
#define FAST_LT_NOT_SUPPORT			0
#define LANE_MAPPING_NORMAL			0x1b
#define LANE_MAPPING_FLIPPED			0xe4
#define ENHANCED				1
#define SCRAMBLER_EN				BIT(4)

#define	FULL_LT_STARTED				BIT(0)
#define FASE_LT_STARTED				BIT(1)
#define CLK_RECOVERY_FINISHED			BIT(2)
#define EQ_PHASE_FINISHED			BIT(3)
#define FASE_LT_START_FINISHED			BIT(4)
#define CLK_RECOVERY_FAILED			BIT(5)
#define EQ_PHASE_FAILED				BIT(6)
#define FASE_LT_FAILED				BIT(7)

#define DPTX_HPD_EVENT						   BIT(0)
#define HDMI_TX_HPD_EVENT					   BIT(0)
#define HDMI_RX_5V_EVENT					   BIT(0)
#define DPTX_TRAINING_EVENT					   BIT(1)
#define HDMI_RX_SCDC_CHANGE_EVENT			   BIT(1)
#define HDCPTX_STATUS_EVENT					   BIT(4)
#define HDCPRX_STATUS_EVENT					   BIT(4)
#define HDCPTX_IS_KM_STORED_EVENT			   BIT(5)
#define HDCPTX_STORE_KM_EVENT				   BIT(6)
#define HDCPTX_IS_RECEIVER_ID_VALID_EVENT	   BIT(7)

#define TU_SIZE					30
#define CDNS_DP_MAX_LINK_RATE	540000

#define F_HDMI_ENCODING(x) (((x) & ((1 << 2) - 1)) << 16)
#define F_VIF_DATA_WIDTH(x) (((x) & ((1 << 2) - 1)) << 2)
#define F_HDMI_MODE(x) (((x) & ((1 << 2) - 1)) << 0)
#define F_GCP_EN(x) (((x) & ((1 << 1) - 1)) << 12)
#define F_CLEAR_AVMUTE(x) (((x) & ((1 << 1) - 1)) << 14)
#define F_DATA_EN(x) (((x) & ((1 << 1) - 1)) << 15)
#define F_HDMI2_PREAMBLE_EN(x) (((x) & ((1 << 1) - 1)) << 18)
#define F_PIC_3D(x) (((x) & ((1 << 4) - 1)) << 7)
#define F_BCH_EN(x) (((x) & ((1 << 1) - 1)) << 11)
#define F_SOURCE_PHY_MHDP_SEL(x) (((x) & ((1 << 2) - 1)) << 3)
#define F_HPD_VALID_WIDTH(x) (((x) & ((1 << 12) - 1)) << 0)
#define F_HPD_GLITCH_WIDTH(x) (((x) & ((1 << 8) - 1)) << 12)
#define F_HDMI2_CTRL_IL_MODE(x) (((x) & ((1 << 1) - 1)) << 19)
#define F_SOURCE_PHY_LANE0_SWAP(x) (((x) & ((1 << 2) - 1)) << 0)
#define F_SOURCE_PHY_LANE1_SWAP(x) (((x) & ((1 << 2) - 1)) << 2)
#define F_SOURCE_PHY_LANE2_SWAP(x) (((x) & ((1 << 2) - 1)) << 4)
#define F_SOURCE_PHY_LANE3_SWAP(x) (((x) & ((1 << 2) - 1)) << 6)
#define F_SOURCE_PHY_COMB_BYPASS(x) (((x) & ((1 << 1) - 1)) << 21)
#define F_SOURCE_PHY_20_10(x) (((x) & ((1 << 1) - 1)) << 22)
#define F_PKT_ALLOC_ADDRESS(x) (((x) & ((1 << 4) - 1)) << 0)
#define F_ACTIVE_IDLE_TYPE(x) (((x) & ((1 << 1) - 1)) << 17)
#define F_FIFO1_FLUSH(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_PKT_ALLOC_WR_EN(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_DATA_WR(x) (x)
#define F_WR_ADDR(x) (((x) & ((1 << 4) - 1)) << 0)
#define F_HOST_WR(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_TYPE_VALID(x) (((x) & ((1 << 1) - 1)) << 16)
#define F_PACKET_TYPE(x) (((x) & ((1 << 8) - 1)) << 8)

/* audio */
#define AUDIO_PACK_EN				BIT(8)
#define SAMPLING_FREQ(x)			(((x) & 0xf) << 16)
#define ORIGINAL_SAMP_FREQ(x)			(((x) & 0xf) << 24)
#define SYNC_WR_TO_CH_ZERO			BIT(1)
#define I2S_DEC_START				BIT(1)
#define AUDIO_SW_RST				BIT(0)
#define SMPL2PKT_EN				BIT(1)
#define MAX_NUM_CH(x)				(((x) & 0x1f) - 1)
#define NUM_OF_I2S_PORTS(x)			((((x) / 2 - 1) & 0x3) << 5)
#define AUDIO_TYPE_LPCM				(2 << 7)
#define CFG_SUB_PCKT_NUM(x)			((((x) - 1) & 0x7) << 11)
#define AUDIO_CH_NUM(x)				((((x) - 1) & 0x1f) << 2)
#define TRANS_SMPL_WIDTH_16			0
#define TRANS_SMPL_WIDTH_24			BIT(11)
#define TRANS_SMPL_WIDTH_32			(2 << 11)
#define I2S_DEC_PORT_EN(x)			(((x) & 0xf) << 17)
#define SPDIF_ENABLE				BIT(21)
#define SPDIF_AVG_SEL				BIT(20)
#define SPDIF_JITTER_BYPASS			BIT(19)
#define SPDIF_FIFO_MID_RANGE(x)			(((x) & 0xff) << 11)
#define SPDIF_JITTER_THRSH(x)			(((x) & 0xff) << 3)
#define SPDIF_JITTER_AVG_WIN(x)			((x) & 0x7)

/* Reference cycles when using lane clock as reference */
#define LANE_REF_CYC				0x8000

#define HOTPLUG_DEBOUNCE_MS		50

#define IRQ_IN    0
#define IRQ_OUT   1
#define IRQ_NUM   2

#define cdns_mhdp_plat_call(mhdp, operation)			\
	(!(mhdp) ? -ENODEV : (((mhdp)->plat_data && (mhdp)->plat_data->operation) ?	\
	 (mhdp)->plat_data->operation(mhdp) : ENOIOCTLCMD))

/* bus access type */
enum {
	BUS_TYPE_NORMAL_APB = 0,
	BUS_TYPE_NORMAL_SAPB = 1,
	BUS_TYPE_LOW4K_APB = 2,
	BUS_TYPE_LOW4K_SAPB = 3,
	BUS_TYPE_LOW4K_HDMI_RX = 4,
};

enum voltage_swing_level {
	VOLTAGE_LEVEL_0,
	VOLTAGE_LEVEL_1,
	VOLTAGE_LEVEL_2,
	VOLTAGE_LEVEL_3,
};

enum pre_emphasis_level {
	PRE_EMPHASIS_LEVEL_0,
	PRE_EMPHASIS_LEVEL_1,
	PRE_EMPHASIS_LEVEL_2,
	PRE_EMPHASIS_LEVEL_3,
};

enum pattern_set {
	PTS1		= BIT(0),
	PTS2		= BIT(1),
	PTS3		= BIT(2),
	PTS4		= BIT(3),
	DP_NONE		= BIT(4)
};

enum vic_color_depth {
	BCS_6 = 0x1,
	BCS_8 = 0x2,
	BCS_10 = 0x4,
	BCS_12 = 0x8,
	BCS_16 = 0x10,
};

enum vic_bt_type {
	BT_601 = 0x0,
	BT_709 = 0x1,
};

enum audio_format {
	AFMT_I2S = 0,
	AFMT_SPDIF_INT = 1,
	AFMT_SPDIF_EXT = 2,
	AFMT_UNUSED,
};

enum {
	MODE_DVI,
	MODE_HDMI_1_4,
	MODE_HDMI_2_0,
};

struct audio_info {
	enum audio_format format;
	int sample_rate;
	int channels;
	int sample_width;
	int connector_type;
	bool non_pcm;
};

enum vic_pxl_encoding_format {
	PXL_RGB = 0x1,
	YCBCR_4_4_4 = 0x2,
	YCBCR_4_2_2 = 0x4,
	YCBCR_4_2_0 = 0x8,
	Y_ONLY = 0x10,
};

enum link_training_type {
	DP_TX_FULL_LINK_TRAINING,
	DP_TX_FAST_LINK_TRAINING,
	DP_TX_NO_AUX_LINK_TRAINING
};

struct video_info {
	bool h_sync_polarity;
	bool v_sync_polarity;
	bool interlaced;
	int color_depth;
	enum vic_pxl_encoding_format color_fmt;
};

struct cdns_mhdp_host {
	unsigned int	link_rate;
	u8	lanes_cnt;
	u8	volt_swing;
	u8	pre_emphasis;
	u8	pattern_supp;
	u8	fast_link;
	u8	lane_mapping;
	u8	enhanced;
};

struct cdns_mhdp_sink {
	unsigned int	link_rate;
	u8	lanes_cnt;
	u8	pattern_supp;
	u8	fast_link;
	u8	enhanced;
};

struct cdns_mhdp_bridge;
struct cdns_mhdp_connector;

struct cdns_mhdp_bridge {
	struct cdns_mhdp_device *mhdp;
	struct drm_bridge base;
	int pbn;
	int8_t stream_id;
	struct cdns_mhdp_connector *connector;
	bool is_active;
};

struct cdns_mhdp_connector {
	struct drm_connector base;
	struct drm_connector_state new_state;
	bool is_mst_connector;
	struct drm_dp_mst_port *port;
	struct cdns_mhdp_bridge *bridge;
};

struct cdns_mhdp_cec {
	struct cec_adapter *adap;
	struct device *dev;
	struct mutex *iolock;
	void __iomem		*regs_base;
	void __iomem		*regs_sec;
	int bus_type;

	struct cec_msg msg;
	struct task_struct *cec_worker;
};

struct cdns_plat_data {
	/* Vendor PHY support */
	int (*bind)(struct platform_device *pdev,
			struct drm_encoder *encoder,
			struct cdns_mhdp_device *mhdp);
	void (*unbind)(struct device *dev);

	void (*plat_init)(struct cdns_mhdp_device *mhdp);
	void (*plat_deinit)(struct cdns_mhdp_device *mhdp);

	int (*phy_set)(struct cdns_mhdp_device *mhdp);
	bool (*phy_video_valid)(struct cdns_mhdp_device *mhdp);
	int (*firmware_init)(struct cdns_mhdp_device *mhdp);
	void (*pclk_rate)(struct cdns_mhdp_device *mhdp);

	int (*suspend)(struct cdns_mhdp_device *mhdp);
	int (*resume)(struct cdns_mhdp_device *mhdp);

	int (*power_on)(struct cdns_mhdp_device *mhdp);
	int (*power_off)(struct cdns_mhdp_device *mhdp);

	int bus_type;
	int video_format;
	char is_dp;
	char *plat_name;
};

/* HDCP */
#define MAX_STORED_KM 64
#define HDCP_PAIRING_M_LEN 16
#define HDCP_PAIRING_M_EKH 16
#define HDCP_PAIRING_R_ID 5

/* HDCP2_TX_SET_DEBUG_RANDOM_NUMBERS */
#define DEBUG_RANDOM_NUMBERS_KM_LEN 16
#define DEBUG_RANDOM_NUMBERS_RN_LEN 8
#define DEBUG_RANDOM_NUMBERS_KS_LEN 16
#define DEBUG_RANDOM_NUMBERS_RIV_LEN 8
#define DEBUG_RANDOM_NUMBERS_RTX_LEN 8

struct hdcp_trans_pairing_data {
	u8 receiver_id[HDCP_PAIRING_R_ID];
	u8 m[HDCP_PAIRING_M_LEN];
	u8 km[DEBUG_RANDOM_NUMBERS_KM_LEN];
	u8 ekh[HDCP_PAIRING_M_EKH];
};

enum hdmi_hdcp_state {
	HDCP_STATE_NO_AKSV,
	HDCP_STATE_INACTIVE,
	HDCP_STATE_ENABLING,
	HDCP_STATE_AUTHENTICATING,
	HDCP_STATE_REAUTHENTICATING,
	HDCP_STATE_AUTHENTICATED,
	HDCP_STATE_DISABLING,
	HDCP_STATE_AUTH_FAILED
};

struct cdns_mhdp_hdcp {
	struct mutex mutex;
	u64 value; /* protected by hdcp_mutex */
	struct delayed_work check_work;
	struct work_struct prop_work;
	u8 state;
	u8 cancel;
	u8 bus_type;
	u8 config;
	struct hdcp_trans_pairing_data pairing[MAX_STORED_KM];
	u8 num_paired;

	u8 events;
	u8 sink_is_repeater;
	u8 reauth_in_progress;
	u8 hdcp_version;
};

struct cdns_mhdp_device {
	void __iomem		*regs_base;
	void __iomem		*regs_sec;

	int bus_type;

	struct device		*dev;
	struct drm_device *drm_dev;

	struct cdns_mhdp_connector  connector;
	struct clk		*spdif_clk;
	struct reset_control	*spdif_rst;

	struct platform_device	*audio_pdev;
	struct audio_info	audio_info;

	struct cdns_mhdp_bridge	bridge;
	struct phy		*phy;

	struct video_info	video_info;
	struct drm_display_mode	mode;
	const struct drm_display_mode	*valid_mode;
	unsigned int		fw_version;

	struct drm_dp_mst_topology_mgr mst_mgr;
	struct delayed_work hotplug_work;

	u32 lane_mapping;
	bool link_up;
	bool force_disconnected_sts;
	bool power_up;
	bool plugged;
	bool force_mode_set;
	bool is_hpd;
	bool is_dp;
	bool is_ls1028a;
	struct mutex lock;
	struct mutex api_lock;
	struct mutex iolock;

	int irq[IRQ_NUM];

	union {
		struct _dp_data {
			u8 dpcd[DP_RECEIVER_CAP_SIZE];
			u32 rate;
			u8 num_lanes;
			u8 vswing[4];
			u8 preemphasis[4];
			u8 force_vswing;
			u8 force_preemphasis;
			enum link_training_type link_training_type;
			struct drm_dp_aux	aux;
			struct cdns_mhdp_host	host;
			struct cdns_mhdp_sink	sink;
			bool is_mst;
			bool can_mst;
		} dp;
		struct _hdmi_data {
			struct cdns_mhdp_cec cec;
			u32 char_rate;
			u32 hdmi_type;
		} hdmi;
	};
	const struct cdns_plat_data *plat_data;

	hdmi_codec_plugged_cb plugged_cb;
	struct device *codec_dev;
	enum drm_connector_status last_connector_result;
	struct cdns_mhdp_hdcp hdcp;
};

u32 cdns_mhdp_bus_read(struct cdns_mhdp_device *mhdp, u32 offset);
void cdns_mhdp_bus_write(u32 val, struct cdns_mhdp_device *mhdp, u32 offset);
void cdns_mhdp_clock_reset(struct cdns_mhdp_device *mhdp);
void cdns_mhdp_set_fw_clk(struct cdns_mhdp_device *mhdp, unsigned long clk);
int cdns_mhdp_load_firmware(struct cdns_mhdp_device *mhdp, const u32 *i_mem,
			    u32 i_size, const u32 *d_mem, u32 d_size);
int cdns_mhdp_set_firmware_active(struct cdns_mhdp_device *mhdp, bool enable);
int cdns_mhdp_set_host_cap(struct cdns_mhdp_device *mhdp);
int cdns_mhdp_event_config(struct cdns_mhdp_device *mhdp);
u32 cdns_mhdp_get_event(struct cdns_mhdp_device *mhdp);
int cdns_mhdp_dpcd_write(struct cdns_mhdp_device *mhdp, u32 addr, u8 value);
int cdns_mhdp_dpcd_read(struct cdns_mhdp_device *mhdp,
			u32 addr, u8 *data, u16 len);

int cdns_mhdp_get_last_i2c_status(struct cdns_mhdp_device *mhdp, u8 *resp);
int cdns_mhdp_i2c_write(struct cdns_mhdp_device *mhdp, u8 addr,
			u8 *value, u8 mot, u16 len, u16 *respLength);
int cdns_mhdp_i2c_read(struct cdns_mhdp_device *mhdp, u8 addr, u8 *data,
	u16 len, u8 mot, u16 *respLength);
int cdns_mhdp_get_edid_block(void *mhdp, u8 *edid,
			     unsigned int block, size_t length);
int cdns_mhdp_train_link(struct cdns_mhdp_device *mhdp);
int cdns_mhdp_set_video_status(struct cdns_mhdp_device *mhdp, int active);
int cdns_mhdp_config_video(struct cdns_mhdp_device *mhdp);
int cdns_mhdp_apb_conf(struct cdns_mhdp_device *mhdp, u8 sel);

/* Audio */
int cdns_mhdp_audio_stop(struct cdns_mhdp_device *mhdp,
			 struct audio_info *audio);
int cdns_mhdp_audio_mute(struct cdns_mhdp_device *mhdp, bool enable);
int cdns_mhdp_audio_config(struct cdns_mhdp_device *mhdp,
			   struct audio_info *audio);
int cdns_mhdp_register_audio_driver(struct device *dev);
void cdns_mhdp_unregister_audio_driver(struct device *dev);

int cdns_mhdp_reg_read(struct cdns_mhdp_device *mhdp, u32 addr);
int cdns_mhdp_reg_write(struct cdns_mhdp_device *mhdp, u32 addr, u32 val);
int cdns_mhdp_reg_write_bit(struct cdns_mhdp_device *mhdp, u16 addr,
			    u8 start_bit, u8 bits_no, u32 val);
int cdns_mhdp_adjust_lt(struct cdns_mhdp_device *mhdp, u8 nlanes,
			u16 udelay, u8 *lanes_data,
			u8 *dpcd);

int cdns_mhdp_read_hpd(struct cdns_mhdp_device *mhdp);
u32 cdns_phy_reg_read(struct cdns_mhdp_device *mhdp, u32 addr);
int cdns_phy_reg_write(struct cdns_mhdp_device *mhdp, u32 addr, u32 val);
int cdns_mhdp_mailbox_send(struct cdns_mhdp_device *mhdp, u8 module_id,
				  u8 opcode, u16 size, u8 *message);
int cdns_mhdp_mailbox_read_receive(struct cdns_mhdp_device *mhdp,
					  u8 *buff, u16 buff_size);
int cdns_mhdp_mailbox_validate_receive(struct cdns_mhdp_device *mhdp,
					      u8 module_id, u8 opcode,
					      u16 req_size);
void cdns_mhdp_infoframe_set(struct cdns_mhdp_device *mhdp,
					u8 entry_id, u8 packet_len, u8 *packet, u8 packet_type);
int cdns_hdmi_get_edid_block(void *data, u8 *edid, u32 block, size_t length);
int cdns_hdmi_scdc_read(struct cdns_mhdp_device *mhdp, u8 addr, u8 *data);
int cdns_hdmi_scdc_write(struct cdns_mhdp_device *mhdp, u8 addr, u8 value);
int cdns_hdmi_ctrl_init(struct cdns_mhdp_device *mhdp, int protocol, u32 char_rate);
int cdns_hdmi_mode_config(struct cdns_mhdp_device *mhdp, struct drm_display_mode *mode,
				struct video_info *video_info);
int cdns_hdmi_disable_gcp(struct cdns_mhdp_device *mhdp);
int cdns_hdmi_enable_gcp(struct cdns_mhdp_device *mhdp);

bool cdns_mhdp_check_alive(struct cdns_mhdp_device *mhdp);

/* HDMI */
int cdns_hdmi_probe(struct platform_device *pdev,
		 struct cdns_mhdp_device *mhdp);
void cdns_hdmi_remove(struct platform_device *pdev);
void cdns_hdmi_unbind(struct device *dev);
int cdns_hdmi_bind(struct platform_device *pdev,
			struct drm_encoder *encoder, struct cdns_mhdp_device *mhdp);
void cdns_hdmi_set_sample_rate(struct cdns_mhdp_device *mhdp, unsigned int rate);
void cdns_hdmi_audio_enable(struct cdns_mhdp_device *mhdp);
void cdns_hdmi_audio_disable(struct cdns_mhdp_device *mhdp);

/* DP  */
int cdns_dp_probe(struct platform_device *pdev,
		 struct cdns_mhdp_device *mhdp);
void cdns_dp_remove(struct platform_device *pdev);
void cdns_dp_unbind(struct device *dev);
int cdns_dp_bind(struct platform_device *pdev,
			struct drm_encoder *encoder, struct cdns_mhdp_device *mhdp);
int cdns_hdmi_set_plugged_cb(struct cdns_mhdp_device *mhdp, hdmi_codec_plugged_cb fn,
			     struct device *codec_dev);

/* CEC */
#ifdef CONFIG_DRM_CDNS_HDMI_CEC
int cdns_mhdp_register_cec_driver(struct cdns_mhdp_cec *cec);
int cdns_mhdp_unregister_cec_driver(struct cdns_mhdp_cec *cec);
#endif

#endif /* CDNS_MHDP_H_ */
