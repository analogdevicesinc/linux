/*
 * Copyright 2020 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef _CDNS_MHDP_HDMIRX_
#define _CDNS_MHDP_HDMIRX_

#include <linux/module.h>
#include <linux/mutex.h>

#include <linux/clk.h>
#include <linux/kthread.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-dv-timings.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-subdev.h>
#include <drm/drm_of.h>
#include <drm/drm_print.h>

#include <uapi/linux/v4l2-dv-timings.h>
#include <drm/bridge/cdns-mhdp.h>
#include "../imx8-common.h"

#define HDMIRX_DRIVER_NAME        "mxc-hdmi-rx"
#define HDMIRX_SUBDEV_NAME        HDMIRX_DRIVER_NAME

/* HDMIRX Subsystem CSR */
#define CSR_PIXEL_LINK_ENC_CTL		0x00
#define PL_ENC_CTL_PXL_VAL          15
#define PL_ENC_CTL_PXL_VPP          14
#define PL_ENC_CTL_PXL_HPP          13
#define PL_ENC_CTL_PXL_VCP          12
#define PL_ENC_CTL_PXL_HCP          11
#define PL_ENC_CTL_PXL_ADD          9
#define PL_ENC_CTL_PXL_EXT          7
#define PL_ENC_CTL_PXL_EN           6
#define PL_ENC_CTL_PXL_ITC          4
#define PL_ENC_CTL_PXL_ODD_EVEN     3
#define PL_ENC_CTL_PXL_TYP          1
#define PL_ENC_CTL_PXL_YUV          0

#define CSR_HDP_RX_CTRL_CTRL0		0x04
#define CSR_HDP_RX_CTRL_CTRL1		0x08

/* Module ID Code */
#define MB_MODULE_ID_HDMI_RX        0x04
#define MB_MODULE_ID_HDCP_RX        0x08
#define MB_MODULE_ID_HDCP_GENERAL	0x09

/* HDMI RX opcode */
#define HDMI_RX_SET_EDID			0x00
#define HDMI_RX_SCDC_SET			0x01
#define HDMI_RX_SCDC_GET			0x02
#define HDMI_RX_SET_HPD				0x04

#define HDMI_RX_READ_EVENTS			0x03
#define HDMI_RX_DEBUG_ECHO			0xAA
#define HDMI_RX_TEST				0xBB

/* HDCP RX opcode */
#define HDCP_RX_SET_CONFIG			0x04
#define HDCP_RX_GET_STATUS			0x05
#define HDCP_RX_NOT_SYNC			0x06

/* Sink VIF */
#define VIDEO_UNPACK_CTRL			0x1804
#define F_CD_ENABLE(x) (((x) & ((1 << 1) - 1)) << 1)

#define VANLYZ_CTRL					0x1810
#define F_VANLYZ_START(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_VANLYZ_RESET(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_VANLYZ_FRAMES_CHECK_EN(x) (((x) & ((1 << 1) - 1)) << 2)
#define F_VANLYZ_FORMAT_FINDER_EN(x) (((x) & ((1 << 1) - 1)) << 3)

/* Sink_CAR */
#define ADDR_SINK_CAR 0x00900
#define SINK_MHL_HD_CAR 0x00900
#define SINK_CEC_CAR 0x00904
#define F_SINK_CEC_SYS_CLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SINK_CEC_SYS_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 1)

/* Sink MHL HD_Comp */
#define ADDR_SINK_MHL_HD 0x01000
#define TMDS_DEC_CTRL 0x1004
#define F_DECODER_ERR_CORR_EN(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_TMDS_DECODER_SW_RST(x) (((x) & ((1 << 1) - 1)) << 1)

#define TMDS_DEC_ST 0x1008

#define TMDS_CH0_ERR_CNT 0x100c

#define TMDS_CH1_ERR_CNT 0x1010

#define TMDS_CH2_ERR_CNT 0x1014

#define PKT_ERR_CNT_HEADER 0x1034

#define PKT_ERR_CNT_01 0x1038

#define PKT_ERR_CNT_23 0x103c

#define TMDS_SCR_CTRL 0x1040
#define F_SCRAMBLER_MODE(x) (((x) & ((1 << 1) - 1)) << 0)

#define TMDS_SCR_CNT_INT_CTRL 0x1044
#define F_SCRAMBLER_SSCP_LINE_DET_THR(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_SCRAMBLER_CTRL_LINE_DET_THR(x) (((x) & ((1 << 24) - 1)) << 8)

#define TMDS_SCR_VALID_CTRL 0x1048
#define F_SCRAMBLER_SSCP_LINE_VALID_THR(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_SCRAMBLER_CTRL_LINE_VALID_THR(x) (((x) & ((1 << 24) - 1)) << 8)

#define PKT_AVI_DATA_LOW 0x10A4
#define PKT_AVI_DATA_HIGH 0x10A8

/* HDMIRX Audio */
#define ADDR_AIF_ENCODER 0x30000
#define PKT2SMPL_CNTL 0x30000
#define F_PKT2SMPL_EN(x) (((x) & ((1 << 1) - 1)) << 1)

/* register ACR_CFG */
#define ACR_CFG 0x30004
#define F_ACR_SW_RESET(x) (((x) & ((1 << 1) - 1)) << 5)

/* register AIF_ACR_N_ST */
#define AIF_ACR_N_ST 0x30028

/* register AIF_ACR_N_OFST_CFG */
#define AIF_ACR_N_OFST_CFG 0x30030
#define F_ACR_N_OFFSET(x) (((x) & ((1 << 24) - 1)) << 0)

/* register AUDIO_SINK_CNTL */
#define AUDIO_SINK_CNTL 0x30100
#define F_I2S_ENC_START(x) (((x) & ((1 << 1) - 1)) << 1)

/* register AUDIO_SINK_CNFG */
#define AUDIO_SINK_CNFG 0x30108
#define F_ENC_LOW_INDEX_MSB(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SINK_AUDIO_CH_NUM(x) (((x) & ((1 << 5) - 1)) << 1)
#define F_ENC_SAMPLE_JUST(x) (((x) & ((1 << 2) - 1)) << 6)
#define F_ENC_SMPL_WIDTH(x) (((x) & ((1 << 2) - 1)) << 8)
#define F_I2S_ENC_WL_SIZE(x) (((x) & ((1 << 2) - 1)) << 10)
#define F_CNTL_SMPL_ONLY_EN(x) (((x) & ((1 << 1) - 1)) << 12)
#define F_CNTL_TYPE_OVRD(x) (((x) & ((1 << 4) - 1)) << 17)
#define F_CNTL_TYPE_OVRD_EN(x) (((x) & ((1 << 1) - 1)) << 21)
#define F_I2S_ENC_PORT_EN(x) (((x) & ((1 << 4) - 1)) << 22)
#define F_WS_POLARITY(x) (((x) & ((1 << 1) - 1)) << 26)

/* register FIFO_CNTL_ADDR */
#define FIFO_CNTL_ADDR 0x3010C
#define F_CFG_FIFO_SW_RST(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_CFG_INDEX_SYNC_EN(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_CFG_FIFO_DIR(x) (((x) & ((1 << 1) - 1)) << 2)
#define F_CFG_DIS_PORT3(x) (((x) & ((1 << 1) - 1)) << 3)

/* HDMI Sink PIF */
#define ADDR_SINK_PIF 0x30800

#define PKT_INFO_TYPE_CFG1 0x30800
#define F_INFO_TYPE1(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_INFO_TYPE2(x) (((x) & ((1 << 8) - 1)) << 8)

#define PKT_INFO_CTRL 0x30810
#define F_PACKET_RDN_WR(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_PACKET_NUM(x) (((x) & ((1 << 4) - 1)) << 1)

#define PKT_INFO_HEADER 0x3081c
#define PKT_INT_STATUS 0x3083c
#define PKT_INT_MASK 0x30840
#define PKT_TRANS_CTRL 0x30844

/* From HDCP standard */
#define HDCP_RRX_SIZE 8
#define HDCP_KSV_SIZE 5
#define HDCP_NUM_DEVICE_KEY_SIZE 7
#define HDCP_NUM_OF_DEVICE_KEYS 40

#define HDCPRX_NUM_LONG_DELAYS 5
#define HDCPRX_DEBUG_NUM_ERRORS 5

#define HDCPRX_ERR_NONE 0
#define HDCPRX_ERR_5V 1
#define HDCPRX_ERR_DDC 2
#define HDCPRX_ERR_SYNC 3

#define HDCPRX_VERSION_2 0x0
#define HDCPRX_VERSION_1 0x1
#define HDCPRX_VERSION_BOTH 0x2

#define ktime_timeout_ms(ms_) ktime_add(ktime_get(), ms_to_ktime(ms_))

/**
 * HDCP config.
 *
 * @field activate [in] Activate HDCP module.
 * @field version [in] Supported HDCP versions.
 *      0 - HDCP 2.2 only.
 *      1 - HDCP 1.3/1.4 only.
 *      2 - HDCP 1.3/1.4 and 2.2.
 *      3 - reserved
 * @field repeater [in] Set for HDCP Repeater.
 * @field use_secondary_link [in] Set to use the Secondary Link.
 * @field use_km_key [in] Set to enable km-key encryption.
 * @field bcaps [in] Bcaps value. HDCP 1.3/1.4 only.
 * @field bstatus [in] Bstatus value. HDCP 1.3/1.4 only.
 */
struct hdcprx_config {
	u8 activate : 1;
	u8 version : 2;
	u8 repeater : 1;
	u8 use_secondary_link : 1;
	u8 use_km_key : 1;
	u8 bcaps; // for HDCP 1.4
	u16 bstatus; // for HDCP 1.4
};

enum clk_ratio_t{
	CLK_RATIO_1_1,
	CLK_RATIO_5_4,
	CLK_RATIO_3_2,
	CLK_RATIO_2_1,
	CLK_RATIO_1_2,
	CLK_RATIO_5_8,
	CLK_RATIO_3_4
};

enum {
	PIXEL_ENCODING_RGB    = 0,
	PIXEL_ENCODING_YUV422 = 1,
	PIXEL_ENCODING_YUV444 = 2,
	PIXEL_ENCODING_YUV420 = 3,
};

/**
 * HDCP Receiver Status.
 *
 * @field key_arrived [out] TODO: what key?
 * @field hdcp_ver [out] HDCP version.
 *      0 - no HDCP
 *      1 - HDCP 1.3/1.4
 *      2 - HDCP 2.2
 *      3 - not valid / reserved
 * @field error [out] Last error code.
 * @field aksv [out] Aksv value sent by HDCP Source (HDCP 1.3/1.4 only).
 * @field ainfo [out] Ainfo value sent by HDCP Source (HDCP 1.3/1.4 only).
 */
struct hdcprx_status {
	u8 flags;
	u8 aksv[HDCP_KSV_SIZE];
	u8 ainfo;
} ;


struct cdns_hdmirx_dev_video_standards {
	struct v4l2_dv_timings timings;
	u8 vic;
	u8 hdmi_vic;
	u8 fps;
};

struct S_HDMI_SCDC_SET_MSG{
	u8 sink_ver;
	u8 manufacturer_oui_1;
	u8 manufacturer_oui_2;
	u8 manufacturer_oui_3;
	u8 devId[8];
	u8 hardware_major_rev;
	u8 hardware_minor_rev;
	u8 software_major_rev;
	u8 software_minor_rev;
	u8 manufacturerSpecific[34];
};

struct S_HDMI_SCDC_GET_MSG {
	u8 source_ver;
	u8 TMDS_Config;
	u8 config_0;
	u8 manufacturerSpecific[34];
};

struct hdmirx_edid_set_msg {
	u8 segment;
	u8 extension;
	u8 edid[128];
};

enum hdp_rx_irq {
	HPD5V_IRQ_IN,
	HPD5V_IRQ_OUT,
	HPD5V_IRQ_NUM,
};

struct cdns_hdmirx_device {
	struct v4l2_subdev sd;
	struct mutex			lock;
	wait_queue_head_t		irq_queue;
	struct media_pad pads[MXC_HDMI_RX_PADS_NUM];

	struct platform_device		*pdev;
	struct v4l2_device			*v4l2_dev;
	struct v4l2_async_connection asd;
	struct v4l2_ctrl_handler ctrl_hdl;
	struct v4l2_mbus_framefmt format;
	struct v4l2_fract aspect_ratio;
	struct {
		u8 edid[512];
		u32 present;
		u32 blocks;
	} edid;

	struct clk		*ref_clk;
	struct clk		*pxl_clk;
	struct clk		*i2s_clk;
	struct clk		*spdif_clk;
	struct clk		*lpcg_sclk;
	struct clk		*lpcg_pclk;
	struct clk		*lpcg_enc_clk;
	struct clk		*lpcg_pxl_link_clk;
	void __iomem		*regs_base;
	void __iomem		*regs_sec;

	u32 flags;
    struct S_HDMI_SCDC_GET_MSG          scdcData;
	int bus_type;

	struct cdns_hdmirx_dev_video_standards *timings;
    u8 vic_code;
	u8 hdmi_vic;
    u8 pixel_encoding;
	u8 color_depth;
	bool cable_plugin;

	u8 is_cec;
	bool cec_running;
	struct cdns_mhdp_cec  cec;
	u32 sample_rate;
	u32 sample_width;
	u32 channels;

	int irq[HPD5V_IRQ_NUM];
	struct delayed_work hpd5v_work;
	int tmds_clk;
	struct task_struct *tmdsmon_th;
	int tmdsmon_state;
	u32 last_avi;
	u8 avi[13];
	u8 vnd[10];
	bool allow_hdcp;
	bool hdcp_fw_loaded;
	bool initialized;
	struct mutex pif_mutex;
	struct mutex iolock;

	u8 last_5v_state;
	u8 rescal_val;
	u8 slicer_tune_val;
	u8 running;

	const struct firmware *fw;
	const char *firmware_name;
};

enum hdmirx_power_state {
	HDMIRX_PM_SUSPENDED = 0x01,
	HDMIRX_PM_POWERED = 0x02,
	HDMIRX_RUNTIME_SUSPEND = 0x04,
};

int cdns_hdmirx_mailbox_send(struct cdns_hdmirx_device *hdmirx, u8 module_id,
				  u8 opcode, u16 size, u8 *message);
int cdns_hdmirx_mailbox_validate_receive(struct cdns_hdmirx_device *hdmirx,
					      u8 module_id, u8 opcode, u16 req_size);
int cdns_hdmirx_mailbox_read_receive(struct cdns_hdmirx_device *hdmirx,
					  u8 *buff, u16 buff_size);
void cdns_hdmirx_bus_write(u32 val, struct cdns_hdmirx_device *hdmirx, u32 offset);
u32 cdns_hdmirx_bus_read(struct cdns_hdmirx_device *hdmirx, u32 offset);
int cdns_hdmirx_reg_read(struct cdns_hdmirx_device *hdmirx, u32 addr);
int cdns_hdmirx_reg_write(struct cdns_hdmirx_device *hdmirx, u32 addr, u32 val);
int cdns_hdmirx_read_hpd(struct cdns_hdmirx_device *hdmirx);
int cdns_hdmirx_sethpd(struct cdns_hdmirx_device *hdmirx, u8 hpd);
void cdns_hdmirx_hotplug_trigger(struct cdns_hdmirx_device *hdmirx);
void cdns_hdmirx_set_clock(struct cdns_hdmirx_device *hdmirx, int clk);
bool cdns_hdmirx_check_alive(struct cdns_hdmirx_device *hdmirx);
int cdns_hdmirx_maincontrol(struct cdns_hdmirx_device *hdmirx, u8 mode);
int cdns_hdmirx_readevent(struct cdns_hdmirx_device *hdmirx, u8 *events_5v);
int cdns_hdmirx_set_edid(struct cdns_hdmirx_device *hdmirx,
						u8 segment,	u8 extension, u8 *edid);
int cdns_hdmirx_wait_edid_read(struct cdns_hdmirx_device *hdmirx);
int cdns_hdmirx_set_scdc_slave(struct cdns_hdmirx_device *hdmirx,
							struct S_HDMI_SCDC_SET_MSG *scdcdata);
int cdns_hdmirx_get_scdc_slave(struct cdns_hdmirx_device *hdmirx,
					     struct S_HDMI_SCDC_GET_MSG *scdcdata);
int cdns_hdmirx_general_loadhdcprx(struct cdns_hdmirx_device *hdmirx);
int cdns_hdmirx_general_unloadhdcprx(struct cdns_hdmirx_device *hdmirx);
int cdns_hdmirx_general_assertphyreset(struct cdns_hdmirx_device *hdmirx);
int cdns_hdmirx_general_deassertphyreset(struct cdns_hdmirx_device *hdmirx);

int cdns_hdmirx_phyinit(struct cdns_hdmirx_device *hdmirx);
int cdns_hdmirx_startup(struct cdns_hdmirx_device *hdmirx);
int cdns_hdmirx_init(struct cdns_hdmirx_device *hdmirx);
int cdns_hdmirx_get_stable_tmds(struct cdns_hdmirx_device *hdmirx);
int cdns_hdmirx_frame_timing(struct cdns_hdmirx_device *hdmirx);
void imx8qm_hdmi_phy_reset(struct cdns_hdmirx_device *hdmirx, u8 reset);
int cdns_hdmirx_get_avi_infoframe(struct cdns_hdmirx_device *hdmirx, u16 timeout_ms);
int cdns_hdmirx_get_vendor_infoframe(struct cdns_hdmirx_device *hdmirx, u16 timeout_ms);
int infoframe_poll(struct cdns_hdmirx_device *hdmirx, u8 type, u8 *buf, u16 timeout_ms);

/* HDMIRX HDCP  */
int cdns_hdcprx_reauth_req_wait(struct cdns_hdmirx_device *hdmirx, u16 timeout_ms);
int cdns_hdcprx_get_status(struct cdns_hdmirx_device *hdmirx,
			   struct hdcprx_status *status);
void cdns_hdcprx_enable(struct cdns_hdmirx_device *hdmirx);
void cdns_hdcprx_disable(struct cdns_hdmirx_device *hdmirx);
int cdns_hdcprx_wait_auth_complete(struct cdns_hdmirx_device *hdmirx, u16 timeout_ms);

/* HDMIRX Audio */
int cdns_hdmirx_audioautoconfig(
						struct cdns_hdmirx_device *hdmirx,
						u8 max_ch_num,
						u8 i2s_ports_num,
						u8 dis_port3,
						u8 enc_sample_width,
						u8 i2s_sample_width);
void cdns_hdmirx_register_audio_driver(struct device *dev);
#endif
