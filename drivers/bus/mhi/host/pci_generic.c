// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * MHI PCI driver - MHI over PCI controller driver
 *
 * This module is a generic driver for registering MHI-over-PCI devices,
 * such as PCIe QCOM modems.
 *
 * Copyright (C) 2020 Linaro Ltd <loic.poulain@linaro.org>
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/mhi.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/pm_runtime.h>
#include <linux/timer.h>
#include <linux/workqueue.h>

#define MHI_PCI_DEFAULT_BAR_NUM 0

#define MHI_POST_RESET_DELAY_MS 2000

#define HEALTH_CHECK_PERIOD (HZ * 2)

/* PCI VID definitions */
#define PCI_VENDOR_ID_THALES	0x1269
#define PCI_VENDOR_ID_QUECTEL	0x1eac
#define PCI_VENDOR_ID_NETPRISMA	0x203e

#define MHI_EDL_DB			91
#define MHI_EDL_COOKIE			0xEDEDEDED

/**
 * struct mhi_pci_dev_info - MHI PCI device specific information
 * @config: MHI controller configuration
 * @vf_config: MHI controller configuration for Virtual function (optional)
 * @name: name of the PCI module
 * @fw: firmware path (if any)
 * @edl: emergency download mode firmware path (if any)
 * @edl_trigger: capable of triggering EDL mode in the device (if supported)
 * @bar_num: PCI base address register to use for MHI MMIO register space
 * @dma_data_width: DMA transfer word size (32 or 64 bits)
 * @vf_dma_data_width: DMA transfer word size for VF's (optional)
 * @mru_default: default MRU size for MBIM network packets
 * @sideband_wake: Devices using dedicated sideband GPIO for wakeup instead
 *		   of inband wake support (such as sdx24)
 * @no_m3: M3 not supported
 * @reset_on_remove: Set true for devices that require SoC during driver removal
 */
struct mhi_pci_dev_info {
	const struct mhi_controller_config *config;
	const struct mhi_controller_config *vf_config;
	const char *name;
	const char *fw;
	const char *edl;
	bool edl_trigger;
	unsigned int bar_num;
	unsigned int dma_data_width;
	unsigned int vf_dma_data_width;
	unsigned int mru_default;
	bool sideband_wake;
	bool no_m3;
	bool reset_on_remove;
};

#define MHI_CHANNEL_CONFIG_UL(ch_num, ch_name, el_count, ev_ring) \
	{						\
		.num = ch_num,				\
		.name = ch_name,			\
		.num_elements = el_count,		\
		.event_ring = ev_ring,			\
		.dir = DMA_TO_DEVICE,			\
		.ee_mask = BIT(MHI_EE_AMSS),		\
		.pollcfg = 0,				\
		.doorbell = MHI_DB_BRST_DISABLE,	\
		.lpm_notify = false,			\
		.offload_channel = false,		\
		.doorbell_mode_switch = false,		\
	}						\

#define MHI_CHANNEL_CONFIG_DL(ch_num, ch_name, el_count, ev_ring) \
	{						\
		.num = ch_num,				\
		.name = ch_name,			\
		.num_elements = el_count,		\
		.event_ring = ev_ring,			\
		.dir = DMA_FROM_DEVICE,			\
		.ee_mask = BIT(MHI_EE_AMSS),		\
		.pollcfg = 0,				\
		.doorbell = MHI_DB_BRST_DISABLE,	\
		.lpm_notify = false,			\
		.offload_channel = false,		\
		.doorbell_mode_switch = false,		\
	}

#define MHI_CHANNEL_CONFIG_DL_AUTOQUEUE(ch_num, ch_name, el_count, ev_ring) \
	{						\
		.num = ch_num,				\
		.name = ch_name,			\
		.num_elements = el_count,		\
		.event_ring = ev_ring,			\
		.dir = DMA_FROM_DEVICE,			\
		.ee_mask = BIT(MHI_EE_AMSS),		\
		.pollcfg = 0,				\
		.doorbell = MHI_DB_BRST_DISABLE,	\
		.lpm_notify = false,			\
		.offload_channel = false,		\
		.doorbell_mode_switch = false,		\
		.auto_queue = true,			\
	}

#define MHI_EVENT_CONFIG_CTRL(ev_ring, el_count) \
	{					\
		.num_elements = el_count,	\
		.irq_moderation_ms = 0,		\
		.irq = (ev_ring) + 1,		\
		.priority = 1,			\
		.mode = MHI_DB_BRST_DISABLE,	\
		.data_type = MHI_ER_CTRL,	\
		.hardware_event = false,	\
		.client_managed = false,	\
		.offload_channel = false,	\
	}

#define MHI_CHANNEL_CONFIG_HW_UL(ch_num, ch_name, el_count, ev_ring) \
	{						\
		.num = ch_num,				\
		.name = ch_name,			\
		.num_elements = el_count,		\
		.event_ring = ev_ring,			\
		.dir = DMA_TO_DEVICE,			\
		.ee_mask = BIT(MHI_EE_AMSS),		\
		.pollcfg = 0,				\
		.doorbell = MHI_DB_BRST_ENABLE,	\
		.lpm_notify = false,			\
		.offload_channel = false,		\
		.doorbell_mode_switch = true,		\
	}						\

#define MHI_CHANNEL_CONFIG_HW_DL(ch_num, ch_name, el_count, ev_ring) \
	{						\
		.num = ch_num,				\
		.name = ch_name,			\
		.num_elements = el_count,		\
		.event_ring = ev_ring,			\
		.dir = DMA_FROM_DEVICE,			\
		.ee_mask = BIT(MHI_EE_AMSS),		\
		.pollcfg = 0,				\
		.doorbell = MHI_DB_BRST_ENABLE,	\
		.lpm_notify = false,			\
		.offload_channel = false,		\
		.doorbell_mode_switch = true,		\
	}

#define MHI_CHANNEL_CONFIG_UL_SBL(ch_num, ch_name, el_count, ev_ring) \
	{						\
		.num = ch_num,				\
		.name = ch_name,			\
		.num_elements = el_count,		\
		.event_ring = ev_ring,			\
		.dir = DMA_TO_DEVICE,			\
		.ee_mask = BIT(MHI_EE_SBL),		\
		.pollcfg = 0,				\
		.doorbell = MHI_DB_BRST_DISABLE,	\
		.lpm_notify = false,			\
		.offload_channel = false,		\
		.doorbell_mode_switch = false,		\
	}						\

#define MHI_CHANNEL_CONFIG_DL_SBL(ch_num, ch_name, el_count, ev_ring) \
	{						\
		.num = ch_num,				\
		.name = ch_name,			\
		.num_elements = el_count,		\
		.event_ring = ev_ring,			\
		.dir = DMA_FROM_DEVICE,			\
		.ee_mask = BIT(MHI_EE_SBL),		\
		.pollcfg = 0,				\
		.doorbell = MHI_DB_BRST_DISABLE,	\
		.lpm_notify = false,			\
		.offload_channel = false,		\
		.doorbell_mode_switch = false,		\
	}

#define MHI_CHANNEL_CONFIG_UL_FP(ch_num, ch_name, el_count, ev_ring) \
	{						\
		.num = ch_num,				\
		.name = ch_name,			\
		.num_elements = el_count,		\
		.event_ring = ev_ring,			\
		.dir = DMA_TO_DEVICE,			\
		.ee_mask = BIT(MHI_EE_FP),		\
		.pollcfg = 0,				\
		.doorbell = MHI_DB_BRST_DISABLE,	\
		.lpm_notify = false,			\
		.offload_channel = false,		\
		.doorbell_mode_switch = false,		\
	}						\

#define MHI_CHANNEL_CONFIG_DL_FP(ch_num, ch_name, el_count, ev_ring) \
	{						\
		.num = ch_num,				\
		.name = ch_name,			\
		.num_elements = el_count,		\
		.event_ring = ev_ring,			\
		.dir = DMA_FROM_DEVICE,			\
		.ee_mask = BIT(MHI_EE_FP),		\
		.pollcfg = 0,				\
		.doorbell = MHI_DB_BRST_DISABLE,	\
		.lpm_notify = false,			\
		.offload_channel = false,		\
		.doorbell_mode_switch = false,		\
	}

#define MHI_EVENT_CONFIG_DATA(ev_ring, el_count) \
	{					\
		.num_elements = el_count,	\
		.irq_moderation_ms = 5,		\
		.irq = (ev_ring) + 1,		\
		.priority = 1,			\
		.mode = MHI_DB_BRST_DISABLE,	\
		.data_type = MHI_ER_DATA,	\
		.hardware_event = false,	\
		.client_managed = false,	\
		.offload_channel = false,	\
	}

#define MHI_EVENT_CONFIG_SW_DATA(ev_ring, el_count) \
	{					\
		.num_elements = el_count,	\
		.irq_moderation_ms = 0,		\
		.irq = (ev_ring) + 1,		\
		.priority = 1,			\
		.mode = MHI_DB_BRST_DISABLE,	\
		.data_type = MHI_ER_DATA,	\
		.hardware_event = false,	\
		.client_managed = false,	\
		.offload_channel = false,	\
	}

#define MHI_EVENT_CONFIG_HW_DATA(ev_ring, el_count, ch_num) \
	{					\
		.num_elements = el_count,	\
		.irq_moderation_ms = 1,		\
		.irq = (ev_ring) + 1,		\
		.priority = 1,			\
		.mode = MHI_DB_BRST_DISABLE,	\
		.data_type = MHI_ER_DATA,	\
		.hardware_event = true,		\
		.client_managed = false,	\
		.offload_channel = false,	\
		.channel = ch_num,		\
	}

static const struct mhi_channel_config mhi_qcom_qdu100_channels[] = {
	MHI_CHANNEL_CONFIG_UL(0, "LOOPBACK", 32, 2),
	MHI_CHANNEL_CONFIG_DL(1, "LOOPBACK", 32, 2),
	MHI_CHANNEL_CONFIG_UL_SBL(2, "SAHARA", 128, 1),
	MHI_CHANNEL_CONFIG_DL_SBL(3, "SAHARA", 128, 1),
	MHI_CHANNEL_CONFIG_UL(4, "DIAG", 64, 3),
	MHI_CHANNEL_CONFIG_DL(5, "DIAG", 64, 3),
	MHI_CHANNEL_CONFIG_UL(9, "QDSS", 64, 3),
	MHI_CHANNEL_CONFIG_UL(14, "NMEA", 32, 4),
	MHI_CHANNEL_CONFIG_DL(15, "NMEA", 32, 4),
	MHI_CHANNEL_CONFIG_UL(16, "CSM_CTRL", 32, 4),
	MHI_CHANNEL_CONFIG_DL(17, "CSM_CTRL", 32, 4),
	MHI_CHANNEL_CONFIG_UL(40, "MHI_PHC", 32, 4),
	MHI_CHANNEL_CONFIG_DL(41, "MHI_PHC", 32, 4),
	MHI_CHANNEL_CONFIG_UL(46, "IP_SW0", 256, 5),
	MHI_CHANNEL_CONFIG_DL(47, "IP_SW0", 256, 5),
};

static struct mhi_event_config mhi_qcom_qdu100_events[] = {
	/* first ring is control+data ring */
	MHI_EVENT_CONFIG_CTRL(0, 64),
	/* SAHARA dedicated event ring */
	MHI_EVENT_CONFIG_SW_DATA(1, 256),
	/* Software channels dedicated event ring */
	MHI_EVENT_CONFIG_SW_DATA(2, 64),
	MHI_EVENT_CONFIG_SW_DATA(3, 256),
	MHI_EVENT_CONFIG_SW_DATA(4, 256),
	/* Software IP channels dedicated event ring */
	MHI_EVENT_CONFIG_SW_DATA(5, 512),
	MHI_EVENT_CONFIG_SW_DATA(6, 512),
	MHI_EVENT_CONFIG_SW_DATA(7, 512),
};

static const struct mhi_controller_config mhi_qcom_qdu100_config = {
	.max_channels = 128,
	.timeout_ms = 120000,
	.num_channels = ARRAY_SIZE(mhi_qcom_qdu100_channels),
	.ch_cfg = mhi_qcom_qdu100_channels,
	.num_events = ARRAY_SIZE(mhi_qcom_qdu100_events),
	.event_cfg = mhi_qcom_qdu100_events,
};

static const struct mhi_pci_dev_info mhi_qcom_qdu100_info = {
	.name = "qcom-qdu100",
	.fw = "qcom/qdu100/xbl_s.melf",
	.edl_trigger = true,
	.config = &mhi_qcom_qdu100_config,
	.bar_num = MHI_PCI_DEFAULT_BAR_NUM,
	.dma_data_width = 32,
	.vf_dma_data_width = 40,
	.sideband_wake = false,
	.no_m3 = true,
	.reset_on_remove = true,
};

static const struct mhi_channel_config mhi_qcom_sa8775p_channels[] = {
	MHI_CHANNEL_CONFIG_UL(46, "IP_SW0", 2048, 1),
	MHI_CHANNEL_CONFIG_DL(47, "IP_SW0", 2048, 2),
};

static struct mhi_event_config mhi_qcom_sa8775p_events[] = {
	/* first ring is control+data ring */
	MHI_EVENT_CONFIG_CTRL(0, 64),
	/* Software channels dedicated event ring */
	MHI_EVENT_CONFIG_SW_DATA(1, 64),
	MHI_EVENT_CONFIG_SW_DATA(2, 64),
};

static const struct mhi_channel_config modem_qcom_v1_mhi_channels[] = {
	MHI_CHANNEL_CONFIG_UL(4, "DIAG", 16, 1),
	MHI_CHANNEL_CONFIG_DL(5, "DIAG", 16, 1),
	MHI_CHANNEL_CONFIG_UL(12, "MBIM", 4, 0),
	MHI_CHANNEL_CONFIG_DL(13, "MBIM", 4, 0),
	MHI_CHANNEL_CONFIG_UL(14, "QMI", 4, 0),
	MHI_CHANNEL_CONFIG_DL(15, "QMI", 4, 0),
	MHI_CHANNEL_CONFIG_UL(20, "IPCR", 8, 0),
	MHI_CHANNEL_CONFIG_DL_AUTOQUEUE(21, "IPCR", 8, 0),
	MHI_CHANNEL_CONFIG_UL_FP(34, "FIREHOSE", 32, 0),
	MHI_CHANNEL_CONFIG_DL_FP(35, "FIREHOSE", 32, 0),
	MHI_CHANNEL_CONFIG_UL(46, "IP_SW0", 64, 2),
	MHI_CHANNEL_CONFIG_DL(47, "IP_SW0", 64, 3),
	MHI_CHANNEL_CONFIG_HW_UL(100, "IP_HW0", 128, 4),
	MHI_CHANNEL_CONFIG_HW_DL(101, "IP_HW0", 128, 5),
};

static struct mhi_event_config modem_qcom_v1_mhi_events[] = {
	/* first ring is control+data ring */
	MHI_EVENT_CONFIG_CTRL(0, 64),
	/* DIAG dedicated event ring */
	MHI_EVENT_CONFIG_DATA(1, 128),
	/* Software channels dedicated event ring */
	MHI_EVENT_CONFIG_SW_DATA(2, 64),
	MHI_EVENT_CONFIG_SW_DATA(3, 64),
	/* Hardware channels request dedicated hardware event rings */
	MHI_EVENT_CONFIG_HW_DATA(4, 1024, 100),
	MHI_EVENT_CONFIG_HW_DATA(5, 2048, 101)
};

static const struct mhi_controller_config mhi_qcom_sa8775p_config = {
	.max_channels = 128,
	.timeout_ms = 8000,
	.num_channels = ARRAY_SIZE(mhi_qcom_sa8775p_channels),
	.ch_cfg = mhi_qcom_sa8775p_channels,
	.num_events = ARRAY_SIZE(mhi_qcom_sa8775p_events),
	.event_cfg = mhi_qcom_sa8775p_events,
};

static const struct mhi_controller_config modem_qcom_v2_mhiv_config = {
	.max_channels = 128,
	.timeout_ms = 8000,
	.ready_timeout_ms = 50000,
	.num_channels = ARRAY_SIZE(modem_qcom_v1_mhi_channels),
	.ch_cfg = modem_qcom_v1_mhi_channels,
	.num_events = ARRAY_SIZE(modem_qcom_v1_mhi_events),
	.event_cfg = modem_qcom_v1_mhi_events,
};

static const struct mhi_controller_config modem_qcom_v1_mhiv_config = {
	.max_channels = 128,
	.timeout_ms = 8000,
	.num_channels = ARRAY_SIZE(modem_qcom_v1_mhi_channels),
	.ch_cfg = modem_qcom_v1_mhi_channels,
	.num_events = ARRAY_SIZE(modem_qcom_v1_mhi_events),
	.event_cfg = modem_qcom_v1_mhi_events,
};

static const struct mhi_pci_dev_info mhi_qcom_sa8775p_info = {
	.name = "qcom-sa8775p",
	.edl_trigger = false,
	.config = &mhi_qcom_sa8775p_config,
	.bar_num = MHI_PCI_DEFAULT_BAR_NUM,
	.dma_data_width = 32,
	.mru_default = 32768,
	.sideband_wake = false,
};

static const struct mhi_pci_dev_info mhi_qcom_sdx75_info = {
	.name = "qcom-sdx75m",
	.fw = "qcom/sdx75m/xbl.elf",
	.edl = "qcom/sdx75m/edl.mbn",
	.edl_trigger = true,
	.config = &modem_qcom_v2_mhiv_config,
	.bar_num = MHI_PCI_DEFAULT_BAR_NUM,
	.dma_data_width = 32,
	.sideband_wake = false,
};

static const struct mhi_pci_dev_info mhi_qcom_sdx65_info = {
	.name = "qcom-sdx65m",
	.fw = "qcom/sdx65m/xbl.elf",
	.edl = "qcom/sdx65m/edl.mbn",
	.edl_trigger = true,
	.config = &modem_qcom_v1_mhiv_config,
	.bar_num = MHI_PCI_DEFAULT_BAR_NUM,
	.dma_data_width = 32,
	.sideband_wake = false,
};

static const struct mhi_pci_dev_info mhi_qcom_sdx55_info = {
	.name = "qcom-sdx55m",
	.fw = "qcom/sdx55m/sbl1.mbn",
	.edl = "qcom/sdx55m/edl.mbn",
	.edl_trigger = true,
	.config = &modem_qcom_v1_mhiv_config,
	.bar_num = MHI_PCI_DEFAULT_BAR_NUM,
	.dma_data_width = 32,
	.mru_default = 32768,
	.sideband_wake = false,
};

static const struct mhi_pci_dev_info mhi_qcom_sdx24_info = {
	.name = "qcom-sdx24",
	.edl = "qcom/prog_firehose_sdx24.mbn",
	.config = &modem_qcom_v1_mhiv_config,
	.bar_num = MHI_PCI_DEFAULT_BAR_NUM,
	.dma_data_width = 32,
	.sideband_wake = true,
};

static const struct mhi_channel_config mhi_quectel_em1xx_channels[] = {
	MHI_CHANNEL_CONFIG_UL(0, "NMEA", 32, 0),
	MHI_CHANNEL_CONFIG_DL(1, "NMEA", 32, 0),
	MHI_CHANNEL_CONFIG_UL_SBL(2, "SAHARA", 32, 0),
	MHI_CHANNEL_CONFIG_DL_SBL(3, "SAHARA", 32, 0),
	MHI_CHANNEL_CONFIG_UL(4, "DIAG", 32, 1),
	MHI_CHANNEL_CONFIG_DL(5, "DIAG", 32, 1),
	MHI_CHANNEL_CONFIG_UL(12, "MBIM", 32, 0),
	MHI_CHANNEL_CONFIG_DL(13, "MBIM", 32, 0),
	MHI_CHANNEL_CONFIG_UL(32, "DUN", 32, 0),
	MHI_CHANNEL_CONFIG_DL(33, "DUN", 32, 0),
	/* The EDL firmware is a flash-programmer exposing firehose protocol */
	MHI_CHANNEL_CONFIG_UL_FP(34, "FIREHOSE", 32, 0),
	MHI_CHANNEL_CONFIG_DL_FP(35, "FIREHOSE", 32, 0),
	MHI_CHANNEL_CONFIG_HW_UL(100, "IP_HW0_MBIM", 128, 2),
	MHI_CHANNEL_CONFIG_HW_DL(101, "IP_HW0_MBIM", 128, 3),
};

static struct mhi_event_config mhi_quectel_em1xx_events[] = {
	MHI_EVENT_CONFIG_CTRL(0, 128),
	MHI_EVENT_CONFIG_DATA(1, 128),
	MHI_EVENT_CONFIG_HW_DATA(2, 1024, 100),
	MHI_EVENT_CONFIG_HW_DATA(3, 1024, 101)
};

static const struct mhi_controller_config modem_quectel_em1xx_config = {
	.max_channels = 128,
	.timeout_ms = 20000,
	.num_channels = ARRAY_SIZE(mhi_quectel_em1xx_channels),
	.ch_cfg = mhi_quectel_em1xx_channels,
	.num_events = ARRAY_SIZE(mhi_quectel_em1xx_events),
	.event_cfg = mhi_quectel_em1xx_events,
};

static const struct mhi_pci_dev_info mhi_quectel_em1xx_info = {
	.name = "quectel-em1xx",
	.edl = "qcom/prog_firehose_sdx24.mbn",
	.config = &modem_quectel_em1xx_config,
	.bar_num = MHI_PCI_DEFAULT_BAR_NUM,
	.dma_data_width = 32,
	.mru_default = 32768,
	.sideband_wake = true,
};

static const struct mhi_pci_dev_info mhi_quectel_rm5xx_info = {
	.name = "quectel-rm5xx",
	.edl = "qcom/prog_firehose_sdx6x.elf",
	.config = &modem_quectel_em1xx_config,
	.bar_num = MHI_PCI_DEFAULT_BAR_NUM,
	.dma_data_width = 32,
	.mru_default = 32768,
	.sideband_wake = true,
};

static const struct mhi_channel_config mhi_foxconn_sdx55_channels[] = {
	MHI_CHANNEL_CONFIG_UL(0, "LOOPBACK", 32, 0),
	MHI_CHANNEL_CONFIG_DL(1, "LOOPBACK", 32, 0),
	MHI_CHANNEL_CONFIG_UL(4, "DIAG", 32, 1),
	MHI_CHANNEL_CONFIG_DL(5, "DIAG", 32, 1),
	MHI_CHANNEL_CONFIG_UL(12, "MBIM", 32, 0),
	MHI_CHANNEL_CONFIG_DL(13, "MBIM", 32, 0),
	MHI_CHANNEL_CONFIG_UL(32, "DUN", 32, 0),
	MHI_CHANNEL_CONFIG_DL(33, "DUN", 32, 0),
	MHI_CHANNEL_CONFIG_UL_FP(34, "FIREHOSE", 32, 0),
	MHI_CHANNEL_CONFIG_DL_FP(35, "FIREHOSE", 32, 0),
	MHI_CHANNEL_CONFIG_HW_UL(100, "IP_HW0_MBIM", 128, 2),
	MHI_CHANNEL_CONFIG_HW_DL(101, "IP_HW0_MBIM", 128, 3),
};

static const struct mhi_channel_config mhi_foxconn_sdx61_channels[] = {
	MHI_CHANNEL_CONFIG_UL(0, "LOOPBACK", 32, 0),
	MHI_CHANNEL_CONFIG_DL(1, "LOOPBACK", 32, 0),
	MHI_CHANNEL_CONFIG_UL(4, "DIAG", 32, 1),
	MHI_CHANNEL_CONFIG_DL(5, "DIAG", 32, 1),
	MHI_CHANNEL_CONFIG_UL(12, "MBIM", 32, 0),
	MHI_CHANNEL_CONFIG_DL(13, "MBIM", 32, 0),
	MHI_CHANNEL_CONFIG_UL(32, "DUN", 32, 0),
	MHI_CHANNEL_CONFIG_DL(33, "DUN", 32, 0),
	MHI_CHANNEL_CONFIG_UL_FP(34, "FIREHOSE", 32, 0),
	MHI_CHANNEL_CONFIG_DL_FP(35, "FIREHOSE", 32, 0),
	MHI_CHANNEL_CONFIG_UL(50, "NMEA", 32, 0),
	MHI_CHANNEL_CONFIG_DL(51, "NMEA", 32, 0),
	MHI_CHANNEL_CONFIG_HW_UL(100, "IP_HW0_MBIM", 128, 2),
	MHI_CHANNEL_CONFIG_HW_DL(101, "IP_HW0_MBIM", 128, 3),
};

static struct mhi_event_config mhi_foxconn_sdx55_events[] = {
	MHI_EVENT_CONFIG_CTRL(0, 128),
	MHI_EVENT_CONFIG_DATA(1, 128),
	MHI_EVENT_CONFIG_HW_DATA(2, 1024, 100),
	MHI_EVENT_CONFIG_HW_DATA(3, 1024, 101)
};

static const struct mhi_controller_config modem_foxconn_sdx55_config = {
	.max_channels = 128,
	.timeout_ms = 20000,
	.num_channels = ARRAY_SIZE(mhi_foxconn_sdx55_channels),
	.ch_cfg = mhi_foxconn_sdx55_channels,
	.num_events = ARRAY_SIZE(mhi_foxconn_sdx55_events),
	.event_cfg = mhi_foxconn_sdx55_events,
};

static const struct mhi_controller_config modem_foxconn_sdx61_config = {
	.max_channels = 128,
	.timeout_ms = 20000,
	.num_channels = ARRAY_SIZE(mhi_foxconn_sdx61_channels),
	.ch_cfg = mhi_foxconn_sdx61_channels,
	.num_events = ARRAY_SIZE(mhi_foxconn_sdx55_events),
	.event_cfg = mhi_foxconn_sdx55_events,
};

static const struct mhi_controller_config modem_foxconn_sdx72_config = {
	.max_channels = 128,
	.timeout_ms = 20000,
	.ready_timeout_ms = 50000,
	.num_channels = ARRAY_SIZE(mhi_foxconn_sdx55_channels),
	.ch_cfg = mhi_foxconn_sdx55_channels,
	.num_events = ARRAY_SIZE(mhi_foxconn_sdx55_events),
	.event_cfg = mhi_foxconn_sdx55_events,
};

static const struct mhi_pci_dev_info mhi_foxconn_sdx55_info = {
	.name = "foxconn-sdx55",
	.edl = "qcom/sdx55m/foxconn/prog_firehose_sdx55.mbn",
	.edl_trigger = true,
	.config = &modem_foxconn_sdx55_config,
	.bar_num = MHI_PCI_DEFAULT_BAR_NUM,
	.dma_data_width = 32,
	.mru_default = 32768,
	.sideband_wake = false,
};

static const struct mhi_pci_dev_info mhi_foxconn_t99w175_info = {
	.name = "foxconn-t99w175",
	.edl = "qcom/sdx55m/foxconn/prog_firehose_sdx55.mbn",
	.edl_trigger = true,
	.config = &modem_foxconn_sdx55_config,
	.bar_num = MHI_PCI_DEFAULT_BAR_NUM,
	.dma_data_width = 32,
	.mru_default = 32768,
	.sideband_wake = false,
};

static const struct mhi_pci_dev_info mhi_foxconn_dw5930e_info = {
	.name = "foxconn-dw5930e",
	.edl = "qcom/sdx55m/foxconn/prog_firehose_sdx55.mbn",
	.edl_trigger = true,
	.config = &modem_foxconn_sdx55_config,
	.bar_num = MHI_PCI_DEFAULT_BAR_NUM,
	.dma_data_width = 32,
	.mru_default = 32768,
	.sideband_wake = false,
};

static const struct mhi_pci_dev_info mhi_foxconn_t99w368_info = {
	.name = "foxconn-t99w368",
	.edl = "qcom/sdx65m/foxconn/prog_firehose_lite.elf",
	.edl_trigger = true,
	.config = &modem_foxconn_sdx55_config,
	.bar_num = MHI_PCI_DEFAULT_BAR_NUM,
	.dma_data_width = 32,
	.mru_default = 32768,
	.sideband_wake = false,
};

static const struct mhi_pci_dev_info mhi_foxconn_t99w373_info = {
	.name = "foxconn-t99w373",
	.edl = "qcom/sdx65m/foxconn/prog_firehose_lite.elf",
	.edl_trigger = true,
	.config = &modem_foxconn_sdx55_config,
	.bar_num = MHI_PCI_DEFAULT_BAR_NUM,
	.dma_data_width = 32,
	.mru_default = 32768,
	.sideband_wake = false,
};

static const struct mhi_pci_dev_info mhi_foxconn_t99w510_info = {
	.name = "foxconn-t99w510",
	.edl = "qcom/sdx24m/foxconn/prog_firehose_sdx24.mbn",
	.edl_trigger = true,
	.config = &modem_foxconn_sdx55_config,
	.bar_num = MHI_PCI_DEFAULT_BAR_NUM,
	.dma_data_width = 32,
	.mru_default = 32768,
	.sideband_wake = false,
};

static const struct mhi_pci_dev_info mhi_foxconn_dw5932e_info = {
	.name = "foxconn-dw5932e",
	.edl = "qcom/sdx65m/foxconn/prog_firehose_lite.elf",
	.edl_trigger = true,
	.config = &modem_foxconn_sdx55_config,
	.bar_num = MHI_PCI_DEFAULT_BAR_NUM,
	.dma_data_width = 32,
	.mru_default = 32768,
	.sideband_wake = false,
};

static const struct mhi_pci_dev_info mhi_foxconn_t99w640_info = {
	.name = "foxconn-t99w640",
	.edl = "qcom/sdx72m/foxconn/edl.mbn",
	.edl_trigger = true,
	.config = &modem_foxconn_sdx72_config,
	.bar_num = MHI_PCI_DEFAULT_BAR_NUM,
	.dma_data_width = 32,
	.mru_default = 32768,
	.sideband_wake = false,
};

static const struct mhi_pci_dev_info mhi_foxconn_dw5934e_info = {
	.name = "foxconn-dw5934e",
	.edl = "qcom/sdx72m/foxconn/edl.mbn",
	.edl_trigger = true,
	.config = &modem_foxconn_sdx72_config,
	.bar_num = MHI_PCI_DEFAULT_BAR_NUM,
	.dma_data_width = 32,
	.mru_default = 32768,
	.sideband_wake = false,
};

static const struct mhi_pci_dev_info mhi_foxconn_t99w696_info = {
	.name = "foxconn-t99w696",
	.edl = "qcom/sdx61/foxconn/prog_firehose_lite.elf",
	.edl_trigger = true,
	.config = &modem_foxconn_sdx61_config,
	.bar_num = MHI_PCI_DEFAULT_BAR_NUM,
	.dma_data_width = 32,
	.mru_default = 32768,
	.sideband_wake = false,
};

static const struct mhi_channel_config mhi_mv3x_channels[] = {
	MHI_CHANNEL_CONFIG_UL(0, "LOOPBACK", 64, 0),
	MHI_CHANNEL_CONFIG_DL(1, "LOOPBACK", 64, 0),
	/* MBIM Control Channel */
	MHI_CHANNEL_CONFIG_UL(12, "MBIM", 64, 0),
	MHI_CHANNEL_CONFIG_DL(13, "MBIM", 64, 0),
	/* MBIM Data Channel */
	MHI_CHANNEL_CONFIG_HW_UL(100, "IP_HW0_MBIM", 512, 2),
	MHI_CHANNEL_CONFIG_HW_DL(101, "IP_HW0_MBIM", 512, 3),
};

static struct mhi_event_config mhi_mv3x_events[] = {
	MHI_EVENT_CONFIG_CTRL(0, 256),
	MHI_EVENT_CONFIG_DATA(1, 256),
	MHI_EVENT_CONFIG_HW_DATA(2, 1024, 100),
	MHI_EVENT_CONFIG_HW_DATA(3, 1024, 101),
};

static const struct mhi_controller_config modem_mv3x_config = {
	.max_channels = 128,
	.timeout_ms = 20000,
	.num_channels = ARRAY_SIZE(mhi_mv3x_channels),
	.ch_cfg = mhi_mv3x_channels,
	.num_events = ARRAY_SIZE(mhi_mv3x_events),
	.event_cfg = mhi_mv3x_events,
};

static const struct mhi_pci_dev_info mhi_mv31_info = {
	.name = "cinterion-mv31",
	.config = &modem_mv3x_config,
	.bar_num = MHI_PCI_DEFAULT_BAR_NUM,
	.dma_data_width = 32,
	.mru_default = 32768,
};

static const struct mhi_pci_dev_info mhi_mv32_info = {
	.name = "cinterion-mv32",
	.config = &modem_mv3x_config,
	.bar_num = MHI_PCI_DEFAULT_BAR_NUM,
	.dma_data_width = 32,
	.mru_default = 32768,
};

static const struct mhi_channel_config mhi_sierra_em919x_channels[] = {
	MHI_CHANNEL_CONFIG_UL_SBL(2, "SAHARA", 32, 0),
	MHI_CHANNEL_CONFIG_DL_SBL(3, "SAHARA", 256, 0),
	MHI_CHANNEL_CONFIG_UL(4, "DIAG", 32, 0),
	MHI_CHANNEL_CONFIG_DL(5, "DIAG", 32, 0),
	MHI_CHANNEL_CONFIG_UL(12, "MBIM", 128, 0),
	MHI_CHANNEL_CONFIG_DL(13, "MBIM", 128, 0),
	MHI_CHANNEL_CONFIG_UL(14, "QMI", 32, 0),
	MHI_CHANNEL_CONFIG_DL(15, "QMI", 32, 0),
	MHI_CHANNEL_CONFIG_UL(32, "DUN", 32, 0),
	MHI_CHANNEL_CONFIG_DL(33, "DUN", 32, 0),
	MHI_CHANNEL_CONFIG_HW_UL(100, "IP_HW0", 512, 1),
	MHI_CHANNEL_CONFIG_HW_DL(101, "IP_HW0", 512, 2),
};

static struct mhi_event_config modem_sierra_em919x_mhi_events[] = {
	/* first ring is control+data and DIAG ring */
	MHI_EVENT_CONFIG_CTRL(0, 2048),
	/* Hardware channels request dedicated hardware event rings */
	MHI_EVENT_CONFIG_HW_DATA(1, 2048, 100),
	MHI_EVENT_CONFIG_HW_DATA(2, 2048, 101)
};

static const struct mhi_controller_config modem_sierra_em919x_config = {
	.max_channels = 128,
	.timeout_ms = 24000,
	.num_channels = ARRAY_SIZE(mhi_sierra_em919x_channels),
	.ch_cfg = mhi_sierra_em919x_channels,
	.num_events = ARRAY_SIZE(modem_sierra_em919x_mhi_events),
	.event_cfg = modem_sierra_em919x_mhi_events,
};

static const struct mhi_pci_dev_info mhi_sierra_em919x_info = {
	.name = "sierra-em919x",
	.config = &modem_sierra_em919x_config,
	.bar_num = MHI_PCI_DEFAULT_BAR_NUM,
	.dma_data_width = 32,
	.mru_default = 32768,
	.sideband_wake = false,
};

static const struct mhi_channel_config mhi_telit_fn980_hw_v1_channels[] = {
	MHI_CHANNEL_CONFIG_UL(14, "QMI", 32, 0),
	MHI_CHANNEL_CONFIG_DL(15, "QMI", 32, 0),
	MHI_CHANNEL_CONFIG_UL(20, "IPCR", 16, 0),
	MHI_CHANNEL_CONFIG_DL_AUTOQUEUE(21, "IPCR", 16, 0),
	MHI_CHANNEL_CONFIG_HW_UL(100, "IP_HW0", 128, 1),
	MHI_CHANNEL_CONFIG_HW_DL(101, "IP_HW0", 128, 2),
};

static struct mhi_event_config mhi_telit_fn980_hw_v1_events[] = {
	MHI_EVENT_CONFIG_CTRL(0, 128),
	MHI_EVENT_CONFIG_HW_DATA(1, 1024, 100),
	MHI_EVENT_CONFIG_HW_DATA(2, 2048, 101)
};

static const struct mhi_controller_config modem_telit_fn980_hw_v1_config = {
	.max_channels = 128,
	.timeout_ms = 20000,
	.num_channels = ARRAY_SIZE(mhi_telit_fn980_hw_v1_channels),
	.ch_cfg = mhi_telit_fn980_hw_v1_channels,
	.num_events = ARRAY_SIZE(mhi_telit_fn980_hw_v1_events),
	.event_cfg = mhi_telit_fn980_hw_v1_events,
};

static const struct mhi_pci_dev_info mhi_telit_fn980_hw_v1_info = {
	.name = "telit-fn980-hwv1",
	.fw = "qcom/sdx55m/sbl1.mbn",
	.edl = "qcom/sdx55m/edl.mbn",
	.config = &modem_telit_fn980_hw_v1_config,
	.bar_num = MHI_PCI_DEFAULT_BAR_NUM,
	.dma_data_width = 32,
	.mru_default = 32768,
	.sideband_wake = false,
};

static const struct mhi_channel_config mhi_telit_fn990_channels[] = {
	MHI_CHANNEL_CONFIG_UL_SBL(2, "SAHARA", 32, 0),
	MHI_CHANNEL_CONFIG_DL_SBL(3, "SAHARA", 32, 0),
	MHI_CHANNEL_CONFIG_UL(4, "DIAG", 64, 1),
	MHI_CHANNEL_CONFIG_DL(5, "DIAG", 64, 1),
	MHI_CHANNEL_CONFIG_UL(12, "MBIM", 32, 0),
	MHI_CHANNEL_CONFIG_DL(13, "MBIM", 32, 0),
	MHI_CHANNEL_CONFIG_UL(32, "DUN", 32, 0),
	MHI_CHANNEL_CONFIG_DL(33, "DUN", 32, 0),
	MHI_CHANNEL_CONFIG_UL(92, "DUN2", 32, 1),
	MHI_CHANNEL_CONFIG_DL(93, "DUN2", 32, 1),
	MHI_CHANNEL_CONFIG_HW_UL(100, "IP_HW0_MBIM", 128, 2),
	MHI_CHANNEL_CONFIG_HW_DL(101, "IP_HW0_MBIM", 128, 3),
};

static struct mhi_event_config mhi_telit_fn990_events[] = {
	MHI_EVENT_CONFIG_CTRL(0, 128),
	MHI_EVENT_CONFIG_DATA(1, 128),
	MHI_EVENT_CONFIG_HW_DATA(2, 1024, 100),
	MHI_EVENT_CONFIG_HW_DATA(3, 2048, 101)
};

static const struct mhi_controller_config modem_telit_fn990_config = {
	.max_channels = 128,
	.timeout_ms = 20000,
	.num_channels = ARRAY_SIZE(mhi_telit_fn990_channels),
	.ch_cfg = mhi_telit_fn990_channels,
	.num_events = ARRAY_SIZE(mhi_telit_fn990_events),
	.event_cfg = mhi_telit_fn990_events,
};

static const struct mhi_pci_dev_info mhi_telit_fn990_info = {
	.name = "telit-fn990",
	.config = &modem_telit_fn990_config,
	.bar_num = MHI_PCI_DEFAULT_BAR_NUM,
	.dma_data_width = 32,
	.sideband_wake = false,
	.mru_default = 32768,
};

static const struct mhi_pci_dev_info mhi_telit_fe990a_info = {
	.name = "telit-fe990a",
	.config = &modem_telit_fn990_config,
	.bar_num = MHI_PCI_DEFAULT_BAR_NUM,
	.dma_data_width = 32,
	.sideband_wake = false,
	.mru_default = 32768,
};

static const struct mhi_channel_config mhi_telit_fn920c04_channels[] = {
	MHI_CHANNEL_CONFIG_UL_SBL(2, "SAHARA", 32, 0),
	MHI_CHANNEL_CONFIG_DL_SBL(3, "SAHARA", 32, 0),
	MHI_CHANNEL_CONFIG_UL(4, "DIAG", 64, 1),
	MHI_CHANNEL_CONFIG_DL(5, "DIAG", 64, 1),
	MHI_CHANNEL_CONFIG_UL(14, "QMI", 32, 0),
	MHI_CHANNEL_CONFIG_DL(15, "QMI", 32, 0),
	MHI_CHANNEL_CONFIG_UL(32, "DUN", 32, 0),
	MHI_CHANNEL_CONFIG_DL(33, "DUN", 32, 0),
	MHI_CHANNEL_CONFIG_UL_FP(34, "FIREHOSE", 32, 0),
	MHI_CHANNEL_CONFIG_DL_FP(35, "FIREHOSE", 32, 0),
	MHI_CHANNEL_CONFIG_UL(92, "DUN2", 32, 1),
	MHI_CHANNEL_CONFIG_DL(93, "DUN2", 32, 1),
	MHI_CHANNEL_CONFIG_HW_UL(100, "IP_HW0", 128, 2),
	MHI_CHANNEL_CONFIG_HW_DL(101, "IP_HW0", 128, 3),
};

static const struct mhi_controller_config modem_telit_fn920c04_config = {
	.max_channels = 128,
	.timeout_ms = 50000,
	.num_channels = ARRAY_SIZE(mhi_telit_fn920c04_channels),
	.ch_cfg = mhi_telit_fn920c04_channels,
	.num_events = ARRAY_SIZE(mhi_telit_fn990_events),
	.event_cfg = mhi_telit_fn990_events,
};

static const struct mhi_pci_dev_info mhi_telit_fn920c04_info = {
	.name = "telit-fn920c04",
	.config = &modem_telit_fn920c04_config,
	.bar_num = MHI_PCI_DEFAULT_BAR_NUM,
	.dma_data_width = 32,
	.sideband_wake = false,
	.mru_default = 32768,
	.edl_trigger = true,
};

static const struct mhi_pci_dev_info mhi_telit_fn990b40_info = {
	.name = "telit-fn990b40",
	.config = &modem_telit_fn920c04_config,
	.bar_num = MHI_PCI_DEFAULT_BAR_NUM,
	.dma_data_width = 32,
	.sideband_wake = false,
	.mru_default = 32768,
	.edl_trigger = true,
};

static const struct mhi_pci_dev_info mhi_telit_fe990b40_info = {
	.name = "telit-fe990b40",
	.config = &modem_telit_fn920c04_config,
	.bar_num = MHI_PCI_DEFAULT_BAR_NUM,
	.dma_data_width = 32,
	.sideband_wake = false,
	.mru_default = 32768,
	.edl_trigger = true,
};

static const struct mhi_pci_dev_info mhi_netprisma_lcur57_info = {
	.name = "netprisma-lcur57",
	.edl = "qcom/prog_firehose_sdx24.mbn",
	.config = &modem_quectel_em1xx_config,
	.bar_num = MHI_PCI_DEFAULT_BAR_NUM,
	.dma_data_width = 32,
	.mru_default = 32768,
	.sideband_wake = true,
};

static const struct mhi_pci_dev_info mhi_netprisma_fcun69_info = {
	.name = "netprisma-fcun69",
	.edl = "qcom/prog_firehose_sdx6x.elf",
	.config = &modem_quectel_em1xx_config,
	.bar_num = MHI_PCI_DEFAULT_BAR_NUM,
	.dma_data_width = 32,
	.mru_default = 32768,
	.sideband_wake = true,
};

/* Keep the list sorted based on the PID. New VID should be added as the last entry */
static const struct pci_device_id mhi_pci_id_table[] = {
	{PCI_DEVICE(PCI_VENDOR_ID_QCOM, 0x0116),
		.driver_data = (kernel_ulong_t) &mhi_qcom_sa8775p_info },
	/* Telit FN920C04 (sdx35) */
	{PCI_DEVICE_SUB(PCI_VENDOR_ID_QCOM, 0x011a, 0x1c5d, 0x2020),
		.driver_data = (kernel_ulong_t) &mhi_telit_fn920c04_info },
	{ PCI_DEVICE(PCI_VENDOR_ID_QCOM, 0x0304),
		.driver_data = (kernel_ulong_t) &mhi_qcom_sdx24_info },
	{ PCI_DEVICE_SUB(PCI_VENDOR_ID_QCOM, 0x0306, PCI_VENDOR_ID_QCOM, 0x010c),
		.driver_data = (kernel_ulong_t) &mhi_foxconn_sdx55_info },
	/* EM919x (sdx55), use the same vid:pid as qcom-sdx55m */
	{ PCI_DEVICE_SUB(PCI_VENDOR_ID_QCOM, 0x0306, 0x18d7, 0x0200),
		.driver_data = (kernel_ulong_t) &mhi_sierra_em919x_info },
	/* EM929x (sdx65), use the same configuration as EM919x */
	{ PCI_DEVICE_SUB(PCI_VENDOR_ID_QCOM, 0x0308, 0x18d7, 0x0301),
		.driver_data = (kernel_ulong_t) &mhi_sierra_em919x_info },
	/* Telit FN980 hardware revision v1 */
	{ PCI_DEVICE_SUB(PCI_VENDOR_ID_QCOM, 0x0306, 0x1C5D, 0x2000),
		.driver_data = (kernel_ulong_t) &mhi_telit_fn980_hw_v1_info },
	{ PCI_DEVICE(PCI_VENDOR_ID_QCOM, 0x0306),
		.driver_data = (kernel_ulong_t) &mhi_qcom_sdx55_info },
	/* Telit FN990 */
	{ PCI_DEVICE_SUB(PCI_VENDOR_ID_QCOM, 0x0308, 0x1c5d, 0x2010),
		.driver_data = (kernel_ulong_t) &mhi_telit_fn990_info },
	/* Telit FE990A */
	{ PCI_DEVICE_SUB(PCI_VENDOR_ID_QCOM, 0x0308, 0x1c5d, 0x2015),
		.driver_data = (kernel_ulong_t) &mhi_telit_fe990a_info },
	/* Foxconn T99W696, all variants */
	{ PCI_DEVICE_SUB(PCI_VENDOR_ID_QCOM, 0x0308, PCI_VENDOR_ID_FOXCONN, PCI_ANY_ID),
		.driver_data = (kernel_ulong_t) &mhi_foxconn_t99w696_info },
	{ PCI_DEVICE(PCI_VENDOR_ID_QCOM, 0x0308),
		.driver_data = (kernel_ulong_t) &mhi_qcom_sdx65_info },
	/* Telit FN990B40 (sdx72) */
	{ PCI_DEVICE_SUB(PCI_VENDOR_ID_QCOM, 0x0309, 0x1c5d, 0x201a),
		.driver_data = (kernel_ulong_t) &mhi_telit_fn990b40_info },
	/* Telit FE990B40 (sdx72) */
	{ PCI_DEVICE_SUB(PCI_VENDOR_ID_QCOM, 0x0309, 0x1c5d, 0x2025),
		.driver_data = (kernel_ulong_t) &mhi_telit_fe990b40_info },
	{ PCI_DEVICE(PCI_VENDOR_ID_QCOM, 0x0309),
		.driver_data = (kernel_ulong_t) &mhi_qcom_sdx75_info },
	/* QDU100, x100-DU */
	{ PCI_DEVICE(PCI_VENDOR_ID_QCOM, 0x0601),
		.driver_data = (kernel_ulong_t) &mhi_qcom_qdu100_info },
	{ PCI_DEVICE(PCI_VENDOR_ID_QUECTEL, 0x1001), /* EM120R-GL (sdx24) */
		.driver_data = (kernel_ulong_t) &mhi_quectel_em1xx_info },
	{ PCI_DEVICE(PCI_VENDOR_ID_QUECTEL, 0x1002), /* EM160R-GL (sdx24) */
		.driver_data = (kernel_ulong_t) &mhi_quectel_em1xx_info },
	/* RM520N-GL (sdx6x), eSIM */
	{ PCI_DEVICE(PCI_VENDOR_ID_QUECTEL, 0x1004),
		.driver_data = (kernel_ulong_t) &mhi_quectel_rm5xx_info },
	/* RM520N-GL (sdx6x), Lenovo variant */
	{ PCI_DEVICE(PCI_VENDOR_ID_QUECTEL, 0x1007),
		.driver_data = (kernel_ulong_t) &mhi_quectel_rm5xx_info },
	{ PCI_DEVICE(PCI_VENDOR_ID_QUECTEL, 0x100d), /* EM160R-GL (sdx24) */
		.driver_data = (kernel_ulong_t) &mhi_quectel_em1xx_info },
	{ PCI_DEVICE(PCI_VENDOR_ID_QUECTEL, 0x2001), /* EM120R-GL for FCCL (sdx24) */
		.driver_data = (kernel_ulong_t) &mhi_quectel_em1xx_info },
	/* T99W175 (sdx55), Both for eSIM and Non-eSIM */
	{ PCI_DEVICE(PCI_VENDOR_ID_FOXCONN, 0xe0ab),
		.driver_data = (kernel_ulong_t) &mhi_foxconn_t99w175_info },
	/* DW5930e (sdx55), With eSIM, It's also T99W175 */
	{ PCI_DEVICE(PCI_VENDOR_ID_FOXCONN, 0xe0b0),
		.driver_data = (kernel_ulong_t) &mhi_foxconn_dw5930e_info },
	/* DW5930e (sdx55), Non-eSIM, It's also T99W175 */
	{ PCI_DEVICE(PCI_VENDOR_ID_FOXCONN, 0xe0b1),
		.driver_data = (kernel_ulong_t) &mhi_foxconn_dw5930e_info },
	/* T99W175 (sdx55), Based on Qualcomm new baseline */
	{ PCI_DEVICE(PCI_VENDOR_ID_FOXCONN, 0xe0bf),
		.driver_data = (kernel_ulong_t) &mhi_foxconn_t99w175_info },
	/* T99W175 (sdx55) */
	{ PCI_DEVICE(PCI_VENDOR_ID_FOXCONN, 0xe0c3),
		.driver_data = (kernel_ulong_t) &mhi_foxconn_t99w175_info },
	/* T99W368 (sdx65) */
	{ PCI_DEVICE(PCI_VENDOR_ID_FOXCONN, 0xe0d8),
		.driver_data = (kernel_ulong_t) &mhi_foxconn_t99w368_info },
	/* T99W373 (sdx62) */
	{ PCI_DEVICE(PCI_VENDOR_ID_FOXCONN, 0xe0d9),
		.driver_data = (kernel_ulong_t) &mhi_foxconn_t99w373_info },
	/* T99W510 (sdx24), variant 1 */
	{ PCI_DEVICE(PCI_VENDOR_ID_FOXCONN, 0xe0f0),
		.driver_data = (kernel_ulong_t) &mhi_foxconn_t99w510_info },
	/* T99W510 (sdx24), variant 2 */
	{ PCI_DEVICE(PCI_VENDOR_ID_FOXCONN, 0xe0f1),
		.driver_data = (kernel_ulong_t) &mhi_foxconn_t99w510_info },
	/* T99W510 (sdx24), variant 3 */
	{ PCI_DEVICE(PCI_VENDOR_ID_FOXCONN, 0xe0f2),
		.driver_data = (kernel_ulong_t) &mhi_foxconn_t99w510_info },
	/* DW5932e-eSIM (sdx62), With eSIM */
	{ PCI_DEVICE(PCI_VENDOR_ID_FOXCONN, 0xe0f5),
		.driver_data = (kernel_ulong_t) &mhi_foxconn_dw5932e_info },
	/* DW5932e (sdx62), Non-eSIM */
	{ PCI_DEVICE(PCI_VENDOR_ID_FOXCONN, 0xe0f9),
		.driver_data = (kernel_ulong_t) &mhi_foxconn_dw5932e_info },
	/* T99W640 (sdx72) */
	{ PCI_DEVICE(PCI_VENDOR_ID_FOXCONN, 0xe118),
		.driver_data = (kernel_ulong_t) &mhi_foxconn_t99w640_info },
	/* DW5934e(sdx72), With eSIM */
	{ PCI_DEVICE(PCI_VENDOR_ID_FOXCONN, 0xe11d),
		.driver_data = (kernel_ulong_t) &mhi_foxconn_dw5934e_info },
	/* DW5934e(sdx72), Non-eSIM */
	{ PCI_DEVICE(PCI_VENDOR_ID_FOXCONN, 0xe11e),
		.driver_data = (kernel_ulong_t) &mhi_foxconn_dw5934e_info },
	/* MV31-W (Cinterion) */
	{ PCI_DEVICE(PCI_VENDOR_ID_THALES, 0x00b3),
		.driver_data = (kernel_ulong_t) &mhi_mv31_info },
	/* MV31-W (Cinterion), based on new baseline */
	{ PCI_DEVICE(PCI_VENDOR_ID_THALES, 0x00b4),
		.driver_data = (kernel_ulong_t) &mhi_mv31_info },
	/* MV32-WA (Cinterion) */
	{ PCI_DEVICE(PCI_VENDOR_ID_THALES, 0x00ba),
		.driver_data = (kernel_ulong_t) &mhi_mv32_info },
	/* MV32-WB (Cinterion) */
	{ PCI_DEVICE(PCI_VENDOR_ID_THALES, 0x00bb),
		.driver_data = (kernel_ulong_t) &mhi_mv32_info },
	/* T99W175 (sdx55), HP variant */
	{ PCI_DEVICE(0x03f0, 0x0a6c),
		.driver_data = (kernel_ulong_t) &mhi_foxconn_t99w175_info },
	/* NETPRISMA LCUR57 (SDX24) */
	{ PCI_DEVICE(PCI_VENDOR_ID_NETPRISMA, 0x1000),
		.driver_data = (kernel_ulong_t) &mhi_netprisma_lcur57_info },
	/* NETPRISMA FCUN69 (SDX6X) */
	{ PCI_DEVICE(PCI_VENDOR_ID_NETPRISMA, 0x1001),
		.driver_data = (kernel_ulong_t) &mhi_netprisma_fcun69_info },
	{  }
};
MODULE_DEVICE_TABLE(pci, mhi_pci_id_table);

enum mhi_pci_device_status {
	MHI_PCI_DEV_STARTED,
	MHI_PCI_DEV_SUSPENDED,
};

struct mhi_pci_device {
	struct mhi_controller mhi_cntrl;
	struct pci_saved_state *pci_state;
	struct work_struct recovery_work;
	struct timer_list health_check_timer;
	unsigned long status;
	bool reset_on_remove;
};

static int mhi_pci_read_reg(struct mhi_controller *mhi_cntrl,
			    void __iomem *addr, u32 *out)
{
	*out = readl(addr);
	return 0;
}

static void mhi_pci_write_reg(struct mhi_controller *mhi_cntrl,
			      void __iomem *addr, u32 val)
{
	writel(val, addr);
}

static void mhi_pci_status_cb(struct mhi_controller *mhi_cntrl,
			      enum mhi_callback cb)
{
	struct pci_dev *pdev = to_pci_dev(mhi_cntrl->cntrl_dev);

	/* Nothing to do for now */
	switch (cb) {
	case MHI_CB_FATAL_ERROR:
	case MHI_CB_SYS_ERROR:
		dev_warn(&pdev->dev, "firmware crashed (%u)\n", cb);
		pm_runtime_forbid(&pdev->dev);
		break;
	case MHI_CB_EE_MISSION_MODE:
		pm_runtime_allow(&pdev->dev);
		break;
	default:
		break;
	}
}

static void mhi_pci_wake_get_nop(struct mhi_controller *mhi_cntrl, bool force)
{
	/* no-op */
}

static void mhi_pci_wake_put_nop(struct mhi_controller *mhi_cntrl, bool override)
{
	/* no-op */
}

static void mhi_pci_wake_toggle_nop(struct mhi_controller *mhi_cntrl)
{
	/* no-op */
}

static bool mhi_pci_is_alive(struct mhi_controller *mhi_cntrl)
{
	struct pci_dev *pdev = to_pci_dev(mhi_cntrl->cntrl_dev);
	u16 vendor = 0;

	if (pci_read_config_word(pci_physfn(pdev), PCI_VENDOR_ID, &vendor))
		return false;

	if (vendor == (u16) ~0 || vendor == 0)
		return false;

	return true;
}

static int mhi_pci_claim(struct mhi_controller *mhi_cntrl,
			 unsigned int bar_num, u64 dma_mask)
{
	struct pci_dev *pdev = to_pci_dev(mhi_cntrl->cntrl_dev);
	int err;

	err = pcim_enable_device(pdev);
	if (err) {
		dev_err(&pdev->dev, "failed to enable pci device: %d\n", err);
		return err;
	}

	mhi_cntrl->regs = pcim_iomap_region(pdev, bar_num, pci_name(pdev));
	if (IS_ERR(mhi_cntrl->regs)) {
		err = PTR_ERR(mhi_cntrl->regs);
		dev_err(&pdev->dev, "failed to map pci region: %d\n", err);
		return err;
	}
	mhi_cntrl->reg_len = pci_resource_len(pdev, bar_num);

	err = dma_set_mask_and_coherent(&pdev->dev, dma_mask);
	if (err) {
		dev_err(&pdev->dev, "Cannot set proper DMA mask\n");
		return err;
	}

	pci_set_master(pdev);

	return 0;
}

static int mhi_pci_get_irqs(struct mhi_controller *mhi_cntrl,
			    const struct mhi_controller_config *mhi_cntrl_config)
{
	struct pci_dev *pdev = to_pci_dev(mhi_cntrl->cntrl_dev);
	int nr_vectors, i;
	int *irq;

	/*
	 * Alloc one MSI vector for BHI + one vector per event ring, ideally...
	 * No explicit pci_free_irq_vectors required, done by pcim_release.
	 */
	mhi_cntrl->nr_irqs = 1 + mhi_cntrl_config->num_events;

	nr_vectors = pci_alloc_irq_vectors(pdev, 1, mhi_cntrl->nr_irqs, PCI_IRQ_MSIX | PCI_IRQ_MSI);
	if (nr_vectors < 0) {
		dev_err(&pdev->dev, "Error allocating MSI vectors %d\n",
			nr_vectors);
		return nr_vectors;
	}

	if (nr_vectors < mhi_cntrl->nr_irqs) {
		dev_warn(&pdev->dev, "using shared MSI\n");

		/* Patch msi vectors, use only one (shared) */
		for (i = 0; i < mhi_cntrl_config->num_events; i++)
			mhi_cntrl_config->event_cfg[i].irq = 0;
		mhi_cntrl->nr_irqs = 1;
	}

	irq = devm_kcalloc(&pdev->dev, mhi_cntrl->nr_irqs, sizeof(int), GFP_KERNEL);
	if (!irq)
		return -ENOMEM;

	for (i = 0; i < mhi_cntrl->nr_irqs; i++) {
		int vector = i >= nr_vectors ? (nr_vectors - 1) : i;

		irq[i] = pci_irq_vector(pdev, vector);
	}

	mhi_cntrl->irq = irq;

	return 0;
}

static int mhi_pci_runtime_get(struct mhi_controller *mhi_cntrl)
{
	/* The runtime_get() MHI callback means:
	 *    Do whatever is requested to leave M3.
	 */
	return pm_runtime_get(mhi_cntrl->cntrl_dev);
}

static void mhi_pci_runtime_put(struct mhi_controller *mhi_cntrl)
{
	/* The runtime_put() MHI callback means:
	 *    Device can be moved in M3 state.
	 */
	pm_runtime_mark_last_busy(mhi_cntrl->cntrl_dev);
	pm_runtime_put(mhi_cntrl->cntrl_dev);
}

static void mhi_pci_recovery_work(struct work_struct *work)
{
	struct mhi_pci_device *mhi_pdev = container_of(work, struct mhi_pci_device,
						       recovery_work);
	struct mhi_controller *mhi_cntrl = &mhi_pdev->mhi_cntrl;
	struct pci_dev *pdev = to_pci_dev(mhi_cntrl->cntrl_dev);
	int err;

	dev_warn(&pdev->dev, "device recovery started\n");

	if (pdev->is_physfn)
		timer_delete(&mhi_pdev->health_check_timer);

	pm_runtime_forbid(&pdev->dev);

	/* Clean up MHI state */
	if (test_and_clear_bit(MHI_PCI_DEV_STARTED, &mhi_pdev->status)) {
		mhi_power_down(mhi_cntrl, false);
		mhi_unprepare_after_power_down(mhi_cntrl);
	}

	pci_set_power_state(pdev, PCI_D0);
	pci_load_saved_state(pdev, mhi_pdev->pci_state);
	pci_restore_state(pdev);

	if (!mhi_pci_is_alive(mhi_cntrl))
		goto err_try_reset;

	err = mhi_prepare_for_power_up(mhi_cntrl);
	if (err)
		goto err_try_reset;

	err = mhi_sync_power_up(mhi_cntrl);
	if (err)
		goto err_unprepare;

	dev_dbg(&pdev->dev, "Recovery completed\n");

	set_bit(MHI_PCI_DEV_STARTED, &mhi_pdev->status);

	if (pdev->is_physfn)
		mod_timer(&mhi_pdev->health_check_timer, jiffies + HEALTH_CHECK_PERIOD);

	return;

err_unprepare:
	mhi_unprepare_after_power_down(mhi_cntrl);
err_try_reset:
	err = pci_try_reset_function(pdev);
	if (err)
		dev_err(&pdev->dev, "Recovery failed: %d\n", err);
}

static void health_check(struct timer_list *t)
{
	struct mhi_pci_device *mhi_pdev = timer_container_of(mhi_pdev, t,
							     health_check_timer);
	struct mhi_controller *mhi_cntrl = &mhi_pdev->mhi_cntrl;

	if (!test_bit(MHI_PCI_DEV_STARTED, &mhi_pdev->status) ||
			test_bit(MHI_PCI_DEV_SUSPENDED, &mhi_pdev->status))
		return;

	if (!mhi_pci_is_alive(mhi_cntrl)) {
		dev_err(mhi_cntrl->cntrl_dev, "Device died\n");
		queue_work(system_long_wq, &mhi_pdev->recovery_work);
		return;
	}

	/* reschedule in two seconds */
	mod_timer(&mhi_pdev->health_check_timer, jiffies + HEALTH_CHECK_PERIOD);
}

static int mhi_pci_generic_edl_trigger(struct mhi_controller *mhi_cntrl)
{
	void __iomem *base = mhi_cntrl->regs;
	void __iomem *edl_db;
	int ret;
	u32 val;

	ret = mhi_device_get_sync(mhi_cntrl->mhi_dev);
	if (ret) {
		dev_err(mhi_cntrl->cntrl_dev, "Failed to wakeup the device\n");
		return ret;
	}

	pm_wakeup_event(&mhi_cntrl->mhi_dev->dev, 0);
	mhi_cntrl->runtime_get(mhi_cntrl);

	ret = mhi_get_channel_doorbell_offset(mhi_cntrl, &val);
	if (ret)
		goto err_get_chdb;

	edl_db = base + val + (8 * MHI_EDL_DB);

	mhi_cntrl->write_reg(mhi_cntrl, edl_db + 4, upper_32_bits(MHI_EDL_COOKIE));
	mhi_cntrl->write_reg(mhi_cntrl, edl_db, lower_32_bits(MHI_EDL_COOKIE));

	mhi_soc_reset(mhi_cntrl);

err_get_chdb:
	mhi_cntrl->runtime_put(mhi_cntrl);
	mhi_device_put(mhi_cntrl->mhi_dev);

	return ret;
}

static int mhi_pci_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	const struct mhi_pci_dev_info *info = (struct mhi_pci_dev_info *) id->driver_data;
	const struct mhi_controller_config *mhi_cntrl_config;
	struct mhi_pci_device *mhi_pdev;
	struct mhi_controller *mhi_cntrl;
	unsigned int dma_data_width;
	int err;

	dev_info(&pdev->dev, "MHI PCI device found: %s\n", info->name);

	/* mhi_pdev.mhi_cntrl must be zero-initialized */
	mhi_pdev = devm_kzalloc(&pdev->dev, sizeof(*mhi_pdev), GFP_KERNEL);
	if (!mhi_pdev)
		return -ENOMEM;

	INIT_WORK(&mhi_pdev->recovery_work, mhi_pci_recovery_work);

	if (pdev->is_virtfn && info->vf_config)
		mhi_cntrl_config = info->vf_config;
	else
		mhi_cntrl_config = info->config;

	/* Initialize health check monitor only for Physical functions */
	if (pdev->is_physfn)
		timer_setup(&mhi_pdev->health_check_timer, health_check, 0);

	mhi_cntrl = &mhi_pdev->mhi_cntrl;

	dma_data_width = (pdev->is_virtfn && info->vf_dma_data_width) ?
			  info->vf_dma_data_width : info->dma_data_width;

	mhi_cntrl->cntrl_dev = &pdev->dev;
	mhi_cntrl->iova_start = 0;
	mhi_cntrl->iova_stop = (dma_addr_t)DMA_BIT_MASK(dma_data_width);
	mhi_cntrl->fw_image = info->fw;
	mhi_cntrl->edl_image = info->edl;

	mhi_cntrl->read_reg = mhi_pci_read_reg;
	mhi_cntrl->write_reg = mhi_pci_write_reg;
	mhi_cntrl->status_cb = mhi_pci_status_cb;
	mhi_cntrl->runtime_get = mhi_pci_runtime_get;
	mhi_cntrl->runtime_put = mhi_pci_runtime_put;
	mhi_cntrl->mru = info->mru_default;
	mhi_cntrl->name = info->name;

	if (pdev->is_physfn)
		mhi_pdev->reset_on_remove = info->reset_on_remove;

	if (info->edl_trigger)
		mhi_cntrl->edl_trigger = mhi_pci_generic_edl_trigger;

	if (info->sideband_wake) {
		mhi_cntrl->wake_get = mhi_pci_wake_get_nop;
		mhi_cntrl->wake_put = mhi_pci_wake_put_nop;
		mhi_cntrl->wake_toggle = mhi_pci_wake_toggle_nop;
	}

	err = mhi_pci_claim(mhi_cntrl, info->bar_num, DMA_BIT_MASK(dma_data_width));
	if (err)
		return err;

	err = mhi_pci_get_irqs(mhi_cntrl, mhi_cntrl_config);
	if (err)
		return err;

	pci_set_drvdata(pdev, mhi_pdev);

	/* Have stored pci confspace at hand for restore in sudden PCI error.
	 * cache the state locally and discard the PCI core one.
	 */
	pci_save_state(pdev);
	mhi_pdev->pci_state = pci_store_saved_state(pdev);
	pci_load_saved_state(pdev, NULL);

	err = mhi_register_controller(mhi_cntrl, mhi_cntrl_config);
	if (err)
		return err;

	/* MHI bus does not power up the controller by default */
	err = mhi_prepare_for_power_up(mhi_cntrl);
	if (err) {
		dev_err(&pdev->dev, "failed to prepare MHI controller\n");
		goto err_unregister;
	}

	err = mhi_sync_power_up(mhi_cntrl);
	if (err) {
		dev_err(&pdev->dev, "failed to power up MHI controller\n");
		goto err_unprepare;
	}

	set_bit(MHI_PCI_DEV_STARTED, &mhi_pdev->status);

	/* start health check */
	if (pdev->is_physfn)
		mod_timer(&mhi_pdev->health_check_timer, jiffies + HEALTH_CHECK_PERIOD);

	/* Allow runtime suspend only if both PME from D3Hot and M3 are supported */
	if (pci_pme_capable(pdev, PCI_D3hot) && !(info->no_m3)) {
		pm_runtime_set_autosuspend_delay(&pdev->dev, 2000);
		pm_runtime_use_autosuspend(&pdev->dev);
		pm_runtime_mark_last_busy(&pdev->dev);
		pm_runtime_put_noidle(&pdev->dev);
	}

	return 0;

err_unprepare:
	mhi_unprepare_after_power_down(mhi_cntrl);
err_unregister:
	mhi_unregister_controller(mhi_cntrl);

	return err;
}

static void mhi_pci_remove(struct pci_dev *pdev)
{
	struct mhi_pci_device *mhi_pdev = pci_get_drvdata(pdev);
	struct mhi_controller *mhi_cntrl = &mhi_pdev->mhi_cntrl;

	pci_disable_sriov(pdev);

	if (pdev->is_physfn)
		timer_delete_sync(&mhi_pdev->health_check_timer);
	cancel_work_sync(&mhi_pdev->recovery_work);

	if (test_and_clear_bit(MHI_PCI_DEV_STARTED, &mhi_pdev->status)) {
		mhi_power_down(mhi_cntrl, true);
		mhi_unprepare_after_power_down(mhi_cntrl);
	}

	/* balancing probe put_noidle */
	if (pci_pme_capable(pdev, PCI_D3hot))
		pm_runtime_get_noresume(&pdev->dev);

	if (mhi_pdev->reset_on_remove)
		mhi_soc_reset(mhi_cntrl);

	mhi_unregister_controller(mhi_cntrl);
}

static void mhi_pci_shutdown(struct pci_dev *pdev)
{
	mhi_pci_remove(pdev);
	pci_set_power_state(pdev, PCI_D3hot);
}

static void mhi_pci_reset_prepare(struct pci_dev *pdev)
{
	struct mhi_pci_device *mhi_pdev = pci_get_drvdata(pdev);
	struct mhi_controller *mhi_cntrl = &mhi_pdev->mhi_cntrl;

	dev_info(&pdev->dev, "reset\n");

	if (pdev->is_physfn)
		timer_delete(&mhi_pdev->health_check_timer);

	/* Clean up MHI state */
	if (test_and_clear_bit(MHI_PCI_DEV_STARTED, &mhi_pdev->status)) {
		mhi_power_down(mhi_cntrl, false);
		mhi_unprepare_after_power_down(mhi_cntrl);
	}

	/* cause internal device reset */
	mhi_soc_reset(mhi_cntrl);

	/* Be sure device reset has been executed */
	msleep(MHI_POST_RESET_DELAY_MS);
}

static void mhi_pci_reset_done(struct pci_dev *pdev)
{
	struct mhi_pci_device *mhi_pdev = pci_get_drvdata(pdev);
	struct mhi_controller *mhi_cntrl = &mhi_pdev->mhi_cntrl;
	int err;

	/* Restore initial known working PCI state */
	pci_load_saved_state(pdev, mhi_pdev->pci_state);
	pci_restore_state(pdev);

	/* Is device status available ? */
	if (!mhi_pci_is_alive(mhi_cntrl)) {
		dev_err(&pdev->dev, "reset failed\n");
		return;
	}

	err = mhi_prepare_for_power_up(mhi_cntrl);
	if (err) {
		dev_err(&pdev->dev, "failed to prepare MHI controller\n");
		return;
	}

	err = mhi_sync_power_up(mhi_cntrl);
	if (err) {
		dev_err(&pdev->dev, "failed to power up MHI controller\n");
		mhi_unprepare_after_power_down(mhi_cntrl);
		return;
	}

	set_bit(MHI_PCI_DEV_STARTED, &mhi_pdev->status);
	if (pdev->is_physfn)
		mod_timer(&mhi_pdev->health_check_timer, jiffies + HEALTH_CHECK_PERIOD);
}

static pci_ers_result_t mhi_pci_error_detected(struct pci_dev *pdev,
					       pci_channel_state_t state)
{
	struct mhi_pci_device *mhi_pdev = pci_get_drvdata(pdev);
	struct mhi_controller *mhi_cntrl = &mhi_pdev->mhi_cntrl;

	dev_err(&pdev->dev, "PCI error detected, state = %u\n", state);

	if (state == pci_channel_io_perm_failure)
		return PCI_ERS_RESULT_DISCONNECT;

	/* Clean up MHI state */
	if (test_and_clear_bit(MHI_PCI_DEV_STARTED, &mhi_pdev->status)) {
		mhi_power_down(mhi_cntrl, false);
		mhi_unprepare_after_power_down(mhi_cntrl);
	} else {
		/* Nothing to do */
		return PCI_ERS_RESULT_RECOVERED;
	}

	pci_disable_device(pdev);

	return PCI_ERS_RESULT_NEED_RESET;
}

static pci_ers_result_t mhi_pci_slot_reset(struct pci_dev *pdev)
{
	if (pci_enable_device(pdev)) {
		dev_err(&pdev->dev, "Cannot re-enable PCI device after reset.\n");
		return PCI_ERS_RESULT_DISCONNECT;
	}

	return PCI_ERS_RESULT_RECOVERED;
}

static void mhi_pci_io_resume(struct pci_dev *pdev)
{
	struct mhi_pci_device *mhi_pdev = pci_get_drvdata(pdev);

	dev_err(&pdev->dev, "PCI slot reset done\n");

	queue_work(system_long_wq, &mhi_pdev->recovery_work);
}

static const struct pci_error_handlers mhi_pci_err_handler = {
	.error_detected = mhi_pci_error_detected,
	.slot_reset = mhi_pci_slot_reset,
	.resume = mhi_pci_io_resume,
	.reset_prepare = mhi_pci_reset_prepare,
	.reset_done = mhi_pci_reset_done,
};

static int  __maybe_unused mhi_pci_runtime_suspend(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct mhi_pci_device *mhi_pdev = dev_get_drvdata(dev);
	struct mhi_controller *mhi_cntrl = &mhi_pdev->mhi_cntrl;
	int err;

	if (test_and_set_bit(MHI_PCI_DEV_SUSPENDED, &mhi_pdev->status))
		return 0;

	if (pdev->is_physfn)
		timer_delete(&mhi_pdev->health_check_timer);

	cancel_work_sync(&mhi_pdev->recovery_work);

	if (!test_bit(MHI_PCI_DEV_STARTED, &mhi_pdev->status) ||
			mhi_cntrl->ee != MHI_EE_AMSS)
		goto pci_suspend; /* Nothing to do at MHI level */

	/* Transition to M3 state */
	err = mhi_pm_suspend(mhi_cntrl);
	if (err) {
		dev_err(&pdev->dev, "failed to suspend device: %d\n", err);
		clear_bit(MHI_PCI_DEV_SUSPENDED, &mhi_pdev->status);
		return -EBUSY;
	}

pci_suspend:
	pci_disable_device(pdev);
	pci_wake_from_d3(pdev, true);

	return 0;
}

static int __maybe_unused mhi_pci_runtime_resume(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct mhi_pci_device *mhi_pdev = dev_get_drvdata(dev);
	struct mhi_controller *mhi_cntrl = &mhi_pdev->mhi_cntrl;
	int err;

	if (!test_and_clear_bit(MHI_PCI_DEV_SUSPENDED, &mhi_pdev->status))
		return 0;

	err = pci_enable_device(pdev);
	if (err)
		goto err_recovery;

	pci_set_master(pdev);
	pci_wake_from_d3(pdev, false);

	if (!test_bit(MHI_PCI_DEV_STARTED, &mhi_pdev->status) ||
			mhi_cntrl->ee != MHI_EE_AMSS)
		return 0; /* Nothing to do at MHI level */

	/* Exit M3, transition to M0 state */
	err = mhi_pm_resume(mhi_cntrl);
	if (err) {
		dev_err(&pdev->dev, "failed to resume device: %d\n", err);
		goto err_recovery;
	}

	/* Resume health check */
	if (pdev->is_physfn)
		mod_timer(&mhi_pdev->health_check_timer, jiffies + HEALTH_CHECK_PERIOD);

	/* It can be a remote wakeup (no mhi runtime_get), update access time */
	pm_runtime_mark_last_busy(dev);

	return 0;

err_recovery:
	/* Do not fail to not mess up our PCI device state, the device likely
	 * lost power (d3cold) and we simply need to reset it from the recovery
	 * procedure, trigger the recovery asynchronously to prevent system
	 * suspend exit delaying.
	 */
	queue_work(system_long_wq, &mhi_pdev->recovery_work);
	pm_runtime_mark_last_busy(dev);

	return 0;
}

static int  __maybe_unused mhi_pci_suspend(struct device *dev)
{
	pm_runtime_disable(dev);
	return mhi_pci_runtime_suspend(dev);
}

static int __maybe_unused mhi_pci_resume(struct device *dev)
{
	int ret;

	/* Depending the platform, device may have lost power (d3cold), we need
	 * to resume it now to check its state and recover when necessary.
	 */
	ret = mhi_pci_runtime_resume(dev);
	pm_runtime_enable(dev);

	return ret;
}

static int __maybe_unused mhi_pci_freeze(struct device *dev)
{
	struct mhi_pci_device *mhi_pdev = dev_get_drvdata(dev);
	struct mhi_controller *mhi_cntrl = &mhi_pdev->mhi_cntrl;

	/* We want to stop all operations, hibernation does not guarantee that
	 * device will be in the same state as before freezing, especially if
	 * the intermediate restore kernel reinitializes MHI device with new
	 * context.
	 */
	flush_work(&mhi_pdev->recovery_work);
	if (test_and_clear_bit(MHI_PCI_DEV_STARTED, &mhi_pdev->status)) {
		mhi_power_down(mhi_cntrl, true);
		mhi_unprepare_after_power_down(mhi_cntrl);
	}

	return 0;
}

static int __maybe_unused mhi_pci_restore(struct device *dev)
{
	struct mhi_pci_device *mhi_pdev = dev_get_drvdata(dev);

	/* Reinitialize the device */
	queue_work(system_long_wq, &mhi_pdev->recovery_work);

	return 0;
}

static const struct dev_pm_ops mhi_pci_pm_ops = {
	SET_RUNTIME_PM_OPS(mhi_pci_runtime_suspend, mhi_pci_runtime_resume, NULL)
#ifdef CONFIG_PM_SLEEP
	.suspend = mhi_pci_suspend,
	.resume = mhi_pci_resume,
	.freeze = mhi_pci_freeze,
	.thaw = mhi_pci_restore,
	.poweroff = mhi_pci_freeze,
	.restore = mhi_pci_restore,
#endif
};

static struct pci_driver mhi_pci_driver = {
	.name		= "mhi-pci-generic",
	.id_table	= mhi_pci_id_table,
	.probe		= mhi_pci_probe,
	.remove		= mhi_pci_remove,
	.shutdown	= mhi_pci_shutdown,
	.err_handler	= &mhi_pci_err_handler,
	.driver.pm	= &mhi_pci_pm_ops,
	.sriov_configure = pci_sriov_configure_simple,
};
module_pci_driver(mhi_pci_driver);

MODULE_AUTHOR("Loic Poulain <loic.poulain@linaro.org>");
MODULE_DESCRIPTION("Modem Host Interface (MHI) PCI controller driver");
MODULE_LICENSE("GPL");
