// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2026, Advanced Micro Devices, Inc.
 */

#include <drm/amdxdna_accel.h>
#include <drm/drm_device.h>

#include "aie4_pci.h"
#include "amdxdna_pci_drv.h"

#define NPU3_MBOX_BAR		0

#define NPU3_MBOX_BUFFER_BAR	2
#define NPU3_MBOX_INFO_OFF	0x0

/* PCIe BAR Index for NPU3 */
#define NPU3_REG_BAR_INDEX	0

static const struct amdxdna_fw_feature_tbl npu3_fw_feature_table[] = {
	{ .major = 5, .min_minor = 10 },
	{ 0 }
};

static const struct amdxdna_dev_priv npu3_dev_priv = {
	.mbox_bar		= NPU3_MBOX_BAR,
	.mbox_rbuf_bar		= NPU3_MBOX_BUFFER_BAR,
	.mbox_info_off		= NPU3_MBOX_INFO_OFF,
};

const struct amdxdna_dev_info dev_npu3_pf_info = {
	.mbox_bar		= NPU3_MBOX_BAR,
	.sram_bar		= NPU3_MBOX_BUFFER_BAR,
	.vbnv			= "RyzenAI-npu3-pf",
	.device_type		= AMDXDNA_DEV_TYPE_PF,
	.dev_priv		= &npu3_dev_priv,
	.fw_feature_tbl		= npu3_fw_feature_table,
	.ops			= &aie4_ops,
};
