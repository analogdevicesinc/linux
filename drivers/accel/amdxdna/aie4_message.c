// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2026, Advanced Micro Devices, Inc.
 */

#include <drm/amdxdna_accel.h>
#include <drm/drm_print.h>
#include <linux/bitfield.h>
#include <linux/iommu.h>
#include <linux/mutex.h>

#include "aie.h"
#include "aie4_msg_priv.h"
#include "aie4_pci.h"
#include "amdxdna_mailbox.h"
#include "amdxdna_mailbox_helper.h"
#include "amdxdna_pci_drv.h"

u32 aie4_msg_pasid(struct amdxdna_client *client)
{
	if (!amdxdna_pasid_on(client))
		return IOMMU_PASID_INVALID;

	return FIELD_PREP(AIE4_MSG_PASID, client->pasid) |
	       FIELD_PREP(AIE4_MSG_PASID_VLD, 1);
}

int aie4_suspend_fw(struct amdxdna_dev_hdl *ndev)
{
	DECLARE_AIE_MSG(aie4_msg_suspend, AIE4_MSG_OP_SUSPEND);
	int ret;

	ret = aie_send_mgmt_msg_wait(&ndev->aie, &msg);
	if (ret)
		XDNA_ERR(ndev->aie.xdna, "Failed to suspend fw, ret %d", ret);

	return ret;
}

int aie4_query_aie_metadata(struct amdxdna_dev_hdl *ndev,
			    struct amdxdna_drm_query_aie_metadata *metadata)
{
	DECLARE_AIE_MSG(aie4_msg_aie4_tile_info, AIE4_MSG_OP_AIE_TILE_INFO);
	int ret;

	ret = aie_send_mgmt_msg_wait(&ndev->aie, &msg);
	if (ret)
		return ret;

	metadata->col_size = resp.info.size;
	metadata->cols = resp.info.cols;
	metadata->rows = resp.info.rows;

	metadata->version.major = resp.info.major;
	metadata->version.minor = resp.info.minor;

	metadata->core.row_count = resp.info.core_rows;
	metadata->core.row_start = resp.info.core_row_start;
	metadata->core.dma_channel_count = resp.info.core_dma_channels;
	metadata->core.lock_count = resp.info.core_locks;
	metadata->core.event_reg_count = resp.info.core_events;

	metadata->mem.row_count = resp.info.mem_rows;
	metadata->mem.row_start = resp.info.mem_row_start;
	metadata->mem.dma_channel_count = resp.info.mem_dma_channels;
	metadata->mem.lock_count = resp.info.mem_locks;
	metadata->mem.event_reg_count = resp.info.mem_events;

	metadata->shim.row_count = resp.info.shim_rows;
	metadata->shim.row_start = resp.info.shim_row_start;
	metadata->shim.dma_channel_count = resp.info.shim_dma_channels;
	metadata->shim.lock_count = resp.info.shim_locks;
	metadata->shim.event_reg_count = resp.info.shim_events;

	return 0;
}

int aie4_query_aie_version(struct amdxdna_dev_hdl *ndev,
			   struct amdxdna_drm_query_aie_version *version)
{
	DECLARE_AIE_MSG(aie4_msg_aie4_version_info, AIE4_MSG_OP_AIE_VERSION_INFO);
	struct amdxdna_dev *xdna = ndev->aie.xdna;
	int ret;

	ret = aie_send_mgmt_msg_wait(&ndev->aie, &msg);
	if (ret)
		return ret;

	XDNA_DBG(xdna, "Query AIE version - major: %u minor: %u",
		 resp.major, resp.minor);

	version->major = resp.major;
	version->minor = resp.minor;

	return 0;
}

int aie4_query_npu_firmware_version(struct amdxdna_dev_hdl *ndev,
				    struct amdxdna_drm_query_firmware_version *fw_version)
{
	DECLARE_AIE_MSG(aie4_msg_identify, AIE4_MSG_OP_IDENTIFY);
	int ret;

	ret = aie_send_mgmt_msg_wait(&ndev->aie, &msg);
	if (ret)
		return ret;

	fw_version->major = resp.fw_major;
	fw_version->minor = resp.fw_minor;
	fw_version->patch = resp.fw_patch;
	fw_version->build = resp.fw_build;

	return 0;
}

int aie4_query_cert_firmware_version(struct amdxdna_dev_hdl *ndev,
				     struct amdxdna_drm_query_firmware_version *cert_version)
{
	DECLARE_AIE_MSG(aie4_msg_query_cert_firmware_version,
			AIE4_MSG_OP_QUERY_CERT_FIRMWARE_VERSION);
	int ret;

	ret = aie_send_mgmt_msg_wait(&ndev->aie, &msg);
	if (ret)
		return ret;

	ret = aie_check_protocol(&ndev->aie,
				 resp.host_queue_major, resp.host_queue_minor,
				 ndev->aie.xdna->dev_info->cert_feature_tbl);
	if (ret) {
		XDNA_ERR(ndev->aie.xdna, "host queue %d.%d is not supported",
			 resp.host_queue_major, resp.host_queue_minor);
		return ret;
	}

	cert_version->major = resp.major_version;
	cert_version->minor = resp.minor_version;
	cert_version->patch = resp.hotfix;
	cert_version->build = resp.build;

	return 0;
}

int aie4_init_dpm_freq_table(struct amdxdna_dev_hdl *ndev)
{
	DECLARE_AIE_MSG(aie4_msg_get_dpm_freq_table, AIE4_MSG_OP_GET_DPM_FREQ_TABLE);
	struct amdxdna_dev *xdna = ndev->aie.xdna;
	u32 aie_levels, npu_levels, i;
	int ret;

	for (i = 0; i < AIE4_MAX_DPM_LEVEL_COUNT && ndev->priv->dpm_clk_tbl &&
	     ndev->priv->dpm_clk_tbl[i].hclk; i++)
		ndev->dpm_clk_tbl[i] = ndev->priv->dpm_clk_tbl[i];
	ndev->max_aieclk_level = i ? i - 1 : 0;
	ndev->max_npuhclk_level = i ? i - 1 : 0;

	ret = aie_send_mgmt_msg_wait(&ndev->aie, &msg);
	if (ret) {
		XDNA_WARN(xdna, "Get DPM freq table failed, ret %d status 0x%x",
			  ret, resp.status);
		return ret;
	}

	aie_levels = resp.aieclk_table.num_levels;
	npu_levels = resp.npuhclk_table.num_levels;

	if (!aie_levels || !npu_levels ||
	    aie_levels > AIE4_MAX_DPM_LEVEL_COUNT ||
	    npu_levels > AIE4_MAX_DPM_LEVEL_COUNT) {
		XDNA_ERR(xdna, "invalid dpm levels, aieclk: %u, npuhclk: %u",
			 aie_levels, npu_levels);
		return -EINVAL;
	}

	for (i = 0; i < aie_levels; i++) {
		if (!resp.aieclk_table.values[i]) {
			XDNA_ERR(xdna, "invalid dpm aieclk frequency 0 at level %u", i);
			return -EINVAL;
		}
	}

	for (i = 0; i < npu_levels; i++) {
		if (!resp.npuhclk_table.values[i]) {
			XDNA_ERR(xdna, "invalid dpm npuhclk frequency 0 at level %u", i);
			return -EINVAL;
		}
	}

	memset(ndev->dpm_clk_tbl, 0, sizeof(ndev->dpm_clk_tbl));
	for (i = 0; i < aie_levels; i++)
		ndev->dpm_clk_tbl[i].npuclk = resp.aieclk_table.values[i];

	for (i = 0; i < npu_levels; i++)
		ndev->dpm_clk_tbl[i].hclk = resp.npuhclk_table.values[i];

	ndev->max_aieclk_level = aie_levels - 1;
	ndev->max_npuhclk_level = npu_levels - 1;

	return 0;
}

int aie4_query_dpm_level(struct amdxdna_dev_hdl *ndev,
			 u32 *aieclk_dpm_level, u32 *npuhclk_dpm_level)
{
	DECLARE_AIE_MSG(aie4_msg_get_dpm_level, AIE4_MSG_OP_GET_CURRENT_DPM_LEVEL);
	struct amdxdna_dev *xdna = ndev->aie.xdna;
	int ret;

	ret = aie_send_mgmt_msg_wait(&ndev->aie, &msg);
	if (ret)
		return ret;

	/*
	 * Validate against ndev->max_aieclk_level and ndev->max_npuhclk_level
	 * to ensure reported levels index into populated entries in dpm_clk_tbl.
	 */
	if (resp.aieclk_dpm_level > ndev->max_aieclk_level ||
	    resp.npuhclk_dpm_level > ndev->max_npuhclk_level) {
		XDNA_ERR(xdna,
			 "invalid dpm level, aie: %u/%u, npu: %u/%u",
			 resp.aieclk_dpm_level, ndev->max_aieclk_level,
			 resp.npuhclk_dpm_level, ndev->max_npuhclk_level);
		return -EINVAL;
	}

	*aieclk_dpm_level = resp.aieclk_dpm_level;
	*npuhclk_dpm_level = resp.npuhclk_dpm_level;

	return 0;
}

int aie4_attach_work_buffer(struct amdxdna_dev_hdl *ndev)
{
	DECLARE_AIE_MSG(aie4_msg_attach_work_buffer, AIE4_MSG_OP_ATTACH_WORK_BUFFER);
	struct amdxdna_dev *xdna = ndev->aie.xdna;
	int ret;

	req.buff_addr = ndev->work_buf_addr;
	req.buff_size = AIE4_WORK_BUFFER_MIN_SIZE;

	ret = aie_send_mgmt_msg_wait(&ndev->aie, &msg);
	if (ret)
		XDNA_ERR(xdna, "Failed to attach work buffer, ret %d", ret);
	else
		XDNA_DBG(xdna, "Attached work buffer");

	return ret;
}

int aie4_start_fw_log(struct amdxdna_dev_hdl *ndev, u32 level)
{
	DECLARE_AIE_MSG(aie4_msg_start_fw_log, AIE4_MSG_OP_START_FW_LOG);
	struct amdxdna_dev *xdna = ndev->aie.xdna;
	int ret;

	req.buff_addr = ndev->fw_log_buf_addr;
	req.buff_size = ndev->fw_log_buf_size;
	req.log_level = level;

	ret = aie_send_mgmt_msg_wait(&ndev->aie, &msg);
	if (ret)
		XDNA_WARN(xdna, "Failed to start fw log, ret %d", ret);
	else
		XDNA_DBG(xdna, "Started fw log, level %u size 0x%x",
			 level, ndev->fw_log_buf_size);

	return ret;
}

int aie4_msg_set_power_mode(struct amdxdna_dev_hdl *ndev, u8 power_mode)
{
	DECLARE_AIE_MSG(aie4_msg_power_override, AIE4_MSG_OP_POWER_OVERRIDE);
	int ret;

	req.power_mode = power_mode;

	ret = aie_send_mgmt_msg_wait(&ndev->aie, &msg);
	if (ret)
		XDNA_ERR(ndev->aie.xdna,
			 "Failed to set power mode %u, ret %d", (u32)power_mode, ret);
	else
		XDNA_DBG(ndev->aie.xdna, "Power mode set to %u", (u32)power_mode);

	return ret;
}

int aie4_set_runtime_cfg(struct amdxdna_dev_hdl *ndev, u32 type,
			 const void *data, size_t size)
{
	DECLARE_AIE_MSG(aie4_msg_set_runtime_cfg, AIE4_MSG_OP_SET_RUNTIME_CONFIG);
	u8 buf[sizeof(req.type) + AIE4_RUNTIME_CFG_MAX_DATA_SIZE] __aligned(4) = { 0 };
	int ret;

	if (size > AIE4_RUNTIME_CFG_MAX_DATA_SIZE)
		return -EINVAL;

	/* Stage runtime config type and payload for firmware mailbox message. */
	req.type = type;
	memcpy(buf, &req.type, sizeof(req.type));
	memcpy(buf + sizeof(req.type), data, size);

	msg.send_data = buf;
	msg.send_size = sizeof(req.type) + size;

	ret = aie_send_mgmt_msg_wait(&ndev->aie, &msg);
	if (ret)
		XDNA_ERR(ndev->aie.xdna, "Failed to set runtime cfg %u: %d", type, ret);
	return ret;
}

int aie4_set_ctx_hysteresis(struct amdxdna_dev_hdl *ndev, u32 timeout_us)
{
	struct aie4_msg_runtime_config_ctx_switch_hysteresis cfg = {
		.timeout_us = timeout_us,
	};
	int ret;

	ret = aie4_set_runtime_cfg(ndev, AIE4_RUNTIME_CONFIG_CTX_SWITCH_HYSTERESIS,
				   &cfg, sizeof(cfg));
	if (ret)
		XDNA_WARN(ndev->aie.xdna,
			  "Failed to set ctx switch hysteresis to %u us (%d), using fw default",
			  timeout_us, ret);
	else
		XDNA_DBG(ndev->aie.xdna, "Context switch hysteresis set to %u us", timeout_us);

	return ret;
}
