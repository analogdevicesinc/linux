// SPDX-License-Identifier: GPL-2.0-only
/*
 * Intel Bluetooth YoYo Debug Framework
 *
 * Copyright (C) 2026 Intel Corporation
 *
 * Runtime-configurable TLV-based debug collection framework for loading and
 * parsing the optional YoYo firmware configuration used to enable debug
 * regions, triggers, HCI commands, and configuration sets.
 */

#include <linux/errno.h>
#include <linux/module.h>
#include <linux/stddef.h>
#include <linux/string.h>
#include <linux/unaligned.h>
#include <net/bluetooth/bluetooth.h>
#include "btintel_pcie_debug.h"

/* TLV-specific parser DEBUG_INFO TLV type */
static int btintel_pcie_yoyo_parse_dbg_info(struct btintel_pcie_yoyo_ctx *ctx,
					    const struct btintel_pcie_tlv_dbg_info *info,
					    u32 len)
{
	const u8 *data = (const u8 *)info;
	u32 domain, image_type, core_type, version;

	if (len != sizeof(*info) - sizeof(struct btintel_pcie_tlv_hdr))
		return -EBADMSG;

	image_type = get_unaligned_le32(data +
				       offsetof(struct btintel_pcie_tlv_dbg_info,
						image_type));
	core_type = get_unaligned_le32(data +
				      offsetof(struct btintel_pcie_tlv_dbg_info,
					       core_type));
	domain = get_unaligned_le32(data +
				   offsetof(struct btintel_pcie_tlv_dbg_info,
					    dbg_hdr.domain));
	version = get_unaligned_le32(data +
				     offsetof(struct btintel_pcie_tlv_dbg_info,
					      dbg_hdr.tlv_data_version));

	if (domain != BTINTEL_PCIE_DOMAIN_BT ||
	    version != BTINTEL_PCIE_DBG_INFO_TLV_VERSION) {
		BT_WARN("YoYo: unsupported domain/version (%u/%u), disabling",
			domain, version);
		return -EINVAL;
	}

	if (image_type != BTINTEL_PCIE_IMAGE_TLV_TYPE_EXTERNAL) {
		BT_WARN("YoYo: unsupported image_type 0x%x, disabling", image_type);
		return -EINVAL;
	}

	if (core_type != BTINTEL_PCIE_CORE_TYPE_BT) {
		BT_WARN("YoYo: non BT core_type %u, disabling", core_type);
		return -EINVAL;
	}

	ctx->dbg_info_present = true;
	BT_DBG("YoYo: config='%.64s' image=%u core=%u",
	       info->config_name, image_type, core_type);
	return 0;
}

static int btintel_pcie_yoyo_parse_buf_alloc(struct btintel_pcie_yoyo_ctx *ctx,
					     const struct btintel_pcie_tlv_buf_alloc *tlv,
					     u32 len)
{
	u32 alloc_id, buf_location, domain, idx, max_num_frags, min_buf_size;
	u32 req_buf_size, version;

	if (len != sizeof(*tlv) - sizeof(struct btintel_pcie_tlv_hdr)) {
		BT_WARN("YoYo: unexpected buf alloc TLV length %u", len);
		return -EBADMSG;
	}

	domain = get_unaligned_le32(&tlv->dbg_hdr.domain);
	version = get_unaligned_le32(&tlv->dbg_hdr.tlv_data_version);
	if (domain != BTINTEL_PCIE_DOMAIN_BT ||
	    version != BTINTEL_PCIE_BUF_ALLOC_TLV_VERSION) {
		BT_WARN("YoYo: unsupported buf allocation domain/version (%u/%u)",
			domain, version);
		return -EINVAL;
	}

	alloc_id = get_unaligned_le32(&tlv->alloc_id);
	if (alloc_id < BTINTEL_PCIE_ALLOC_ID_DBGC1 ||
	    alloc_id > BTINTEL_PCIE_ALLOC_ID_DBGC3) {
		BT_WARN("YoYo: invalid buf allocation id %u", alloc_id);
		return -EINVAL;
	}

	idx = alloc_id - BTINTEL_PCIE_ALLOC_ID_DBGC1;
	buf_location = get_unaligned_le32(&tlv->buf_location);
	max_num_frags = get_unaligned_le32(&tlv->max_num_frags);
	min_buf_size = get_unaligned_le32(&tlv->min_buf_size);
	req_buf_size = get_unaligned_le32(&tlv->req_buf_size);

	if (buf_location != BTINTEL_PCIE_BUF_LOC_DRAM) {
		BT_WARN("YoYo: buf allocation id=%u unsupported buffer location %u",
			alloc_id, buf_location);
		return -EINVAL;
	}

	if (!max_num_frags || max_num_frags > BTINTEL_PCIE_MAX_BUF_FRAGS ||
	    min_buf_size < BTINTEL_PCIE_MIN_BUF_SIZE ||
	    min_buf_size > BTINTEL_PCIE_MAX_BUF_SIZE ||
	    req_buf_size > BTINTEL_PCIE_MAX_BUF_SIZE ||
	    req_buf_size % max_num_frags ||
	    req_buf_size / max_num_frags < min_buf_size) {
		BT_WARN("YoYo: buf allocation id=%u req size=%u max frags=%u min size=%u",
			alloc_id, req_buf_size, max_num_frags, min_buf_size);
		return -EINVAL;
	}

	ctx->buf_alloc[idx].alloc_id = alloc_id;
	ctx->buf_alloc[idx].buf_location = buf_location;
	ctx->buf_alloc[idx].req_buf_size = req_buf_size;
	ctx->buf_alloc[idx].max_num_frags = max_num_frags;
	ctx->buf_alloc[idx].min_buf_size = min_buf_size;
	ctx->buf_alloc[idx].present = true;

	if (idx + 1 > ctx->num_buf_allocs)
		ctx->num_buf_allocs = idx + 1;

	BT_DBG("YoYo: buf allocation id=%u loc=%u req size=%u max frags=%u min size=%u",
	       alloc_id, ctx->buf_alloc[idx].buf_location,
	       ctx->buf_alloc[idx].req_buf_size,
	       ctx->buf_alloc[idx].max_num_frags,
	       ctx->buf_alloc[idx].min_buf_size);

	return 0;
}

/*
 * Parse the optional YoYo firmware TLV stream and validate each record.
 * This is the entry point for the runtime debug configuration payload.
 */
static int btintel_pcie_yoyo_parse_tlv_buffer(struct btintel_pcie_yoyo_ctx *ctx,
					      const u8 *buf, size_t buf_len)
{
	const struct btintel_pcie_tlv_hdr *tlv;
	const u8 *ptr = buf;
	size_t remaining = buf_len;
	u32 count = 0;
	int ret = 0;

	while (remaining >= sizeof(*tlv)) {
		size_t tlv_size;
		u32 type, len;

		tlv = (const struct btintel_pcie_tlv_hdr *)ptr;
		type = get_unaligned_le32(&tlv->type);
		len = get_unaligned_le32(&tlv->length);

		if (len > remaining - sizeof(*tlv)) {
			BT_ERR("YoYo: tlv %u truncated (len=%u rem=%zu)", count,
			       len, remaining);
			return -EBADMSG;
		}
		tlv_size = sizeof(*tlv) + len;

		switch (type) {
		case BTINTEL_PCIE_TLV_TYPE_DBG_INFO:
			ret = btintel_pcie_yoyo_parse_dbg_info(ctx,
							       (const void *)tlv, len);
			if (ret) {
				ctx->dbg_info_present = false;
			}
			break;
		case BTINTEL_PCIE_TLV_TYPE_DBG_BUF_ALLOC:
			ret = btintel_pcie_yoyo_parse_buf_alloc(ctx,
								(const void *)tlv, len);
			break;
		case BTINTEL_PCIE_TLV_TYPE_DBG_REGION:
		case BTINTEL_PCIE_TLV_TYPE_DBG_TRIGGER:
		case BTINTEL_PCIE_TLV_TYPE_DBG_HCMD:
		case BTINTEL_PCIE_TLV_TYPE_DBG_CONFIG_SET:
		case BTINTEL_PCIE_TLV_TYPE_DBG_PRESET_OVERRIDE:
			BT_DBG("YoYo: valid tlv type 0x%x len=%u", type, len);
			break;
		default: /* Log and continue, do not fail*/
			BT_DBG("YoYo: unknown tlv type 0x%x len=%u", type, len);
			break;
		}

		ptr += tlv_size;
		remaining -= tlv_size;

		if (ret) {
			BT_ERR("YoYo: error parsing tlv type 0x%x (ret=%d)", type, ret);
			return ret;
		}

		count++;
	}

	/* If no TLVs were parsed, the YoYo binary is considered invalid */
	if (remaining || count == 0) {
		BT_ERR("YoYo: trailing %zu bytes after tlv stream", remaining);
		return -EBADMSG;
	}
	if (!ctx->num_buf_allocs) {
		BT_ERR("YoYo: mandatory buffer allocation TLV missing");
		return -EBADMSG;
	}

	BT_DBG("YoYo: parsed %u tlv's", count);
	return 0;
}

/**
 * btintel_pcie_yoyo_init - Initialize the YoYo context
 * @ctx:   Pre-allocated context (typically embedded in driver data)
 */
int btintel_pcie_yoyo_init(struct btintel_pcie_yoyo_ctx *ctx)
{
	if (!ctx)
		return -EINVAL;

	memset(ctx, 0, sizeof(*ctx));
	INIT_LIST_HEAD(&ctx->region_list);
	INIT_LIST_HEAD(&ctx->trigger_list);
	INIT_LIST_HEAD(&ctx->hcmd_list);
	INIT_LIST_HEAD(&ctx->config_set_list);

	/* set it to true after successful TLV parsing */
	ctx->enabled = false;

	return 0;
}
EXPORT_SYMBOL_GPL(btintel_pcie_yoyo_init);

/**
 * btintel_pcie_yoyo_cleanup - Clean up the YoYo context
 * @ctx:  YoYo context to clean up
 */
void btintel_pcie_yoyo_cleanup(struct btintel_pcie_yoyo_ctx *ctx)
{
	if (!ctx)
		return;

	ctx->enabled = false;
	ctx->dbg_info_present = false;
	ctx->num_buf_allocs = 0;
	memset(ctx->buf_alloc, 0, sizeof(ctx->buf_alloc));
}
EXPORT_SYMBOL_GPL(btintel_pcie_yoyo_cleanup);

/**
 * btintel_pcie_yoyo_load_and_parse - Load ibt_debug_yoyo.bin and parse TLVs
 * @ctx:  Initialized YoYo context
 * @dev:  Device for firmware loading (typically &pdev->dev)
 *
 * Uses request_firmware_direct, since the YoYo binary
 * is optional. On success, sets ctx->enabled.
 */
int btintel_pcie_yoyo_load_and_parse(struct btintel_pcie_yoyo_ctx *ctx,
				     struct device *dev)
{
	const struct firmware *fw;
	int ret;

	if (!ctx)
		return -EINVAL;

	ret = request_firmware_direct(&fw, BTINTEL_PCIE_YOYO_FW_NAME, dev);
	if (ret) {
		BT_WARN("YoYo: failed to load %s (%d)",
			BTINTEL_PCIE_YOYO_FW_NAME, ret);
		return ret;
	}

	/* Sanity check firmware size */
	if (fw->size < sizeof(struct btintel_pcie_tlv_hdr)) {
		release_firmware(fw);
		return -EBADMSG;
	}

	BT_DBG("YoYo: loaded %s (%zu bytes)", BTINTEL_PCIE_YOYO_FW_NAME,
	       fw->size);

	ret = btintel_pcie_yoyo_parse_tlv_buffer(ctx, fw->data, fw->size);
	release_firmware(fw);

	/* File Corruption / TLV validation failed */
	if (ret) {
		BT_ERR("YoYo: tlv parsing failed (%d)", ret);
		return ret;
	}
	if (!ctx->dbg_info_present) {
		BT_ERR("YoYo: Invalid BT debug Info tlv, using legacy DBGC defaults");
		return -EBADMSG;
	}

	ctx->enabled = true;

	BT_INFO("YoYo: feature enabled and configured");
	return 0;
}
EXPORT_SYMBOL_GPL(btintel_pcie_yoyo_load_and_parse);

/**
 * btintel_pcie_yoyo_get_buf_alloc - Query buffer allocation config for a DBGC
 * @ctx: YoYo context
 * @alloc_id: Allocation ID (1=DBGC1, 2=DBGC2, 3=DBGC3)
 *
 * Return the parsed configuration, or NULL if no matching TLV was present.
 */
const struct btintel_pcie_buf_alloc *
btintel_pcie_yoyo_get_buf_alloc(struct btintel_pcie_yoyo_ctx *ctx, u32 alloc_id)
{
	u32 idx;

	if (!ctx || !ctx->enabled ||
	    alloc_id < BTINTEL_PCIE_ALLOC_ID_DBGC1 ||
	    alloc_id > BTINTEL_PCIE_ALLOC_ID_DBGC3)
		return NULL;

	idx = alloc_id - BTINTEL_PCIE_ALLOC_ID_DBGC1;
	if (!ctx->buf_alloc[idx].present)
		return NULL;

	return &ctx->buf_alloc[idx];
}
EXPORT_SYMBOL_GPL(btintel_pcie_yoyo_get_buf_alloc);

MODULE_DESCRIPTION("Intel Bluetooth yoyo debug framework");
MODULE_LICENSE("GPL");
