// SPDX-License-Identifier: GPL-2.0-only
/*
 * Intel Bluetooth PCIe Debug Framework
 *
 * Copyright (C) 2026 Intel Corporation
 *
 * Runtime-configurable TLV-based debug collection framework for loading and
 * parsing the optional YoYo firmware configuration used to enable debug
 * regions, triggers, HCI commands, and configuration sets. Also provides
 * debug INI configuration and fatal-error injection testmode.
 */

#include <linux/errno.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/stddef.h>
#include <linux/string.h>
#include <linux/unaligned.h>
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>
#include "btintel_pcie.h"
#include "btintel_pcie_debug.h"

static char *btintel_pcie_read_debug_ini(struct btintel_pcie_data *data)
{
	const struct firmware *fw;
	char *buf;
	int err;

	err = request_firmware_direct(&fw, BTINTEL_PCIE_DBG_INI_FILE,
				      &data->pdev->dev);
	if (err)
		return ERR_PTR(err);

	buf = kmemdup_nul(fw->data, fw->size, GFP_KERNEL);
	release_firmware(fw);
	if (!buf)
		return ERR_PTR(-ENOMEM);

	return buf;
}

static char *btintel_pcie_ini_next(char **cursor, char **key)
{
	char *line, *eq;

	while ((line = strsep(cursor, "\n"))) {
		line = strim(line);
		if (!line[0] || line[0] == '#' || line[0] == ';')
			continue;

		eq = strchr(line, '=');
		if (!eq)
			continue;

		*eq = '\0';
		*key = strim(line);
		return strim(eq + 1);
	}

	return NULL;
}

/* Loads optional "DBG_PATH=<value>", "ENABLE_INI=<value>" and
 * "LOG_MASK=<value>" overrides from BTINTEL_PCIE_DBG_INI_FILE and applies
 * them to data->dbg_path_cache, data->enable_ini and data->log_mask.
 *
 * DBG_PATH accepts BTINTEL_PCIE_WIFI_DBGC (0x06, default) and
 * BTINTEL_PCIE_DRAM (0x01, legacy); an invalid or missing entry leaves
 * the cached value set by the caller unchanged.
 *
 * ENABLE_INI accepts 1 (enabled) or 0 (disabled); an invalid or missing
 * entry leaves data->enable_ini unchanged.
 *
 * LOG_MASK accepts any u32 bitmask (see BTINTEL_PCIE_DUMP_TX and friends).
 * The default enables alive interrupt and Sx logging; an invalid or missing
 * entry leaves data->log_mask unchanged.
 */
void btintel_pcie_load_dbg_path_from_ini(struct btintel_pcie_data *data)
{
	const char *ini_file = BTINTEL_PCIE_DBG_INI_FILE;
	char *buf, *cursor, *key, *val;
	bool dbg_path_applied = false;
	bool log_mask_applied = false;

	buf = btintel_pcie_read_debug_ini(data);
	if (IS_ERR(buf)) {
		BT_DBG("%s unavailable (%ld), using dbg_path=0x%02x log_mask=0x%08x",
		       ini_file, PTR_ERR(buf), data->dbg_path_cache,
		       data->log_mask);
		if (data->dbg_path_cache != BTINTEL_PCIE_DRAM)
			data->enable_ini = false;
		return;
	}

	cursor = buf;
	while ((val = btintel_pcie_ini_next(&cursor, &key))) {
		unsigned int parsed;

		if (!strcasecmp(key, "DBG_PATH")) {
			BT_INFO("Read DBG_PATH from %s: '%s'", ini_file, val);
			if (!kstrtouint(val, 0, &parsed) &&
			    (parsed == BTINTEL_PCIE_DRAM ||
			     parsed == BTINTEL_PCIE_WIFI_DBGC)) {
				data->dbg_path_cache = parsed;
				dbg_path_applied = true;
				BT_INFO("WRT dbg_path accepted: 0x%02x",
					data->dbg_path_cache);
			} else {
				BT_WARN("Invalid DBG_PATH='%s' in %s, using dbg_path=0x%02x",
					val, ini_file, data->dbg_path_cache);
			}
		} else if (!strcasecmp(key, "LOG_MASK")) {
			BT_INFO("Read LOG_MASK from %s: '%s'", ini_file, val);
			if (!kstrtouint(val, 0, &parsed)) {
				data->log_mask = parsed;
				log_mask_applied = true;
				BT_INFO("WRT log_mask accepted: 0x%08x",
					data->log_mask);
			} else {
				BT_WARN("Invalid LOG_MASK='%s' in %s, using log_mask=0x%08x",
					val, ini_file, data->log_mask);
			}
		} else if (!strcasecmp(key, "ENABLE_INI")) {
			BT_INFO("Read ENABLE_INI from %s: '%s'", ini_file, val);
			if (!kstrtouint(val, 0, &parsed) && parsed <= 1) {
				data->enable_ini = !!parsed;
				BT_INFO("WRT enable_ini accepted: %u",
					data->enable_ini);
			} else {
				BT_WARN("Invalid ENABLE_INI='%s' in %s, using enable_ini=%u",
					val, ini_file, data->enable_ini);
			}
		}
	}

	if (data->dbg_path_cache != BTINTEL_PCIE_DRAM)
		data->enable_ini = false;

	if (dbg_path_applied)
		BT_DBG("Loaded dbg_path=0x%02x from %s", data->dbg_path_cache, ini_file);
	else
		BT_WARN("No valid DBG_PATH in %s, dbg_path=0x%02x", ini_file, data->dbg_path_cache);

	BT_DBG("ENABLE_INI effective state: %u", data->enable_ini);

	if (log_mask_applied)
		BT_DBG("Loaded log_mask=0x%08x from %s", data->log_mask, ini_file);
	else
		BT_DBG("No valid LOG_MASK in %s, log_mask=0x%08x", ini_file, data->log_mask);

	kfree(buf);
}
EXPORT_SYMBOL_GPL(btintel_pcie_load_dbg_path_from_ini);

void btintel_pcie_fatal_err_init(struct btintel_pcie_data *data)
{
	data->fatal_err.testmode_enabled = false;
	data->fatal_err.context = BTINTEL_PCIE_DEBUG_EXCEPTION_CONTEXT_THREAD;
	data->fatal_err.type = BTINTEL_PCIE_DEBUG_EXCEPTION_FAILURE_TYPE_DEFAULT;
}
EXPORT_SYMBOL_GPL(btintel_pcie_fatal_err_init);

void btintel_pcie_fatal_err_load(struct btintel_pcie_data *data)
{
	char *buf, *cursor, *key, *val;
	u32 parsed;

	WRITE_ONCE(data->fatal_err.testmode_enabled, false);
	clear_bit(BTINTEL_PCIE_FATAL_ERR_POST_SETUP_PENDING, &data->flags);
	buf = btintel_pcie_read_debug_ini(data);
	if (IS_ERR(buf)) {
		BT_DBG("%s unavailable (%ld); fatal-error testmode disabled",
		       BTINTEL_PCIE_DBG_INI_FILE, PTR_ERR(buf));
		return;
	}

	cursor = buf;
	while ((val = btintel_pcie_ini_next(&cursor, &key))) {
		if (strcasecmp(key, "FATAL_ERR_TESTMODE"))
			continue;

		if (kstrtou32(val, 0, &parsed) ||
		    BTINTEL_PCIE_FATAL_ERR_ENABLE(parsed) > 1 ||
		    BTINTEL_PCIE_FATAL_ERR_CONTEXT(parsed) >
				BTINTEL_PCIE_DEBUG_EXCEPTION_CONTEXT_RFKILL ||
		    parsed >> 24) {
			BT_ERR("Invalid FATAL_ERR_TESTMODE='%s' in %s; testmode disabled",
			       val, BTINTEL_PCIE_DBG_INI_FILE);
			break;
		}

		data->fatal_err.context = BTINTEL_PCIE_FATAL_ERR_CONTEXT(parsed);
		data->fatal_err.type = BTINTEL_PCIE_FATAL_ERR_TYPE(parsed);
		WRITE_ONCE(data->fatal_err.testmode_enabled,
			   BTINTEL_PCIE_FATAL_ERR_ENABLE(parsed));
		BT_INFO("FATAL_ERR_TESTMODE accepted: 0x%08x", parsed);
		break;
	}

	kfree(buf);
}
EXPORT_SYMBOL_GPL(btintel_pcie_fatal_err_load);

void btintel_pcie_fatal_err_check_api_lock(struct btintel_pcie_data *data,
					   bool api_lock)
{
	if (api_lock) {
		if (READ_ONCE(data->fatal_err.testmode_enabled))
			bt_dev_info(data->hdev,
				    "OPFW API lock enabled; disabling fatal-error testmode");
		WRITE_ONCE(data->fatal_err.testmode_enabled, false);
	} else if (READ_ONCE(data->fatal_err.testmode_enabled)) {
		bt_dev_info(data->hdev, "fatal-error testmode active: ctx=0x%02x type=0x%02x",
			    data->fatal_err.context, data->fatal_err.type);
	}
}
EXPORT_SYMBOL_GPL(btintel_pcie_fatal_err_check_api_lock);

static int btintel_pcie_inject_fatal_err(struct btintel_pcie_data *data,
					 u8 context)
{
	struct hci_dev *hdev = data->hdev;
	struct sk_buff *skb;
	u8 param[2];
	int err = 0;

	/* Setup and the rest of HCI initialization hold req_lock. Wait for
	 * them before testing HCI_UP; a delayed worker alone is not enough.
	 */
	hci_req_sync_lock(hdev);
	if (!test_bit(HCI_UP, &hdev->flags) ||
	    hci_dev_test_flag(hdev, HCI_UNREGISTER)) {
		err = -ENETDOWN;
		goto unlock;
	}

	if (!READ_ONCE(data->fatal_err.testmode_enabled))
		goto unlock;

	param[0] = data->fatal_err.type;
	param[1] = context;
	skb = __hci_cmd_sync(hdev, 0xfc4d, sizeof(param), param, HCI_CMD_TIMEOUT);
	if (IS_ERR(skb)) {
		err = PTR_ERR(skb);
		goto unlock;
	}

	kfree_skb(skb);
unlock:
	hci_req_sync_unlock(hdev);
	if (err && err != -ENETDOWN)
		bt_dev_err(hdev, "fatal-error inject failed (ctx=0x%02x err=%d)",
			   context, err);
	return err;
}

bool btintel_pcie_fatal_err_handle_rfkill(struct btintel_pcie_data *data,
					  bool rfkill_on)
{
	if (!READ_ONCE(data->fatal_err.testmode_enabled))
		return false;

	if (rfkill_on) {
		/* HCI down: nothing to inject, use the normal handling */
		if (btintel_pcie_inject_fatal_err(data, data->fatal_err.context) ==
		    -ENETDOWN)
			return false;
		return true;
	}

	/* Release of an assertion handled by the normal path: let it through */
	if (test_bit(BTINTEL_PCIE_HWRFKILL_ON, &data->flags))
		return false;

	bt_dev_dbg(data->hdev, "fatal-error testmode active: ignoring RF-KILL OFF");
	return true;
}
EXPORT_SYMBOL_GPL(btintel_pcie_fatal_err_handle_rfkill);

bool btintel_pcie_fatal_err_post_setup_pending(struct btintel_pcie_data *data)
{
	int err;

	if (!test_bit(BTINTEL_PCIE_FATAL_ERR_POST_SETUP_PENDING, &data->flags))
		return false;

	/* Intentional: RFKILL context tells the controller testmode is enabled */
	err = btintel_pcie_inject_fatal_err(data,
					    BTINTEL_PCIE_DEBUG_EXCEPTION_CONTEXT_RFKILL);
	/* HCI is down: keep the injection pending for a later pass and let
	 * the normal rfkill handling run.
	 */
	if (err == -ENETDOWN)
		return false;

	clear_bit(BTINTEL_PCIE_FATAL_ERR_POST_SETUP_PENDING, &data->flags);
	return true;
}
EXPORT_SYMBOL_GPL(btintel_pcie_fatal_err_post_setup_pending);

bool btintel_pcie_fatal_err_testmode_active(struct btintel_pcie_data *data)
{
	return READ_ONCE(data->fatal_err.testmode_enabled);
}
EXPORT_SYMBOL_GPL(btintel_pcie_fatal_err_testmode_active);

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

MODULE_DESCRIPTION("Intel Bluetooth PCIe Debug Framework");
MODULE_LICENSE("GPL");
