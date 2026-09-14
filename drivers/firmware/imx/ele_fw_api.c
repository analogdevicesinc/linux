// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2026 NXP
 */

#include "se_ctrl.h"
#include "ele_common.h"
#include "ele_fw_api.h"

static int se_cmd_receiver_allowed_cmd(struct se_if_device_ctx *dev_ctx,
				       struct se_api_msg *msg, u32 tx_msg_sz)
{
	u8 cmd = msg->header.command;

	switch (cmd) {
	case ELE_SESSION_CLOSE_REQ:
		if (tx_msg_sz < ELE_SESSION_CLOSE_REQ_SZ ||
		    msg->data[0] != dev_ctx->sess_hdl)
			return -EINVAL;
		return 0;
	case ELE_STORAGE_CLOSE_REQ:
		if (tx_msg_sz < ELE_STORAGE_CLOSE_REQ_SZ ||
		    msg->data[0] != dev_ctx->strg_hdl)
			return -EINVAL;
		return 0;
	case ELE_STORAGE_MASTER_IMPORT_REQ: {
		const struct se_cmd_addr_field *fields;
		size_t count;

		fields = ele_fw_cmd_addr_fields(cmd, &count);
		return se_val_cmd_addrs(dev_ctx, msg, tx_msg_sz, fields, count);
	}
	default:
		return -EOPNOTSUPP;
	}
}

static int se_cmd_receiver_allowed_rsp(struct se_if_device_ctx *dev_ctx,
				       struct se_api_msg *msg, u32 tx_msg_sz)
{
	struct cmd_rcvr_data_info *crcvr_info = &dev_ctx->priv->crcvr_info;
	const struct se_cmd_addr_field *fields;
	u8 cmd = msg->header.command;
	size_t count;

	switch (cmd) {
	case ELE_STORAGE_EXPORT_FINISH_REQ:
	case ELE_STORAGE_CHUNK_GET_DONE_REQ:
	case ELE_STORAGE_CHUNK_DELETE_REQ:
		return 0;
	default:
		/*
		 * These responses supply a kernel buffer address to firmware.
		 * Range-check the embedded DMA address against the calling
		 * context's shared-memory window before the message is sent.
		 */
		if (crcvr_info->cmd_rcvr_last_rcvd_cmd_id != cmd)
			return -EINVAL;

		fields = ele_fw_rsp_addr_fields(cmd, &count);
		if (!count)
			return -EOPNOTSUPP;

		return se_val_cmd_addrs(dev_ctx, msg, tx_msg_sz, fields, count);
	}
}

int ele_uapi_allowed_fw_rsp(struct se_if_device_ctx *dev_ctx, struct se_msg_hdr *header,
			    u32 tx_msg_sz)
{
	struct se_api_msg *msg = container_of(header, struct se_api_msg, header);
	struct se_if_priv *priv = dev_ctx->priv;

	scoped_guard(mutex, &priv->modify_lock)
		if (dev_ctx != priv->cmd_receiver_clbk_hdl.dev_ctx)
			return -EINVAL;

	return se_cmd_receiver_allowed_rsp(dev_ctx, msg, tx_msg_sz);
}

int ele_uapi_allowed_fw_cmd(struct se_if_device_ctx *dev_ctx, struct se_msg_hdr *header,
			    u32 tx_msg_sz, u32 rx_msg_sz)
{
	struct se_api_msg *msg = container_of(header, struct se_api_msg, header);
	struct se_if_priv *priv = dev_ctx->priv;
	const struct se_cmd_addr_field *fields;
	bool receiver_exists = false;
	bool is_cmd_receiver = false;
	size_t count;
	int ret = 0;

	scoped_guard(mutex, &priv->modify_lock) {
		if (priv->cmd_receiver_clbk_hdl.dev_ctx)
			receiver_exists = true;
		if (dev_ctx == priv->cmd_receiver_clbk_hdl.dev_ctx)
			is_cmd_receiver = true;
	}

	if (is_cmd_receiver && header->tag == priv->if_defs->cmd_tag)
		return se_cmd_receiver_allowed_cmd(dev_ctx, msg, tx_msg_sz);

	/* Reject any response message with non-command receiver */
	if (header->tag == priv->if_defs->rsp_tag)
		return -EOPNOTSUPP;

	/* Reject any other tag */
	if (header->tag != priv->if_defs->cmd_tag)
		return -EOPNOTSUPP;

	/*
	 * Identify the command first. Session/storage commands enforce their
	 * own-handle checks; crypto commands that embed DMA addresses defer to
	 * the shared range check below. Any command not named here is left with
	 * ret == 0 (permitted) as before.
	 */
	switch (header->command) {
	case ELE_SESSION_OPEN_REQ:
		/* Might be cleared as part of tear down. */
		ret = dev_ctx->sess_hdl ? -EEXIST : 0;
		if (rx_msg_sz < ELE_SESSION_OPEN_RSP_SZ)
			ret = -EINVAL;
		break;
	case ELE_SESSION_CLOSE_REQ:
		/* Might be cleared as part of tear down. */
		if (!dev_ctx->sess_hdl) {
			ret = -ENXIO;
			break;
		}
		/*
		 * A close request must target this context's own session. The
		 * handle to close is carried in the payload (data[0]); reject a
		 * request whose buffer is too short to hold it, or whose handle
		 * does not match this context. Checking the buffer size first
		 * also keeps the data[0] read in bounds. This stops one process
		 * from closing - and leaking - another process's session with a
		 * spoofed handle.
		 */
		if (tx_msg_sz < ELE_SESSION_CLOSE_REQ_SZ ||
		    msg->data[0] != dev_ctx->sess_hdl)
			ret = -EINVAL;
		break;
	case ELE_FW_GET_INFO_REQ:
	case ELE_KEY_STORE_OPEN_REQ:
	case ELE_KEY_STORE_CLOSE_REQ:
	case ELE_KEY_MGMT_OPEN_REQ:
	case ELE_KEY_MGMT_CLOSE_REQ:
	case ELE_MANAGE_KEY_GROUP_REQ:
	case ELE_GET_KEY_ATTR_REQ:
	case ELE_KEY_DELETE_REQ:
	case ELE_MAC_OPEN_REQ:
	case ELE_MAC_CLOSE_REQ:
	case ELE_CIPHER_OPEN_REQ:
	case ELE_CIPHER_CLOSE_REQ:
	case ELE_SIGNATURE_GENERATE_OPEN_REQ:
	case ELE_SIGNATURE_GENERATE_CLOSE_REQ:
	case ELE_SIGNATURE_VERIFY_OPEN_REQ:
	case ELE_SIGNATURE_VERIFY_CLOSE_REQ:
	case ELE_DATA_STORAGE_OPEN_REQ:
	case ELE_DATA_STORAGE_CLOSE_REQ:
	case ELE_DATA_DELETE_REQ:
		ret = 0;
		break;
	case ELE_STORAGE_OPEN_REQ:
		/* Might be cleared as part of tear down. */
		if (dev_ctx->strg_hdl) {
			ret = -EEXIST;
			break;
		}
		/*
		 * NOTE: this advisory early check is intentionally not the
		 * definitive exclusivity gate. modify_lock is dropped before the
		 * command is sent, creating a TOCTOU window. That window is
		 * closed by two additional layers:
		 *   1. se_if_cmd_lock, held for the full send+receive cycle, so
		 *      only one ELE_STORAGE_OPEN_REQ is in flight per MU at a
		 *      time.
		 *   2. set_dev_ctx_as_command_receiver(), which re-checks under
		 *      modify_lock after the response arrives. If two callers
		 *      race past this check, FW itself rejects the second
		 *      ELE_STORAGE_OPEN_REQ before any handle is allocated.
		 * See Documentation/driver-api/firmware/other_interfaces.rst,
		 * section "ELE_STORAGE_OPEN_REQ concurrency and
		 * command-receiver exclusivity".
		 */
		if (receiver_exists && !is_cmd_receiver)
			ret = -EBUSY;
		if (rx_msg_sz < ELE_STORAGE_OPEN_RSP_SZ)
			ret = -EINVAL;
		break;
	case ELE_STORAGE_CLOSE_REQ:
		/* Might be cleared as part of tear down. */
		if (!dev_ctx->strg_hdl) {
			ret = -ENXIO;
			break;
		}
		/* Same self-ownership check as the session close above. */
		if (tx_msg_sz < ELE_STORAGE_CLOSE_REQ_SZ ||
		    msg->data[0] != dev_ctx->strg_hdl)
			ret = -EINVAL;
		break;
	case ELE_STORAGE_STATUS_REQ:
		ret = 0;
		break;
	default:
		/* FW commands that embed DMA addresses. */
		fields = ele_fw_cmd_addr_fields(header->command, &count);
		if (!count) {
			ret = -EOPNOTSUPP;
			break;
		}

		ret = se_val_cmd_addrs(dev_ctx, msg, tx_msg_sz, fields, count);
		break;
	}

	return ret;
}

void cmd_receiver_specific_ops(struct se_if_device_ctx *dev_ctx,
			       struct se_api_msg *rx_msg)
{
	struct cmd_rcvr_data_info *crcvr_info = &dev_ctx->priv->crcvr_info;
	struct se_msg_hdr *header = &rx_msg->header;

	crcvr_info->cmd_rcvr_last_rcvd_cmd_id = 0;
	crcvr_info->cmd_rcvr_var_size = 0;
	switch (header->command) {
	case ELE_STORAGE_MASTER_EXPORT_REQ:
		/*
		 * FW sent an export-start command with key_store_size at
		 * data[1]. Save it so se_val_cmd_addrs() can range-check the
		 * response buffer when the cmd_receiver sends back the address.
		 */
		crcvr_info->cmd_rcvr_last_rcvd_cmd_id = ELE_STORAGE_MASTER_EXPORT_REQ;
		crcvr_info->cmd_rcvr_var_size = rx_msg->data[1];
		break;
	case ELE_STORAGE_CHUNK_EXPORT_REQ:
		/*
		 * FW sent a chunk-export command with chunk_size at data[1].
		 * Save it so se_val_cmd_addrs() can range-check the response
		 * buffer when the cmd_receiver sends back the address.
		 */
		crcvr_info->cmd_rcvr_last_rcvd_cmd_id = ELE_STORAGE_CHUNK_EXPORT_REQ;
		crcvr_info->cmd_rcvr_var_size = rx_msg->data[1];
		break;
	case ELE_STORAGE_CHUNK_GET_REQ:
		crcvr_info->cmd_rcvr_last_rcvd_cmd_id = ELE_STORAGE_CHUNK_GET_REQ;
		break;
	}
}

int fw_api_specific_ops(struct se_if_device_ctx *dev_ctx, struct se_api_msg *rx_msg,
			bool is_cmd_interrupted)
{
	struct se_msg_hdr *header = &rx_msg->header;
	struct se_if_priv *priv = dev_ctx->priv;

	switch (header->command) {
	case ELE_SESSION_OPEN_REQ:
		dev_ctx->sess_hdl = rx_msg->data[1];
		if (is_cmd_interrupted) {
			if (se_close_session(dev_ctx, dev_ctx->sess_hdl))
				dev_err(dev_ctx->priv->dev, "failed to close session.\n");
			dev_ctx->sess_hdl = 0;
		}
		break;
	case ELE_SESSION_CLOSE_REQ:
		dev_ctx->sess_hdl = 0;
		break;
	case ELE_STORAGE_OPEN_REQ: {
		int rc;

		/*
		 * Record the storage handle before registering as command
		 * receiver. FW has already allocated the handle; if we assigned
		 * it only after a successful registration, a failing
		 * set_dev_ctx_as_command_receiver() (e.g. -EBUSY) would leave
		 * strg_hdl at 0 while the ioctl still returns success to
		 * userspace. The kernel would then never close the handle on
		 * teardown, leaking it in FW. Storing it first guarantees
		 * cleanup_dev_ctx() closes it on the next close(), regardless
		 * of whether registration succeeded.
		 */
		dev_ctx->strg_hdl = rx_msg->data[1];

		rc = is_cmd_interrupted ? 0 : set_dev_ctx_as_command_receiver(dev_ctx);
		if (is_cmd_interrupted || rc) {
			if (se_close_storage(dev_ctx, dev_ctx->strg_hdl))
				dev_err(dev_ctx->priv->dev, "failed to close storage.\n");
			dev_ctx->strg_hdl = 0;
			if (rc)
				dev_err(priv->dev,
					"Failed to register %s as CMD-Receiver: %d\n",
					dev_ctx->devname, rc);
			return rc;
		}
		break;
	}
	case ELE_STORAGE_CLOSE_REQ:
		scoped_guard(mutex, &priv->modify_lock)
			unset_dev_ctx_as_command_receiver(dev_ctx);
		dev_ctx->strg_hdl = 0;
		break;
	}

	return 0;
}

int se_close_session(struct se_if_device_ctx *dev_ctx, u32 session_hdl)
{
	struct se_api_msg *tx_msg __free(kfree) =
		kzalloc(ELE_SESSION_CLOSE_REQ_SZ, GFP_KERNEL);
	struct se_api_msg *rx_msg __free(kfree) =
		kzalloc(ELE_SESSION_CLOSE_RSP_SZ, GFP_KERNEL);
	struct se_if_priv *priv;
	int ret;

	if (!dev_ctx || !dev_ctx->priv)
		return -EINVAL;

	if (!tx_msg || !rx_msg)
		return -ENOMEM;

	priv = dev_ctx->priv;

	/*
	 * Session close is a FW-API command; pass is_base_api=false so the
	 * header carries fw_api_ver.
	 */
	se_fill_cmd_msg_hdr(priv, (struct se_msg_hdr *)&tx_msg->header,
			    ELE_SESSION_CLOSE_REQ, ELE_SESSION_CLOSE_REQ_SZ, false);

	tx_msg->data[0] = session_hdl;

	/*
	 * Transmit on the caller's own context. Using dev_ctx (rather than
	 * hardcoding priv->priv_dev_ctx) keeps a userspace close() subject to
	 * the going_away check in ele_msg_send_rcv(): if unbind has begun and
	 * freed priv->tx_chan, the send is rejected with -ENODEV instead of
	 * touching the freed mailbox channel.
	 *
	 * The teardown and fw_busy-recovery paths instead reserve the messaging
	 * interface for their own task via se_reserve_msg_if() before calling
	 * this. That reservation (msg_excl_owner == current) is what makes
	 * ele_msg_send_rcv() let their resync closes through the going_away and
	 * fw_busy gates; they also pass priv_dev_ctx so the wait is the
	 * uninterruptible internal-context wait.
	 */

	ret = ele_msg_send_rcv(dev_ctx,
			       tx_msg,
			       ELE_SESSION_CLOSE_REQ_SZ,
			       rx_msg,
			       ELE_SESSION_CLOSE_RSP_SZ, NULL);
	if (ret < 0)
		return ret;

	ret = se_val_rsp_hdr_n_status(dev_ctx,
				      rx_msg,
				      ELE_SESSION_CLOSE_REQ,
				      ELE_SESSION_CLOSE_RSP_SZ,
				      priv->if_defs->fw_api_ver);
	return ret;
}

int se_close_storage(struct se_if_device_ctx *dev_ctx, u32 storage_hdl)
{
	struct se_api_msg *tx_msg __free(kfree) =
		kzalloc(ELE_STORAGE_CLOSE_REQ_SZ, GFP_KERNEL);
	struct se_api_msg *rx_msg __free(kfree) =
		kzalloc(ELE_STORAGE_CLOSE_RSP_SZ, GFP_KERNEL);
	struct se_if_priv *priv;
	int ret;

	if (!dev_ctx || !dev_ctx->priv)
		return -EINVAL;

	if (!tx_msg || !rx_msg)
		return -ENOMEM;

	priv = dev_ctx->priv;

	/* Same FW-API version handling as se_close_session() above. */
	se_fill_cmd_msg_hdr(priv, (struct se_msg_hdr *)&tx_msg->header,
			    ELE_STORAGE_CLOSE_REQ, ELE_STORAGE_CLOSE_REQ_SZ, false);

	tx_msg->data[0] = storage_hdl;

	/* Transmit on the caller's own context; see se_close_session(). */
	ret = ele_msg_send_rcv(dev_ctx,
			       tx_msg,
			       ELE_STORAGE_CLOSE_REQ_SZ,
			       rx_msg,
			       ELE_STORAGE_CLOSE_RSP_SZ, NULL);
	if (ret < 0)
		return ret;

	ret = se_val_rsp_hdr_n_status(dev_ctx,
				      rx_msg,
				      ELE_STORAGE_CLOSE_REQ,
				      ELE_STORAGE_CLOSE_RSP_SZ,
				      priv->if_defs->fw_api_ver);
	return ret;
}
