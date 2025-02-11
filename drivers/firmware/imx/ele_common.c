// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2024-2025 NXP
 */

#include <uapi/linux/se_ioctl.h>

#include "ele_base_msg.h"
#include "ele_common.h"
#include "se_msg_sqfl_ctrl.h"
#include "v2x_base_msg.h"

extern u32 se_rcv_msg_timeout;

u32 se_add_msg_crc(u32 *msg, u32 msg_len)
{
	u32 nb_words = msg_len / (u32)sizeof(u32);
	u32 crc = 0;
	u32 i;

	for (i = 0; i < nb_words - 1; i++)
		crc ^= *(msg + i);

	return crc;
}

int ele_msg_rcv(struct se_if_device_ctx *dev_ctx,
		struct se_clbk_handle *se_clbk_hdl)
{
	struct se_if_priv *priv = dev_ctx->priv;
	bool wait_timeout_enabled = true;
	unsigned int wait;
	int err;

	do {
		if (priv->cmd_receiver_clbk_hdl.dev_ctx == dev_ctx) {
			/* For NVM-D that are slaves of SE-FW, are waiting indefinitly
			 * to receive the command from SE-FW.
			 */
			wait_timeout_enabled = false;

			/* If callback is executed before entrying to wait state,
			 * it will immediately come out after entering the wait state,
			 * but completion_done(&se_clbk_hdl->done), will return false
			 * after exiting the wait state, with err = 0.
			 */
			err = wait_for_completion_interruptible(&se_clbk_hdl->done);
		} else {
			/* FW must send the message response to application in a finite
			 * time.
			 */
			wait = msecs_to_jiffies(se_rcv_msg_timeout);
			err = wait_for_completion_interruptible_timeout(&se_clbk_hdl->done, wait);
		}
		if (err == -ERESTARTSYS) {
			if (priv->waiting_rsp_clbk_hdl.dev_ctx) {
				priv->waiting_rsp_clbk_hdl.signal_rcvd = true;
				continue;
			}
			err = -EINTR;
			break;
		}
		if (err == 0) {
			if (wait_timeout_enabled) {
				err = -ETIMEDOUT;
				dev_err(priv->dev,
					"Fatal Error: SE interface: %s%d, hangs indefinitely.\n",
					get_se_if_name(priv->if_defs->se_if_type),
					priv->if_defs->se_instance_id);
			}
			break;
		}
	} while (err < 0);

	if (err >= 0) {
		se_dump_to_logfl(dev_ctx,
				 SE_DUMP_MU_RCV_BUFS,
				 se_clbk_hdl->rx_msg_sz,
				 (u8 *)se_clbk_hdl->rx_msg);
		err = se_clbk_hdl->rx_msg_sz;
	}

	return err;
}

int ele_msg_send(struct se_if_device_ctx *dev_ctx,
		 void *tx_msg,
		 int tx_msg_sz)
{
	struct se_if_priv *priv = dev_ctx->priv;
	struct se_msg_hdr *header;
	int err;

	header = tx_msg;

	/*
	 * Check that the size passed as argument matches the size
	 * carried in the message.
	 */
	if (header->size << 2 != tx_msg_sz) {
		err = -EINVAL;
		dev_err(priv->dev,
			"%s: User buf hdr: 0x%x, sz mismatced with input-sz (%d != %d).",
			dev_ctx->devname,
			*(u32 *)header,
			header->size << 2, tx_msg_sz);
		goto exit;
	}

	err = mbox_send_message(priv->tx_chan, tx_msg);
	if (err < 0) {
		dev_err(priv->dev,
			"%s: Error: mbox_send_message failure.",
			dev_ctx->devname);
		return err;
	}
	err = tx_msg_sz;
	se_dump_to_logfl(dev_ctx, SE_DUMP_MU_SND_BUFS, tx_msg_sz, tx_msg);

exit:
	return err;
}

/* API used for send/receive blocking call. */
int ele_msg_send_rcv(struct se_if_device_ctx *dev_ctx,
		     void *tx_msg,
		     int tx_msg_sz,
		     void *rx_msg,
		     int exp_rx_msg_sz)
{
	int err;
	struct se_if_priv *priv = dev_ctx->priv;

	se_qualify_msg_seq_flow(&priv->se_msg_sq_ctl, tx_msg);

	guard(mutex)(&priv->se_if_cmd_lock);

	/* Capture request timer */
	ktime_get_ts64(&priv->time_frame.t_start);
	priv->waiting_rsp_clbk_hdl.dev_ctx = dev_ctx;
	priv->waiting_rsp_clbk_hdl.rx_msg_sz = exp_rx_msg_sz;
	priv->waiting_rsp_clbk_hdl.rx_msg = rx_msg;

	err = ele_msg_send(dev_ctx, tx_msg, tx_msg_sz);
	if (err < 0)
		goto exit;

	err = ele_msg_rcv(dev_ctx, &priv->waiting_rsp_clbk_hdl);

	if (priv->waiting_rsp_clbk_hdl.signal_rcvd) {
		err = -EINTR;
		priv->waiting_rsp_clbk_hdl.signal_rcvd = false;
		dev_err(priv->dev,
			"%s: Err[0x%x]:Interrupted by signal.\n",
			dev_ctx->devname,
			err);
	}
	priv->waiting_rsp_clbk_hdl.dev_ctx = NULL;

	/* Capture response timer */
	ktime_get_ts64(&priv->time_frame.t_end);
exit:
	return err;
}

static bool exception_for_size(struct se_if_priv *priv,
				struct se_msg_hdr *header)
{
	/* List of API(s) that can be accepte variable length
	 * response buffer.
	 */
	if ((header->command == ELE_DEBUG_DUMP_REQ || header->command == V2X_DBG_DUMP_REQ) &&
		header->ver == priv->if_defs->base_api_ver &&
		header->size >= 0 &&
		header->size <= ELE_DEBUG_DUMP_RSP_SZ)
		return true;

	return false;
}

/*
 * Callback called by mailbox FW, when data is received.
 */
void se_if_rx_callback(struct mbox_client *mbox_cl, void *msg)
{
	struct se_clbk_handle *se_clbk_hdl;
	struct device *dev = mbox_cl->dev;
	struct se_msg_hdr *header;
	struct se_if_priv *priv;
	u32 rx_msg_sz;

	priv = dev_get_drvdata(dev);

	/* The function can be called with NULL msg */
	if (!msg) {
		dev_err(dev, "Message is invalid\n");
		return;
	}

	if ((((uint32_t *)msg)[1] & 0xFFFF) == ELE_ABORT_ERR_CODE)
		se_dump_to_logfl(priv->priv_dev_ctx,
				 SE_DUMP_KDEBUG_BUFS, 0,
				 "Rx-Msg(0x%x): Fatal abort received  by %s.\n",
				 ((uint32_t *)msg)[0], priv->priv_dev_ctx->devname);

	header = msg;
	rx_msg_sz = header->size << 2;

	if (priv->if_defs->se_if_type == SE_TYPE_ID_V2X_DBG &&
			header->tag == V2X_DBG_MU_MSG_RSP_TAG) {
		header->tag = priv->if_defs->rsp_tag;
		header->ver = priv->if_defs->base_api_ver;
	}

	/* Incoming command: wake up the receiver if any. */
	if (header->tag == priv->if_defs->cmd_tag) {
		se_clbk_hdl = &priv->cmd_receiver_clbk_hdl;
		dev_dbg(dev,
			"Selecting cmd receiver:%s for mesg header:0x%x.",
			se_clbk_hdl->dev_ctx->devname,
			*(u32 *) header);

		/* Pre-allocated buffer of MAX_NVM_MSG_LEN
		 * as the NVM command are initiated by FW.
		 * Size is revealed as part of this call function.
		 */
		if (rx_msg_sz > MAX_NVM_MSG_LEN) {
			dev_err(dev,
				"%s: CMD-RCVER NVM: hdr(0x%x) with different sz(%d != %d).\n",
				se_clbk_hdl->dev_ctx->devname,
				*(u32 *) header,
				rx_msg_sz, se_clbk_hdl->rx_msg_sz);

			se_clbk_hdl->rx_msg_sz = MAX_NVM_MSG_LEN;
		}
		se_clbk_hdl->rx_msg_sz = rx_msg_sz;

	} else if (header->tag == priv->if_defs->rsp_tag) {
		se_clbk_hdl = &priv->waiting_rsp_clbk_hdl;
		dev_dbg(dev,
			"Selecting resp waiter:%s for mesg header:0x%x.",
			se_clbk_hdl->dev_ctx->devname,
			*(u32 *) header);

		if (rx_msg_sz != se_clbk_hdl->rx_msg_sz
				&& !exception_for_size(priv, header)) {
			dev_err(dev,
				"%s: Rsp to CMD: hdr(0x%x) with different sz(%d != %d).\n",
				se_clbk_hdl->dev_ctx->devname,
				*(u32 *) header,
				rx_msg_sz, se_clbk_hdl->rx_msg_sz);

			se_clbk_hdl->rx_msg_sz = min(rx_msg_sz, se_clbk_hdl->rx_msg_sz);
		}
	} else {
		dev_err(dev, "Failed to select a device for message: %.8x\n",
			*((u32 *) header));
		return;
	}

	memcpy(se_clbk_hdl->rx_msg, msg, se_clbk_hdl->rx_msg_sz);

	/* Allow user to read */
	complete(&se_clbk_hdl->done);
}

int se_val_rsp_hdr_n_status(struct se_if_priv *priv,
			    struct se_api_msg *msg,
			    uint8_t msg_id,
			    uint8_t sz,
			    bool is_base_api)
{
	u32 status;
	struct se_msg_hdr *header = &msg->header;

	if (header->tag != priv->if_defs->rsp_tag) {
		dev_err(priv->dev,
			"MSG[0x%x] Hdr: Resp tag mismatch. (0x%x != 0x%x)",
			msg_id, header->tag, priv->if_defs->rsp_tag);
		return -EINVAL;
	}

	if (header->command != msg_id) {
		dev_err(priv->dev,
			"MSG Header: Cmd id mismatch. (0x%x != 0x%x)",
			header->command, msg_id);
		return -EINVAL;
	}

	if (header->size != (sz >> 2) && !exception_for_size(priv, header)) {
		dev_err(priv->dev,
			"MSG[0x%x] Hdr: Cmd size mismatch. (0x%x != 0x%x)",
			msg_id, header->size, (sz >> 2));
		return -EINVAL;
	}

	if (is_base_api && (header->ver != priv->if_defs->base_api_ver)) {
		dev_err(priv->dev,
			"MSG[0x%x] Hdr: Base API Vers mismatch. (0x%x != 0x%x)",
			msg_id, header->ver, priv->if_defs->base_api_ver);
		return -EINVAL;
	} else if (!is_base_api && header->ver != priv->if_defs->fw_api_ver) {
		dev_err(priv->dev,
			"MSG[0x%x] Hdr: FW API Vers mismatch. (0x%x != 0x%x)",
			msg_id, header->ver, priv->if_defs->fw_api_ver);
		return -EINVAL;
	}

	status = RES_STATUS(msg->data[0]);
	if (status != priv->if_defs->success_tag) {
		dev_err(priv->dev, "Command Id[%x], Response Failure = 0x%x",
			header->command, status);
		return -EPERM;
	}

	return 0;
}

int se_save_imem_state(struct se_if_priv *priv, struct se_imem_buf *imem)
{
	struct ele_dev_info s_info = {0};
	int ret;

	/* get info from ELE */
	ret = ele_get_info(priv, &s_info);
	if (ret) {
		dev_err(priv->dev, "Failed to get info from ELE.\n");
		return ret;
	}

	/* Do not save the IMEM buffer, if the current IMEM state is BAD. */
	if (s_info.d_addn_info.imem_state == ELE_IMEM_STATE_BAD)
		return ret;

	/* EXPORT command will save encrypted IMEM to given address,
	 * so later in resume, IMEM can be restored from the given
	 * address.
	 *
	 * Size must be at least 64 kB.
	 */
	ret = ele_service_swap(priv,
			       imem->phyaddr,
			       ELE_IMEM_SIZE,
			       ELE_IMEM_EXPORT);
	if (ret < 0) {
		dev_err(priv->dev, "Failed to export IMEM\n");
		imem->size = 0;
	} else {
		dev_info(priv->dev,
			 "Exported %d bytes of encrypted IMEM\n",
			 ret);
		imem->size = ret;
	}

	return ret > 0 ? 0 : -1;
}

int se_restore_imem_state(struct se_if_priv *priv, struct se_imem_buf *imem)
{
	struct ele_dev_info s_info;
	int ret;

	/* get info from ELE */
	ret = ele_get_info(priv, &s_info);
	if (ret) {
		dev_err(priv->dev, "Failed to get info from ELE.\n");
		return ret;
	}
	imem->state = s_info.d_addn_info.imem_state;

	/* Get IMEM state, if 0xFE and saved/exported IMEM buffer size is non-zero,
	 * then import IMEM
	 */
	if (s_info.d_addn_info.imem_state == ELE_IMEM_STATE_BAD && imem->size) {
		/* IMPORT command will restore IMEM from the given
		 * address, here size is the actual size returned by ELE
		 * during the export operation
		 */
		ret = ele_service_swap(priv,
				       imem->phyaddr,
				       imem->size,
				       ELE_IMEM_IMPORT);
		if (ret) {
			dev_err(priv->dev, "Failed to import IMEM\n");
			goto exit;
		}
	} else
		goto exit;

	/* After importing IMEM, check if IMEM state is equal to 0xCA
	 * to ensure IMEM is fully loaded and
	 * ELE functionality can be used.
	 */
	ret = ele_get_info(priv, &s_info);
	if (ret) {
		dev_err(priv->dev, "Failed to get info from ELE.\n");
		goto exit;
	}
	imem->state = s_info.d_addn_info.imem_state;

	if (s_info.d_addn_info.imem_state == ELE_IMEM_STATE_OK)
		dev_info(priv->dev, "Successfully restored IMEM\n");
	else
		dev_err(priv->dev, "Failed to restore IMEM\n");

exit:
	return ret;
}
