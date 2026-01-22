// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2025 NXP
 */

#include "ele_base_msg.h"
#include "ele_common.h"

/*
 * se_get_msg_chksum() - to calculate checksum word by word.
 *
 * @msg : reference to the input msg-data.
 * @msg_len : reference to the input msg-data length in bytes.
 *            Includes extra 4 bytes (or 1 words) chksum.
 *
 * This function returns the checksum calculated by ORing word by word.
 *
 * Return:
 *  0: if the input length is not 4 byte aligned, or num of words < 5.
 *  chksum: calculated word by word.
 */
u32 se_get_msg_chksum(u32 *msg, u32 msg_len)
{
	u32 nb_words = msg_len / (u32)sizeof(u32);
	u32 chksum = 0;
	u32 i;

	if (nb_words < 5)
		return chksum;

	if (msg_len % SE_MSG_WORD_SZ) {
		pr_err("Msg-len is not 4-byte aligned.");
		return chksum;
	}

	/* nb_words include one checksum word, so skip it. */
	nb_words--;

	for (i = 0; i < nb_words; i++)
		chksum ^= *(msg + i);

	return chksum;
}

int ele_msg_rcv(struct se_if_priv *priv, struct se_clbk_handle *se_clbk_hdl)
{
	unsigned long timeout;
	int ret;

	do {
		timeout = MAX_SCHEDULE_TIMEOUT;

		ret = wait_for_completion_interruptible_timeout(&se_clbk_hdl->done, timeout);
		if (ret == -ERESTARTSYS) {
			if (priv->waiting_rsp_clbk_hdl.rx_msg) {
				priv->waiting_rsp_clbk_hdl.signal_rcvd = true;
				continue;
			}
			ret = -EINTR;
			break;
		}
		ret = se_clbk_hdl->rx_msg_sz;
		break;
	} while (ret < 0);

	return ret;
}

int ele_msg_send(struct se_if_priv *priv,
		 void *tx_msg,
		 int tx_msg_sz)
{
	struct se_msg_hdr *header = tx_msg;
	int err;

	/*
	 * Check that the size passed as argument matches the size
	 * carried in the message.
	 */
	if (header->size << 2 != tx_msg_sz) {
		dev_err(priv->dev,
			"User buf hdr: 0x%x, sz mismatced with input-sz (%d != %d).",
			*(u32 *)header, header->size << 2, tx_msg_sz);
		return -EINVAL;
	}

	err = mbox_send_message(priv->tx_chan, tx_msg);
	if (err < 0) {
		dev_err(priv->dev, "Error: mbox_send_message failure.\n");
		return err;
	}

	return tx_msg_sz;
}

/* API used for send/receive blocking call. */
int ele_msg_send_rcv(struct se_if_priv *priv, void *tx_msg, int tx_msg_sz,
		     void *rx_msg, int exp_rx_msg_sz)
{
	int err;

	guard(mutex)(&priv->se_if_cmd_lock);

	priv->waiting_rsp_clbk_hdl.rx_msg_sz = exp_rx_msg_sz;
	priv->waiting_rsp_clbk_hdl.rx_msg = rx_msg;

	err = ele_msg_send(priv, tx_msg, tx_msg_sz);
	if (err < 0)
		return err;

	err = ele_msg_rcv(priv, &priv->waiting_rsp_clbk_hdl);

	if (priv->waiting_rsp_clbk_hdl.signal_rcvd) {
		err = -EINTR;
		priv->waiting_rsp_clbk_hdl.signal_rcvd = false;
		dev_err(priv->dev, "Err[0x%x]:Interrupted by signal.\n", err);
	}

	return err;
}

static bool check_hdr_exception_for_sz(struct se_if_priv *priv,
				       struct se_msg_hdr *header)
{
	/*
	 * List of API(s) header that can be accepte variable length
	 * response buffer.
	 */
	if (header->command == ELE_DEBUG_DUMP_REQ &&
	    header->ver == priv->if_defs->base_api_ver &&
	    header->size >= 0 && header->size <= ELE_DEBUG_DUMP_RSP_SZ)
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

	header = msg;
	rx_msg_sz = header->size << 2;

	/* Incoming command: wake up the receiver if any. */
	if (header->tag == priv->if_defs->cmd_tag) {
		se_clbk_hdl = &priv->cmd_receiver_clbk_hdl;
		dev_dbg(dev, "Selecting cmd receiver for mesg header:0x%x.",
			*(u32 *)header);

		/*
		 * Pre-allocated buffer of MAX_NVM_MSG_LEN
		 * as the NVM command are initiated by FW.
		 * Size is revealed as part of this call function.
		 */
		if (rx_msg_sz > MAX_NVM_MSG_LEN) {
			dev_err(dev,
				"CMD-RCVER NVM: hdr(0x%x) with different sz(%d != %d).\n",
				*(u32 *)header, rx_msg_sz, se_clbk_hdl->rx_msg_sz);

			se_clbk_hdl->rx_msg_sz = MAX_NVM_MSG_LEN;
		}
		se_clbk_hdl->rx_msg_sz = rx_msg_sz;

	} else if (header->tag == priv->if_defs->rsp_tag) {
		se_clbk_hdl = &priv->waiting_rsp_clbk_hdl;
		dev_dbg(dev, "Selecting resp waiter for mesg header:0x%x.",
			*(u32 *)header);

		if (rx_msg_sz != se_clbk_hdl->rx_msg_sz &&
		    check_hdr_exception_for_sz(priv, header)) {
			dev_err(dev,
				"Rsp to CMD: hdr(0x%x) with different sz(%d != %d).\n",
				*(u32 *)header, rx_msg_sz, se_clbk_hdl->rx_msg_sz);

			se_clbk_hdl->rx_msg_sz = min(rx_msg_sz, se_clbk_hdl->rx_msg_sz);
		}
	} else {
		dev_err(dev, "Failed to select a device for message: %.8x\n",
			*((u32 *)header));
		return;
	}

	memcpy(se_clbk_hdl->rx_msg, msg, se_clbk_hdl->rx_msg_sz);

	/* Allow user to read */
	complete(&se_clbk_hdl->done);
}

int se_val_rsp_hdr_n_status(struct se_if_priv *priv, struct se_api_msg *msg,
			    u8 msg_id, u8 sz, bool is_base_api)
{
	struct se_msg_hdr *header = &msg->header;
	u32 status;

	if (header->tag != priv->if_defs->rsp_tag) {
		dev_err(priv->dev, "MSG[0x%x] Hdr: Resp tag mismatch. (0x%x != 0x%x)",
			msg_id, header->tag, priv->if_defs->rsp_tag);
		return -EINVAL;
	}

	if (header->command != msg_id) {
		dev_err(priv->dev, "MSG Header: Cmd id mismatch. (0x%x != 0x%x)",
			header->command, msg_id);
		return -EINVAL;
	}

	if ((sz % 4) || (header->size != (sz >> 2) &&
			 !check_hdr_exception_for_sz(priv, header))) {
		dev_err(priv->dev, "MSG[0x%x] Hdr: Cmd size mismatch. (0x%x != 0x%x)",
			msg_id, header->size, (sz >> 2));
		return -EINVAL;
	}

	if (is_base_api && header->ver != priv->if_defs->base_api_ver) {
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

	ret = ele_get_info(priv, &s_info);
	if (ret) {
		dev_err(priv->dev, "Failed to get info from ELE.\n");
		return ret;
	}

	/* Check for the imem-state before continue to save imem state. */
	if (s_info.d_addn_info.imem_state == ELE_IMEM_STATE_BAD)
		return -EIO;

	/*
	 * EXPORT command will save encrypted IMEM to given address,
	 * so later in resume, IMEM can be restored from the given
	 * address.
	 *
	 * Size must be at least 64 kB.
	 */
	ret = ele_service_swap(priv, imem->phyaddr, ELE_IMEM_SIZE, ELE_IMEM_EXPORT);
	if (ret < 0) {
		dev_err(priv->dev, "Failed to export IMEM.");
		imem->size = 0;
	} else {
		dev_dbg(priv->dev,
			"Exported %d bytes of encrypted IMEM.",
			ret);
		imem->size = ret;
	}

	return ret > 0 ? 0 : ret;
}

int se_restore_imem_state(struct se_if_priv *priv, struct se_imem_buf *imem)
{
	struct ele_dev_info s_info;
	int ret;

	/* get info from ELE */
	ret = ele_get_info(priv, &s_info);
	if (ret) {
		dev_err(priv->dev, "Failed to get info from ELE.");
		return ret;
	}
	imem->state = s_info.d_addn_info.imem_state;

	/* Check for the imem-state and imem-size before continue to
	 * restore imem state.
	 */
	if (s_info.d_addn_info.imem_state != ELE_IMEM_STATE_BAD || !imem->size)
		return -EIO;

	/*
	 * IMPORT command will restore IMEM from the given
	 * address, here size is the actual size returned by ELE
	 * during the export operation
	 */
	ret = ele_service_swap(priv, imem->phyaddr, imem->size, ELE_IMEM_IMPORT);
	if (ret) {
		dev_err(priv->dev, "Failed to import IMEM");
		return ret;
	}

	/*
	 * After importing IMEM, check if IMEM state is equal to 0xCA
	 * to ensure IMEM is fully loaded and
	 * ELE functionality can be used.
	 */
	ret = ele_get_info(priv, &s_info);
	if (ret) {
		dev_err(priv->dev, "Failed to get info from ELE.");
		return ret;
	}
	imem->state = s_info.d_addn_info.imem_state;

	if (s_info.d_addn_info.imem_state == ELE_IMEM_STATE_OK)
		dev_dbg(priv->dev, "Successfully restored IMEM.");
	else
		dev_err(priv->dev, "Failed to restore IMEM.");

	return ret;
}
