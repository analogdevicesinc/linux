// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2025 NXP
 */

#include "ele_base_msg.h"
#include "ele_common.h"

/**
 * se_update_msg_chksum() - calculate and update message checksum word.
 * @msg: message buffer.
 * @msg_len: message length in bytes.
 *
 * The message length must be 4-byte aligned. The last word is treated as the
 * checksum field and is not included in the checksum calculation.
 *
 * Return: 0 on success, negative errno on failure.
 */
int se_update_msg_chksum(u32 *msg, u32 msg_len)
{
	u32 nb_words;
	u32 chksum = 0;
	u32 i;

	if (!msg)
		return -EINVAL;

	if (msg_len % SE_MSG_WORD_SZ) {
		pr_err("Msg-len is not 4-byte aligned.\n");
		return -EINVAL;
	}

	nb_words = msg_len / sizeof(*msg);
	if (nb_words < 5)
		return -EINVAL;

	/* Last word is the checksum word, so skip it. */
	nb_words--;

	for (i = 0; i < nb_words; i++)
		chksum ^= msg[i];

	msg[nb_words] = chksum;

	return 0;
}

/**
 * ele_msg_rcv() - wait for a response from the secure enclave.
 * @priv: pointer to the SE interface private data.
 * @se_clbk_hdl: callback handle whose completion will be signaled when the
 *               response arrives.
 *
 * Blocks until the firmware delivers a response into the buffer registered
 * in @se_clbk_hdl, or until the per-interface timeout expires.  When waiting
 * on the response path a deadline is enforced; on timeout the firmware-busy
 * circuit breaker is armed to prevent further transactions until the delayed
 * response arrives and clears it.
 *
 * Return: number of bytes received on success, negative errno on error
 * (e.g. -ETIMEDOUT, -ERESTARTSYS).
 */
int ele_msg_rcv(struct se_if_priv *priv, struct se_clbk_handle *se_clbk_hdl)
{
	bool is_rsp_wait_with_timeout = false;
	bool wait_uninterruptible = false;
	unsigned long remaining_jiffies;
	unsigned long deadline_jiffies;
	unsigned long flags;
	int ret;

	remaining_jiffies = msecs_to_jiffies(SE_RCV_MSG_DEFAULT_TIMEOUT_MS);
	if (se_clbk_hdl == &priv->waiting_rsp_clbk_hdl) {
		is_rsp_wait_with_timeout = true;
		deadline_jiffies = jiffies + remaining_jiffies;
	}

	do {
		if (is_rsp_wait_with_timeout) {
			unsigned long now = jiffies;

			if (time_after_eq(now, deadline_jiffies)) {
				/* Deadline hit: fence hung FW, like the ret==0 path. */
				spin_lock_irqsave(&se_clbk_hdl->clbk_rx_lock, flags);
				se_clbk_hdl->rx_msg = NULL;
				if (!completion_done(&se_clbk_hdl->done))
					atomic_set(&priv->fw_busy, 1);
				spin_unlock_irqrestore(&se_clbk_hdl->clbk_rx_lock, flags);
				ret = -ETIMEDOUT;
				break;
			}
			remaining_jiffies = deadline_jiffies - now;
		}

		if (wait_uninterruptible)
			ret = wait_for_completion_timeout(&se_clbk_hdl->done,
							  remaining_jiffies);
		else
			ret = wait_for_completion_interruptible_timeout(&se_clbk_hdl->done,
									remaining_jiffies);
		if (ret == -ERESTARTSYS) {
			/*
			 * Record that a signal was observed, then continue waiting non-
			 * interruptibly until the response arrives or the timeout
			 * expires. The caller can surface the interruption to userspace
			 * after the protocol transaction is brought back to a
			 * synchronized state.
			 */
			if (is_rsp_wait_with_timeout &&
			    READ_ONCE(se_clbk_hdl->rx_msg)) {
				WRITE_ONCE(se_clbk_hdl->signal_rcvd, true);
				wait_uninterruptible = true;
				continue;
			}
			break;
		}

		if (ret == 0) {
			/*
			 * The response buffer belongs to the caller of ele_msg_send_rcv()
			 * and may be freed as soon as this function returns. Clear rx_msg
			 * under clbk_rx_lock so that a late se_if_rx_callback() can
			 * observe that the waiter has timed out and must not copy into
			 * the stale buffer.
			 *
			 * If the completion has not yet been signaled, mark the firmware
			 * path busy. This acts as a circuit breaker: reject new
			 * command/response transactions until the delayed response
			 * arrives and the callback closes the breaker.
			 */

			spin_lock_irqsave(&se_clbk_hdl->clbk_rx_lock, flags);
			se_clbk_hdl->rx_msg = NULL;
			if (!completion_done(&se_clbk_hdl->done))
				atomic_set(&priv->fw_busy, 1);

			spin_unlock_irqrestore(&se_clbk_hdl->clbk_rx_lock, flags);
			ret = -ETIMEDOUT;
			dev_err(priv->dev,
				"Fatal Error: SE interface %s0, hangs indefinitely.\n",
				get_se_if_name(priv->if_defs->se_if_type));
			break;
		}
		ret = se_clbk_hdl->rx_msg_sz;
		break;
	} while (ret < 0);

	return ret;
}

/**
 * ele_msg_send() - send a message to the secure enclave over the mailbox.
 * @priv: pointer to the SE interface private data.
 * @tx_msg: buffer containing the message to send.
 * @tx_msg_sz: size of @tx_msg in bytes; must match the size field in the
 *             message header.
 *
 * Copies the message into the MU TX registers via the mailbox framework.
 * The MU controller does not retain the caller's buffer after this call
 * returns, so the caller may free @tx_msg immediately on success.
 *
 * Return: @tx_msg_sz on success, negative errno on error.
 */
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
			"User buf hdr: 0x%x, sz mismatched with input-sz (%d != %d).\n",
			*(u32 *)header, header->size << 2, tx_msg_sz);
		return -EINVAL;
	}

	/*
	 * The i.MX MU mailbox controller copies the payload words into MU
	 * registers synchronously from its send path. It does not retain the
	 * caller-provided tx_msg pointer after mbox_send_message() returns, so
	 * the caller-owned buffer may be released after a successful send.
	 */
	err = mbox_send_message(priv->tx_chan, tx_msg);
	if (err < 0) {
		dev_err(priv->dev, "Error: mbox_send_message failure.\n");
		return err;
	}

	return tx_msg_sz;
}

static void ele_msg_send_rcv_cleanup(struct se_if_priv *priv)
{
	unsigned long flags;

	spin_lock_irqsave(&priv->waiting_rsp_clbk_hdl.clbk_rx_lock, flags);
	priv->waiting_rsp_clbk_hdl.rx_msg = NULL;
	priv->waiting_rsp_clbk_hdl.rx_msg_sz = 0;
	spin_unlock_irqrestore(&priv->waiting_rsp_clbk_hdl.clbk_rx_lock, flags);
}

/**
 * ele_msg_send_rcv() - send a command and wait for the response.
 * @priv: pointer to the SE interface private data.
 * @tx_msg: buffer containing the command message to send.
 * @tx_msg_sz: size of @tx_msg in bytes.
 * @rx_msg: caller-provided buffer to receive the response into.
 * @exp_rx_msg_sz: expected response size in bytes.
 *
 * Holds the SE command lock for the duration of the exchange to prevent
 * concurrent transactions.  Signals are deferred until the protocol
 * resynchronizes; -ERESTARTSYS is returned to the caller after a clean
 * response is received if a signal arrived during the wait.
 *
 * Return: number of bytes received on success, negative errno on error.
 */
int ele_msg_send_rcv(struct se_if_priv *priv, void *tx_msg, int tx_msg_sz,
		     void *rx_msg, int exp_rx_msg_sz)
{
	unsigned long flags;
	int err;

	guard(mutex)(&priv->se_if_cmd_lock);

	if (atomic_read(&priv->fw_busy)) {
		dev_dbg(priv->dev, "ELE became unresponsive.\n");
		return -EBUSY;
	}
	reinit_completion(&priv->waiting_rsp_clbk_hdl.done);
	/* Publish rx_msg/rx_msg_sz under the lock read by se_if_rx_callback(). */
	spin_lock_irqsave(&priv->waiting_rsp_clbk_hdl.clbk_rx_lock, flags);
	priv->waiting_rsp_clbk_hdl.rx_msg_sz = exp_rx_msg_sz;
	priv->waiting_rsp_clbk_hdl.rx_msg = rx_msg;
	spin_unlock_irqrestore(&priv->waiting_rsp_clbk_hdl.clbk_rx_lock, flags);

	err = ele_msg_send(priv, tx_msg, tx_msg_sz);
	if (err < 0) {
		ele_msg_send_rcv_cleanup(priv);
		return err;
	}

	err = ele_msg_rcv(priv, &priv->waiting_rsp_clbk_hdl);

	if (priv->waiting_rsp_clbk_hdl.signal_rcvd) {
		/*
		 * Signal was deferred until the FW/kernel protocol resynchronized.
		 * On success report -ERESTARTSYS for the interrupted wait; the
		 * command is not re-sent. Keep real errors like -ETIMEDOUT.
		 */
		if (err > 0)
			err = -ERESTARTSYS;
		priv->waiting_rsp_clbk_hdl.signal_rcvd = false;
		dev_dbg(priv->dev, "Err[0x%x]:Interrupted by signal.\n", err);
	}

	ele_msg_send_rcv_cleanup(priv);

	return err;
}

static bool check_hdr_exception_for_sz(struct se_if_priv *priv,
				       struct se_msg_hdr *header)
{
	/*
	 * List of API headers that can accept a variable length response buffer.
	 */
	if (header->command == ELE_DEBUG_DUMP_REQ &&
	    header->ver == priv->if_defs->base_api_ver &&
	    header->size >= 2 && header->size <= (ELE_DEBUG_DUMP_RSP_SZ / 4))
		return true;

	return false;
}

/**
 * se_if_rx_callback() - mailbox RX callback for secure enclave messages.
 * @mbox_cl: mailbox client registered for this SE interface.
 * @msg: pointer to the received message buffer; may be NULL or an ERR_PTR.
 *
 * Dispatches the incoming message to either the command receiver (cmd_tag)
 * or the synchronous response waiter (rsp_tag).  Called from mailbox IRQ
 * context; must not sleep.
 */
void se_if_rx_callback(struct mbox_client *mbox_cl, void *msg)
{
	struct se_clbk_handle *se_clbk_hdl;
	struct device *dev = mbox_cl->dev;
	struct se_msg_hdr *header;
	bool sz_mismatch = false;
	struct se_if_priv *priv;
	unsigned long flags;
	u32 rx_msg_sz;

	priv = dev_get_drvdata(dev);
	if (!priv)
		return;

	/* The function can be called with NULL msg */
	if (IS_ERR_OR_NULL(msg)) {
		dev_err(dev, "Message is invalid\n");
		return;
	}

	header = msg;
	rx_msg_sz = header->size << 2;

	/* Incoming command: wake up the receiver if any. */
	if (header->tag == priv->if_defs->cmd_tag) {
		se_clbk_hdl = &priv->cmd_receiver_clbk_hdl;
		spin_lock_irqsave(&se_clbk_hdl->clbk_rx_lock, flags);
		if (!se_clbk_hdl->rx_msg) {
			spin_unlock_irqrestore(&se_clbk_hdl->clbk_rx_lock, flags);
			dev_warn(dev, "No command receiver registered for message: %.8x\n",
				 *((u32 *)header));
			return;
		}

		/*
		 * cmd_tag messages are delivered only to the explicitly registered
		 * command receiver. Unlike the synchronous response waiter path, the
		 * command receiver uses a dedicated long-lived buffer installed by
		 * SE_IOCTL_ENABLE_CMD_RCV and is not subject to the timeout/circuit-
		 * breaker handling used for rsp_tag messages.
		 */
		dev_dbg(dev, "Selecting cmd receiver: for mesg header:0x%x.\n",
			*(u32 *)header);

		/*
		 * Pre-allocated buffer of MAX_NVM_MSG_LEN
		 * as the NVM command are initiated by FW.
		 * Size is revealed as part of this call function.
		 */

		if (rx_msg_sz > MAX_NVM_MSG_LEN)
			sz_mismatch = true;

		/*
		 * Clamp the copy length to the pre-allocated receiver buffer (MAX_NVM_MSG_LEN).
		 */
		se_clbk_hdl->rx_msg_sz = min(rx_msg_sz, MAX_NVM_MSG_LEN);
		memcpy(se_clbk_hdl->rx_msg, msg, se_clbk_hdl->rx_msg_sz);
		complete(&se_clbk_hdl->done);
		spin_unlock_irqrestore(&se_clbk_hdl->clbk_rx_lock, flags);
		if (sz_mismatch)
			dev_err(dev,
				"CMD-RCVER NVM: hdr(0x%x) with different sz(%d != %d).\n",
				*(u32 *)header,
				(header->size << 2), rx_msg_sz);
	} else if (header->tag == priv->if_defs->rsp_tag) {
		bool exception_for_sz_mismatch = check_hdr_exception_for_sz(priv, header);
		u32 exp_rx_msg_sz;

		/*
		 * rx_msg and rx_msg_sz are owned by the sender under clbk_rx_lock.
		 * Read both under the lock: drop a late response instead of copying
		 * into freed memory, and avoid a stale size. A late response also
		 * closes the firmware-busy circuit breaker.
		 */
		se_clbk_hdl = &priv->waiting_rsp_clbk_hdl;
		spin_lock_irqsave(&se_clbk_hdl->clbk_rx_lock, flags);
		if (!se_clbk_hdl->rx_msg) {
			/* Close circuit breaker on spinlock race */
			atomic_set(&priv->fw_busy, 0);
			spin_unlock_irqrestore(&se_clbk_hdl->clbk_rx_lock, flags);
			dev_info(dev, "ELE responded (late), recovery FW available.\n");
			return;
		}
		exp_rx_msg_sz = se_clbk_hdl->rx_msg_sz;
		dev_dbg(dev, "Selecting resp waiter: for mesg header:0x%x.\n",
			*(u32 *)header);

		/*
		 * For rsp_tag traffic, the sender provides the expected response
		 * buffer size. If firmware returns a different size, clamp the copy
		 * length to the caller's buffer capacity before memcpy() and report the
		 * mismatch after dropping the spinlock.
		 */
		if (rx_msg_sz != exp_rx_msg_sz && !exception_for_sz_mismatch)
			sz_mismatch = true;

		se_clbk_hdl->rx_msg_sz = min(rx_msg_sz, exp_rx_msg_sz);
		memcpy(se_clbk_hdl->rx_msg, msg, se_clbk_hdl->rx_msg_sz);
		complete(&se_clbk_hdl->done);
		spin_unlock_irqrestore(&se_clbk_hdl->clbk_rx_lock, flags);

		if (sz_mismatch)
			dev_err(dev,
				"Rsp to CMD: hdr(0x%x) with different sz(%d != %d).\n",
				*(u32 *)header,
				(header->size << 2), exp_rx_msg_sz);
	} else {
		dev_err(dev, "Failed to select a device for message: %.8x\n",
			*((u32 *)header));
	}
}

/**
 * se_val_rsp_hdr_n_status() - validate a response message header and status.
 * @priv: pointer to the SE interface private data.
 * @msg: response message buffer to validate.
 * @msg_id: expected command identifier.
 * @sz: expected message size in bytes.
 * @version: expected API version byte from the command message header
 *           (tx_msg->header.ver); the response header->ver must match this
 *           value exactly.
 *
 * Checks the response tag, command id, API version, status word, and
 * response size. The size check is performed after the status check so that
 * a well-formed response whose FW-declared size differs from the userspace
 * buffer size still has its status extracted and its FW handle recorded.
 * Size mismatch returns -ENOSPC (not -EINVAL) so callers can distinguish it
 * from a header field mismatch and still run fw_api_specific_ops().
 *
 * Return: 0 on success, -EINVAL if tag/command/version mismatch, -EPERM if
 * the firmware status indicates a command failure, -ENOSPC if the response
 * size does not match @sz.
 */
int se_val_rsp_hdr_n_status(struct se_if_priv *priv, struct se_api_msg *msg,
			    u8 msg_id, u8 sz, u8 version)
{
	struct se_msg_hdr *header = &msg->header;
	u32 status;

	if (header->tag != priv->if_defs->rsp_tag) {
		dev_dbg(priv->dev, "MSG[0x%x] Hdr: Resp tag mismatch. (0x%x != 0x%x)\n",
			msg_id, header->tag, priv->if_defs->rsp_tag);
		return -EINVAL;
	}

	if (header->command != msg_id) {
		dev_dbg(priv->dev, "MSG Header: Cmd id mismatch. (0x%x != 0x%x)\n",
			header->command, msg_id);
		return -EINVAL;
	}

	if (header->ver != version) {
		dev_dbg(priv->dev,
			"MSG[0x%x] Hdr: API Vers mismatch. (0x%x != 0x%x)\n",
			msg_id, header->ver, version);
		return -EINVAL;
	}

	if (header->size > SE_MU_HDR_WORD_SZ && (sz >> 2) > SE_MU_HDR_WORD_SZ) {
		status = RES_STATUS(msg->data[0]);
		if (status != priv->if_defs->success_tag) {
			dev_dbg(priv->dev, "Command Id[%x], Response Failure = 0x%x\n",
				header->command, status);
			return -EPERM;
		}
	}

	if ((sz % 4) || (header->size != (sz >> 2) &&
			 !check_hdr_exception_for_sz(priv, header))) {
		dev_dbg(priv->dev, "MSG[0x%x] Hdr: Cmd size mismatch. (0x%x != 0x%x)\n",
			msg_id, header->size, (sz >> 2));
		return -ENOSPC;
	}

	return 0;
}

/**
 * se_save_imem_state() - export and save the encrypted IMEM state.
 * @priv: pointer to the SE interface private data.
 * @imem: IMEM buffer descriptor; @imem->daddr must point to a DMA-coherent
 *        buffer of at least ELE_IMEM_SIZE bytes.
 *
 * Issues an ELE_IMEM_EXPORT service-swap command to save the current IMEM
 * content into the pre-allocated DMA buffer.  Intended to be called during
 * system suspend.
 *
 * Return: 0 on success, negative errno on failure.
 */
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
		return 0;

	/*
	 * EXPORT command will save encrypted IMEM to given address,
	 * so later in resume, IMEM can be restored from the given
	 * address.
	 *
	 * Size must be at least 64 kB.
	 */
	ret = ele_service_swap(priv, imem->daddr, ELE_IMEM_SIZE, ELE_IMEM_EXPORT);
	if (ret < 0) {
		dev_err(priv->dev, "Failed to export IMEM.\n");
		imem->size = 0;
	} else if (ret > ELE_IMEM_SIZE) {
		dev_err(priv->dev, "Invalid exported IMEM size %d.\n", ret);
		imem->size = 0;
		ret = -EIO;
	} else {
		dev_dbg(priv->dev,
			"Exported %d bytes of encrypted IMEM.\n",
			ret);
		imem->size = ret;
	}

	return ret > 0 ? 0 : ret;
}

/**
 * se_restore_imem_state() - restore the encrypted IMEM state after resume.
 * @priv: pointer to the SE interface private data.
 * @imem: IMEM buffer descriptor populated by a prior se_save_imem_state()
 *        call; @imem->size must be non-zero.
 *
 * Issues an ELE_IMEM_IMPORT service-swap command to restore IMEM from the
 * saved DMA buffer, then verifies that the enclave reports
 * ELE_IMEM_STATE_OK.  Intended to be called during system resume.
 *
 * Return: 0 on success, -EIO if IMEM state is not OK after import, or
 * another negative errno on communication failure.
 */
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

	/* Check for the imem-state and imem-size before continue to
	 * restore imem state.
	 */
	if (s_info.d_addn_info.imem_state != ELE_IMEM_STATE_BAD || !imem->size)
		return 0;

	/*
	 * IMPORT command will restore IMEM from the given
	 * address, here size is the actual size returned by ELE
	 * during the export operation
	 */
	ret = ele_service_swap(priv, imem->daddr, imem->size, ELE_IMEM_IMPORT);
	if (ret) {
		dev_err(priv->dev, "Failed to import IMEM\n");
		return ret;
	}

	/*
	 * After importing IMEM, check if IMEM state is equal to 0xCA
	 * to ensure IMEM is fully loaded and
	 * ELE functionality can be used.
	 */
	ret = ele_get_info(priv, &s_info);
	if (ret) {
		dev_err(priv->dev, "Failed to get info from ELE.\n");
		return ret;
	}
	imem->state = s_info.d_addn_info.imem_state;

	if (s_info.d_addn_info.imem_state == ELE_IMEM_STATE_OK) {
		dev_dbg(priv->dev, "Successfully restored IMEM.\n");
	} else {
		dev_err(priv->dev, "Failed to restore IMEM: state=0x%02x, expected 0x%02x.\n",
			s_info.d_addn_info.imem_state, ELE_IMEM_STATE_OK);
		/*
		 * ele_get_info() succeeded (ret == 0) but the IMEM state
		 * reported by the hardware is not ELE_IMEM_STATE_OK. Return
		 * -EIO so the PM subsystem knows the enclave is non-functional
		 * after resume, instead of silently continuing with bad state.
		 */
		ret = -EIO;
	}

	return ret;
}
