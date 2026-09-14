// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2025 NXP
 */

#include "ele_base_msg.h"
#include "ele_common.h"
#include "ele_fw_api.h"
#include "se_ctrl.h"

int se_chk_tx_rsp_msg_hdr(struct se_if_device_ctx *dev_ctx, struct se_msg_hdr *header,
			  u32 tx_msg_sz)
{
	struct se_if_priv *priv = dev_ctx->priv;

	if (!header->size || header->size > MAX_WORD_SIZE)
		return -EINVAL;

	if (header->tag != priv->if_defs->rsp_tag)
		return -EINVAL;

	if (header->ver == priv->if_defs->base_api_ver)
		return -EINVAL;

	else if (header->ver == priv->if_defs->fw_api_ver)
		return ele_uapi_allowed_fw_rsp(dev_ctx, header, tx_msg_sz);

	return -EINVAL;
}

int se_chk_tx_cmd_msg_hdr(struct se_if_device_ctx *dev_ctx, struct se_msg_hdr *header,
			  u32 tx_msg_sz, u32 rx_msg_sz)
{
	struct se_if_priv *priv = dev_ctx->priv;

	if (!header->size || header->size > MAX_WORD_SIZE)
		return -EINVAL;

	if (header->tag != priv->if_defs->cmd_tag)
		return -EINVAL;

	if (header->ver == priv->if_defs->base_api_ver)
		return ele_uapi_allowed_base_cmd(dev_ctx, header, tx_msg_sz);
	else if (header->ver == priv->if_defs->fw_api_ver)
		return ele_uapi_allowed_fw_cmd(dev_ctx, header, tx_msg_sz, rx_msg_sz);

	return -EINVAL;
}

/*
 * Reject a command that embeds a DMA physical address which does not point
 * inside this context's shared-memory window. The userspace library stages all
 * command buffers in that coherent region (see get_shared_mem_slot), so any
 * address outside [dma_addr, dma_addr + size) is not one the driver handed out
 * and must not be forwarded to firmware. Absent optional buffers are encoded as
 * a zero address and skipped; polymorphic key words are only range-checked when
 * their gating flag marks them as a plaintext-key buffer rather than an integer
 * key identifier. An address may occupy one word (FW-API, low 32 bits only) or
 * two words (some base-API commands split it into low and high halves).
 *
 * When a field also names a size word, the buffer length carried there is
 * validated too: the whole buffer [addr, addr + len) must fit inside the
 * window, not just its start address. The check is written as len > end - addr
 * (addr is already known to be < end) so it cannot overflow.
 */
int se_val_cmd_addrs(struct se_if_device_ctx *dev_ctx, struct se_api_msg *msg,
		     u32 tx_msg_sz, const struct se_cmd_addr_field *fields,
		     size_t count)
{
	const struct se_shared_mem *mem = &dev_ctx->se_shared_mem_mgmt.non_secure_mem;
	u32 payload_words;
	size_t i;
	u64 base, end;

	if (!fields || !count)
		return 0;

	if (!msg)
		return -EINVAL;

	/* Number of complete u32 payload words present after the header. */
	if (tx_msg_sz < SE_MU_HDR_SZ)
		return -EINVAL;
	/*
	 * The caller-supplied byte count must agree with the size the firmware
	 * will act on (header word-size field, in 32-bit words), so a lying
	 * header cannot make us validate fewer words than are actually sent.
	 */
	if (tx_msg_sz != (u32)msg->header.size * sizeof(u32))
		return -EINVAL;
	payload_words = (tx_msg_sz - SE_MU_HDR_SZ) / sizeof(u32);

	base = (u64)mem->dma_addr;
	end = base + mem->size;

	/* A zero-sized or wrapping window can never contain a valid buffer. */
	if (end <= base)
		return -EINVAL;

	for (i = 0; i < count; i++) {
		const struct se_cmd_addr_field *f = &fields[i];
		u64 addr;

		/* Every word the field references must lie within the message. */
		if (f->lsb_idx >= payload_words)
			return -EINVAL;
		if (f->has_msb && f->msb_idx >= payload_words)
			return -EINVAL;

		if (f->flag_idx != SE_CMD_ADDR_ALWAYS) {
			bool flag_set;

			if (f->flag_idx >= payload_words)
				return -EINVAL;

			flag_set = !!(msg->data[f->flag_idx] & f->flag_mask);
			/*
			 * When the flag does not select DMA-address mode the
			 * word holds an integer key identifier; leave it alone.
			 */
			if (flag_set != f->is_addr_when_set)
				continue;
		}

		addr = msg->data[f->lsb_idx];
		if (f->has_msb)
			addr |= (u64)msg->data[f->msb_idx] << 32;

		/* Zero marks an absent optional buffer. */
		if (!addr)
			continue;

		if (addr < base || addr >= end)
			return -EACCES;

		/*
		 * The whole buffer [addr, addr + len) must fit inside the
		 * window, not just its start. addr is already >= base and
		 * < end here, so end - addr is a positive value and every
		 * "len > end - addr" comparison below cannot overflow. Where
		 * the length comes from depends on f->size_idx.
		 */
		switch (f->size_idx) {
		case SE_CMD_ADDR_DEDUCE_SZ: {
			/*
			 * No length word is carried in the payload. Instead the
			 * buffer begins with a struct fw_tag_len_vers_info
			 * header whose length field gives the total buffer byte
			 * count.
			 */
			const struct fw_tag_len_vers_info *hdr;
			u64 len;

			/*
			 * The header itself must lie inside the window before it
			 * can be read. addr is already >= base and < end, so
			 * end - addr is positive and cannot overflow.
			 */
			if (sizeof(*hdr) > end - addr)
				return -EACCES;

			/*
			 * Translate the validated DMA address to its kernel
			 * virtual alias inside the coherent shared-memory
			 * mapping before dereferencing it; a raw DMA address
			 * must never be dereferenced directly.
			 */
			hdr = (const struct fw_tag_len_vers_info *)
				(mem->ptr + (addr - base));
			len = le16_to_cpu(hdr->length);
			if (!len || len > end - addr)
				return -EACCES;
			break;
		}
		case SE_CMD_ADDR_FIXED_SIZE:
			/* buf_size: literal byte count (FW-defined constant). */
			if (!f->buf_size)
				return -EINVAL;
			if ((u64)f->buf_size > end - addr)
				return -EACCES;
			break;
		case SE_CMD_RCVR_ADDR_VAR_SIZE: {
			struct cmd_rcvr_data_info *crcvr_info =
							&dev_ctx->priv->crcvr_info;
			/*
			 * Export-response buffer: the size was supplied by FW
			 * in the preceding export command and stored per SE
			 * interface in cmd_rcvr_var_size.
			 */
			if ((u64)crcvr_info->cmd_rcvr_var_size > end - addr)
				return -EACCES;
			break;
		}
		default: {
			/* size_idx names a payload word carrying the length. */
			u64 len;

			if (f->size_idx >= payload_words)
				return -EINVAL;

			/* size_mask == 0 with a valid size_idx is a descriptor bug. */
			if (!f->size_mask)
				return -EINVAL;

			/*
			 * Widen to u64 before shifting: size_shift is u8 and
			 * shifting a u32 by >= 32 is undefined behaviour.
			 */
			len = ((u64)msg->data[f->size_idx] >> f->size_shift) & f->size_mask;
			if (len > end - addr)
				return -EACCES;
			break;
		}
		}
	}

	return 0;
}

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

static void se_mark_fw_busy(struct se_if_device_ctx *dev_ctx)
{
	struct se_if_priv *priv = dev_ctx->priv;
	struct fw_busy_info *fbusy_info = &priv->fw_busy_info;
	unsigned long flags;

	spin_lock_irqsave(&fbusy_info->fw_busy_lock, flags);
	if (!fbusy_info->fw_busy_dev_ctx) {
		kref_get(&dev_ctx->refcount);
		fbusy_info->fw_busy_dev_ctx = dev_ctx;
		/*
		 * Snapshot the devname now, while it is still valid. If the
		 * userspace fd is closed while the breaker is armed,
		 * cleanup_dev_ctx() frees dev_ctx->devname and sets it to NULL,
		 * so se_clear_fw_busy() cannot rely on it for its late-response
		 * diagnostics. devname is short ("%s0_ch%d"); strscpy() safely
		 * truncates into the fixed buffer.
		 */
		if (dev_ctx->devname)
			strscpy(fbusy_info->devname, dev_ctx->devname,
				sizeof(fbusy_info->devname));
		else
			fbusy_info->devname[0] = '\0';
		atomic_set(&fbusy_info->fw_busy, 1);
	}
	spin_unlock_irqrestore(&fbusy_info->fw_busy_lock, flags);
}

void set_se_rcv_msg_timeout(struct se_if_device_ctx *dev_ctx, u32 timeout_ms)
{
	dev_ctx->rcv_msg_timeout_jiffies = msecs_to_jiffies(timeout_ms);
}

/**
 * ele_msg_rcv() - wait for a response from the secure enclave.
 * @dev_ctx: pointer to the SE dev context data.
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
int ele_msg_rcv(struct se_if_device_ctx *dev_ctx, struct se_clbk_handle *se_clbk_hdl)
{
	struct se_if_priv *priv = dev_ctx->priv;
	bool is_rsp_wait_with_timeout = false;
	bool wait_uninterruptible = false;
	bool wait_killable = false;
	unsigned long remaining_jiffies;
	unsigned long deadline_jiffies;
	unsigned long flags;
	int ret;

	remaining_jiffies = dev_ctx->rcv_msg_timeout_jiffies;
	if (se_clbk_hdl == &priv->waiting_rsp_clbk_hdl) {
		is_rsp_wait_with_timeout = true;
		deadline_jiffies = jiffies + remaining_jiffies;

		/*
		 * Internal kernel transactions run on priv_dev_ctx (probe
		 * get_info/ping, FW auth, PM IMEM swap). They are not tied to a
		 * restartable syscall, so wait uninterruptibly: PM freezer fake
		 * signals must not abort them with -ERESTARTSYS. Userspace
		 * waiters stay interruptible via the deferred-signal path below.
		 */
		if (se_clbk_hdl->dev_ctx == priv->priv_dev_ctx)
			wait_uninterruptible = true;
	}

	do {
		if (is_rsp_wait_with_timeout) {
			unsigned long now = jiffies;

			if (time_after_eq(now, deadline_jiffies)) {
				/* Deadline hit: fence hung FW, like the ret==0 path. */
				spin_lock_irqsave(&se_clbk_hdl->clbk_rx_lock, flags);
				se_clbk_hdl->rx_msg = NULL;
				/* rx_delivered is set only after a real response has
				 * been copied under clbk_rx_lock, so it correctly
				 * distinguishes a genuine timeout (no response → mark
				 * busy) from a spurious teardown-forced wakeup where
				 * the data is not yet safe to free.
				 */
				if (!se_clbk_hdl->rx_delivered)
					se_mark_fw_busy(dev_ctx);
				spin_unlock_irqrestore(&se_clbk_hdl->clbk_rx_lock, flags);
				ret = -ETIMEDOUT;
				break;
			}
			remaining_jiffies = deadline_jiffies - now;
		}

		if (wait_uninterruptible)
			ret = wait_for_completion_timeout(&se_clbk_hdl->done,
							  remaining_jiffies);
		else if (wait_killable)
			ret = wait_for_completion_killable_timeout(&se_clbk_hdl->done,
								   remaining_jiffies);
		else
			ret = wait_for_completion_interruptible_timeout(&se_clbk_hdl->done,
									remaining_jiffies);
		if (ret == -ERESTARTSYS) {
			/*
			 * First, non-fatal signal on the interruptible userspace
			 * path: defer it. Record that a signal was observed and keep
			 * waiting - now only killably - until the response arrives or
			 * the timeout expires. ele_msg_send_rcv() then surfaces the
			 * interruption to userspace as -ERESTARTSYS once the protocol
			 * transaction has resynchronised, so the in-flight command is
			 * neither abandoned nor re-sent.
			 *
			 * Waiting killably rather than fully uninterruptibly is what
			 * keeps a fatal signal (SIGKILL) able to terminate the task:
			 * a non-fatal signal no longer aborts the wait, but the task
			 * can never get stuck for the multi-thousand-second long
			 * timeout and trip the hung-task watchdog.
			 */
			if (is_rsp_wait_with_timeout && !wait_killable &&
			    READ_ONCE(se_clbk_hdl->rx_msg)) {
				WRITE_ONCE(se_clbk_hdl->signal_rcvd, true);
				wait_killable = true;
				continue;
			}

			/*
			 * Command-receiver path, or a fatal signal on the
			 * killable path: the task is dying but the enclave may
			 * still DMA into the soon-to-be-freed buffer. Under
			 * clbk_rx_lock, drop rx_msg and arm fw_busy so a late
			 * callback cannot write freed memory. Exception: if
			 * rx_delivered is set a real response already landed, so
			 * report its size and keep the handle.
			 */
			if (is_rsp_wait_with_timeout) {
				spin_lock_irqsave(&se_clbk_hdl->clbk_rx_lock, flags);
				if (se_clbk_hdl->rx_delivered) {
					ret = se_clbk_hdl->rx_msg_sz;
					spin_unlock_irqrestore(&se_clbk_hdl->clbk_rx_lock, flags);
					break;
				}
				if (se_clbk_hdl->rx_msg) {
					se_clbk_hdl->rx_msg = NULL;
					se_mark_fw_busy(dev_ctx);
				}
				spin_unlock_irqrestore(&se_clbk_hdl->clbk_rx_lock, flags);
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
			/*
			 * rx_delivered helps to decide if the circuit breaker is armed
			 * or not. rx_delivered is set only after a real response has
			 * been copied under clbk_rx_lock, so it correctly distinguishes
			 * a genuine timeout (no response → mark busy) from a spurious
			 * teardown-forced wakeup where the data is not yet safe to free.
			 */
			if (!se_clbk_hdl->rx_delivered)
				se_mark_fw_busy(dev_ctx);

			spin_unlock_irqrestore(&se_clbk_hdl->clbk_rx_lock, flags);
			ret = -ETIMEDOUT;
			dev_err(priv->dev,
				"Fatal Error: SE interface %s0, hangs indefinitely.\n",
				get_se_if_name(priv->if_defs->se_if_type));
			break;
		}

		/*
		 * A positive wait return normally means a real response. During
		 * teardown, se_if_probe_cleanup() forces this wait to return via
		 * complete_all() with no response, while the enclave may still
		 * DMA into the shared buffer. Treat that as a failed transaction
		 * and arm the circuit breaker so the buffer is quarantined, not
		 * freed.
		 *
		 * rx_delivered tells the two apart: se_if_rx_callback() sets it
		 * under clbk_rx_lock only after copying a real response. This
		 * keeps teardown-time session/storage close responses from being
		 * mistaken for the forced abort, which would fail the close and
		 * leak its DMA buffer.
		 */
		spin_lock_irqsave(&se_clbk_hdl->clbk_rx_lock, flags);
		if (is_rsp_wait_with_timeout && atomic_read(&priv->going_away) &&
		    !se_clbk_hdl->rx_delivered) {
			se_clbk_hdl->rx_msg = NULL;
			se_mark_fw_busy(dev_ctx);
			spin_unlock_irqrestore(&se_clbk_hdl->clbk_rx_lock, flags);
			ret = -ENODEV;
			break;
		}
		spin_unlock_irqrestore(&se_clbk_hdl->clbk_rx_lock, flags);

		ret = se_clbk_hdl->rx_msg_sz;
		break;

	} while (ret < 0);

	return ret;
}

/**
 * ele_msg_send() - send a message to the secure enclave over the mailbox.
 * @dev_ctx: pointer to the SE device context.
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
int ele_msg_send(struct se_if_device_ctx *dev_ctx,
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
		dev_err(dev_ctx->priv->dev,
			"%s: User buf hdr: 0x%x, sz mismatched with input-sz (%d != %d).\n",
			dev_ctx->devname, *(u32 *)header, header->size << 2, tx_msg_sz);
		return -EINVAL;
	}

	/*
	 * The i.MX MU mailbox controller copies the payload words into MU
	 * registers synchronously from its send path. It does not retain the
	 * caller-provided tx_msg pointer after mbox_send_message() returns, so
	 * the caller-owned buffer may be released after a successful send.
	 */
	err = mbox_send_message(dev_ctx->priv->tx_chan, tx_msg);
	if (err < 0) {
		dev_err(dev_ctx->priv->dev,
			"%s: Error: mbox_send_message failure.\n", dev_ctx->devname);
		return err;
	}

	return tx_msg_sz;
}

static void ele_msg_send_rcv_cleanup(struct se_if_priv *priv, int *act_rx_msg_sz)
{
	unsigned long flags;

	spin_lock_irqsave(&priv->waiting_rsp_clbk_hdl.clbk_rx_lock, flags);
	priv->waiting_rsp_clbk_hdl.dev_ctx = NULL;
	priv->waiting_rsp_clbk_hdl.rx_msg = NULL;
	if (act_rx_msg_sz)
		*act_rx_msg_sz = priv->waiting_rsp_clbk_hdl.rx_msg_sz;
	priv->waiting_rsp_clbk_hdl.rx_msg_sz = 0;
	spin_unlock_irqrestore(&priv->waiting_rsp_clbk_hdl.clbk_rx_lock, flags);
}

/**
 * ele_msg_send_rcv() - send a command and wait for the response.
 * @dev_ctx: pointer to the dev_ctx data.
 * @tx_msg: buffer containing the command message to send.
 * @tx_msg_sz: size of @tx_msg in bytes.
 * @rx_msg: caller-provided buffer to receive the response into.
 * @exp_rx_msg_sz: expected response size in bytes.
 * @act_rx_msg_sz: optional output pointer; if non-NULL, receives the actual
 *                 number of bytes copied into @rx_msg (min of FW-declared
 *                 size and @exp_rx_msg_sz). Pass NULL when the caller does not
 *                 need this value.
 *
 * Holds the SE command lock for the duration of the exchange to prevent
 * concurrent transactions.  Signals are deferred until the protocol
 * resynchronizes; -ERESTARTSYS is returned to the caller after a clean
 * response is received if a signal arrived during the wait.
 *
 * Return: number of bytes received on success, negative errno on error.
 */
int ele_msg_send_rcv(struct se_if_device_ctx *dev_ctx, void *tx_msg,
		     int tx_msg_sz, void *rx_msg, int exp_rx_msg_sz, int *act_rx_msg_sz)
{
	struct se_if_priv *priv = dev_ctx->priv;
	struct fw_busy_info *fbusy_info = &priv->fw_busy_info;
	struct task_struct *msg_excl_owner;
	unsigned long flags;
	int err;

	guard(mutex)(&priv->se_if_cmd_lock);

	/*
	 * Arm under clbk_rx_lock so the going_away check and arming are atomic
	 * against teardown (closes the lost-wakeup window); priv_dev_ctx close
	 * commands still pass. Check going_away before fw_busy so a caller
	 * racing unbind gets the permanent -ENODEV, not the retryable -EBUSY -
	 * these are deliberately distinct from the transient msg_if reservation.
	 */

	spin_lock_irqsave(&priv->waiting_rsp_clbk_hdl.clbk_rx_lock, flags);
	msg_excl_owner = READ_ONCE(priv->msg_excl_flow.msg_excl_owner);
	if (atomic_read(&priv->going_away) && msg_excl_owner != current) {
		spin_unlock_irqrestore(&priv->waiting_rsp_clbk_hdl.clbk_rx_lock, flags);
		return -ENODEV;
	}
	/*
	 * fw_busy is the circuit breaker: while it is set, reject new
	 * transactions with -EBUSY. The one exception is a flow that has
	 * reserved this interface exclusively for itself via se_reserve_msg_if()
	 * by publishing its task in priv->msg_excl_flow.msg_excl_owner (e.g.
	 * the recovery flow in se_clear_fw_busy()). Only that owning task is let
	 * through here to issue its teardown-close messages; every other caller
	 * still gets -EBUSY. When the flow calls se_release_msg_if() the owner
	 * is cleared and the interface returns to general se_if_cmd_lock message
	 * exchange. The owner is only compared against current, so a stale read
	 * is harmless (a non-owner can never match) and no msg_excl_lock is
	 * needed here; there is no deadlock.
	 */
	if (atomic_read(&fbusy_info->fw_busy) && msg_excl_owner != current) {
		spin_unlock_irqrestore(&priv->waiting_rsp_clbk_hdl.clbk_rx_lock, flags);
		return -EBUSY;
	}

	reinit_completion(&priv->waiting_rsp_clbk_hdl.done);
	priv->waiting_rsp_clbk_hdl.dev_ctx = dev_ctx;
	priv->waiting_rsp_clbk_hdl.rx_msg_sz = exp_rx_msg_sz;
	priv->waiting_rsp_clbk_hdl.rx_msg = rx_msg;
	/*
	 * Arm a fresh transaction: clear the delivered flag so a stale value
	 * from a previous response cannot make ele_msg_rcv() mistake a
	 * teardown-forced complete_all() for a genuine firmware response.
	 */
	priv->waiting_rsp_clbk_hdl.rx_delivered = false;
	spin_unlock_irqrestore(&priv->waiting_rsp_clbk_hdl.clbk_rx_lock, flags);

	err = ele_msg_send(dev_ctx, tx_msg, tx_msg_sz);
	if (err < 0) {
		ele_msg_send_rcv_cleanup(priv, NULL);
		return err;
	}

	err = ele_msg_rcv(dev_ctx, &priv->waiting_rsp_clbk_hdl);

	if (priv->waiting_rsp_clbk_hdl.signal_rcvd) {
		/*
		 * Signal was deferred until the FW/kernel protocol resynchronized.
		 * On success report -ERESTARTSYS for the interrupted wait; the
		 * command is not re-sent. Keep real errors like -ETIMEDOUT.
		 */
		if (err > 0)
			err = -ERESTARTSYS;
		priv->waiting_rsp_clbk_hdl.signal_rcvd = false;
		dev_dbg(priv->dev, "%s: Err[0x%x]:Interrupted by signal.\n",
			dev_ctx->devname, err);
	}

	ele_msg_send_rcv_cleanup(priv, act_rx_msg_sz);

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
	struct fw_busy_info *fbusy_info;
	struct se_msg_hdr *header;
	bool sz_mismatch = false;
	struct se_if_priv *priv;
	/*
	 * devname_snap: a local copy of dev_ctx->devname taken while
	 * clbk_rx_lock is held.
	 */
	char devname_snap[32];
	unsigned long flags;
	u32 rx_msg_sz;

	priv = dev_get_drvdata(dev);
	if (!priv)
		return;

	fbusy_info = &priv->fw_busy_info;

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
		if (!se_clbk_hdl->dev_ctx || !se_clbk_hdl->rx_msg) {
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
		dev_dbg(dev, "Selecting cmd receiver:%s for mesg header:0x%x.\n",
			se_clbk_hdl->dev_ctx->devname,  *(u32 *)header);

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
		strscpy(devname_snap, se_clbk_hdl->dev_ctx->devname,
			sizeof(devname_snap));
		memcpy(se_clbk_hdl->rx_msg, msg, se_clbk_hdl->rx_msg_sz);
		complete(&se_clbk_hdl->done);
		spin_unlock_irqrestore(&se_clbk_hdl->clbk_rx_lock, flags);
		if (sz_mismatch)
			dev_err(dev,
				"%s: CMD-RCVER NVM: hdr(0x%x) with different sz(%d != %d).\n",
				devname_snap, *(u32 *)header,
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
			/*
			 * Schedule fw_busy_work only while going_away is clear:
			 * teardown sets going_away under this lock before
			 * cancel_work_sync(), so scheduling after that would
			 * re-queue against freed priv (UAF). schedule_work()
			 * only enqueues (the handler runs later in process
			 * context, so se_clear_fw_busy()'s mutex is not taken
			 * under this spinlock). se_clear_fw_busy() also runs
			 * from teardown; both take fw_busy_lock first, so the
			 * second caller sees NULL and returns.
			 */
			if (atomic_read(&fbusy_info->fw_busy)) {
				/*
				 * Snapshot the late response before scheduling
				 * fw_busy_work. se_clear_fw_busy() will parse
				 * this buffer to detect a session-open or
				 * storage-open response and immediately close
				 * the leaked firmware handle via
				 * fw_api_specific_ops(). The buffer is sized
				 * to MAX_ALLOWED_RX_MSG_SZ; clamp the copy
				 * length so an oversized FW message cannot
				 * overflow it.
				 */
				memcpy(fbusy_info->orphan_fw_rx_msg, msg,
				       min(rx_msg_sz,
					   (u32)MAX_ALLOWED_RX_MSG_SZ));
				if (!atomic_read(&priv->going_away))
					schedule_work(&fbusy_info->fw_busy_work);
			}
			spin_unlock_irqrestore(&se_clbk_hdl->clbk_rx_lock, flags);

			dev_info(dev, "ELE responded (late), recovery FW available.\n");
			return;
		}
		exp_rx_msg_sz = se_clbk_hdl->rx_msg_sz;
		dev_dbg(dev, "Selecting resp waiter:%s for mesg header:0x%x.\n",
			se_clbk_hdl->dev_ctx->devname, *(u32 *)header);

		/*
		 * For rsp_tag traffic, the sender provides the expected response
		 * buffer size. If firmware returns a different size, clamp the copy
		 * length to the caller's buffer capacity before memcpy() and report the
		 * mismatch after dropping the spinlock.
		 */
		if (rx_msg_sz != exp_rx_msg_sz && !exception_for_sz_mismatch)
			sz_mismatch = true;

		se_clbk_hdl->rx_msg_sz = min(rx_msg_sz, exp_rx_msg_sz);
		/* Snapshot devname before complete() can free the context. */
		strscpy(devname_snap, se_clbk_hdl->dev_ctx->devname,
			sizeof(devname_snap));
		memcpy(se_clbk_hdl->rx_msg, msg, se_clbk_hdl->rx_msg_sz);
		/*
		 * Mark that a genuine firmware response was delivered. ele_msg_rcv()
		 * reads this under clbk_rx_lock to avoid mistaking this response for
		 * a teardown-forced complete_all() wakeup.
		 */
		se_clbk_hdl->rx_delivered = true;
		complete(&se_clbk_hdl->done);
		spin_unlock_irqrestore(&se_clbk_hdl->clbk_rx_lock, flags);

		if (sz_mismatch)
			dev_err(dev,
				"%s: Rsp to CMD: hdr(0x%x) with different sz(%d != %d).\n",
				devname_snap, *(u32 *)header,
				(header->size << 2), exp_rx_msg_sz);
	} else {
		dev_err(dev, "Failed to select a device for message: %.8x\n",
			*((u32 *)header));
	}
}

/**
 * se_val_rsp_hdr_n_status() - validate a response message header and status.
 * @dev_ctx: pointer to the SE device context.
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
int se_val_rsp_hdr_n_status(struct se_if_device_ctx *dev_ctx, struct se_api_msg *msg,
			    u8 msg_id, u8 sz, u8 version)
{
	struct se_if_priv *priv = dev_ctx->priv;
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
