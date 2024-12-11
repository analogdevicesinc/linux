// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2024 NXP
 */

#include <linux/types.h>
#include <linux/completion.h>

#include "ele_common.h"
#include "v2x_base_msg.h"

/*
 * v2x_start_rng() - prepare and send the command to start
 *                   initialization of the ELE RNG context
 *
 * returns:  0 on success.
 */
int v2x_start_rng(struct se_if_priv *priv)
{
	struct se_api_msg *tx_msg __free(kfree) = NULL;
	struct se_api_msg *rx_msg __free(kfree) = NULL;
	int ret = 0;

	if (!priv) {
		ret = -EINVAL;
		goto exit;
	}

	tx_msg = kzalloc(V2X_START_RNG_REQ_MSG_SZ, GFP_KERNEL);
	if (!tx_msg) {
		ret = -ENOMEM;
		goto exit;
	}

	rx_msg = kzalloc(V2X_START_RNG_RSP_MSG_SZ, GFP_KERNEL);
	if (!rx_msg) {
		ret = -ENOMEM;
		goto exit;
	}

	ret = se_fill_cmd_msg_hdr(priv,
				  (struct se_msg_hdr *)&tx_msg->header,
				  V2X_START_RNG_REQ,
				  V2X_START_RNG_REQ_MSG_SZ,
				  true);
	if (ret)
		goto exit;

	ret = ele_msg_send_rcv(priv->priv_dev_ctx,
			       tx_msg,
			       V2X_START_RNG_REQ_MSG_SZ,
			       rx_msg,
			       V2X_START_RNG_RSP_MSG_SZ);
	if (ret < 0)
		goto exit;

	ret = se_val_rsp_hdr_n_status(priv,
				      rx_msg,
				      V2X_START_RNG_REQ,
				      V2X_START_RNG_RSP_MSG_SZ,
				      true);
	if (ret) {
		/* Initialization in progress for:
		 * P-TRNG at bit 0
		 * S-TRNG at bit 1
		 * Any of the bit is set, it in progress.
		 */
		if (rx_msg->data[1] & 0x3)
			goto exit;

		ret = -1;
	}
exit:
	return ret;
}
