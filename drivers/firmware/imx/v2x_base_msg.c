// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2024 NXP
 */

#include <linux/types.h>
#include <linux/completion.h>

#include "ele_common.h"
#include "v2x_base_msg.h"

#define FW_DBG_DUMP_FIXED_STR		"\nS40X: "

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

/*
 * v2x_pwr_state() - prepare and send the command to change
 *                   the power state of V2X-FW
 *
 * returns:  0 on success.
 */
int v2x_pwr_state(struct se_if_priv *priv, u16 action)
{
	struct se_api_msg *tx_msg __free(kfree) = NULL;
	struct se_api_msg *rx_msg __free(kfree) = NULL;
	int ret = 0;

	if (!priv) {
		ret = -EINVAL;
		goto exit;
	}

	tx_msg = kzalloc(V2X_PWR_STATE_MSG_SZ, GFP_KERNEL);
	if (!tx_msg) {
		ret = -ENOMEM;
		goto exit;
	}

	rx_msg = kzalloc(V2X_PWR_STATE_RSP_MSG_SZ, GFP_KERNEL);
	if (!rx_msg) {
		ret = -ENOMEM;
		goto exit;
	}

	ret = se_fill_cmd_msg_hdr(priv,
				  (struct se_msg_hdr *)&tx_msg->header,
				  V2X_PWR_STATE,
				  V2X_PWR_STATE_MSG_SZ,
				  true);
	if (ret)
		goto exit;

	tx_msg->data[0] = action;

	ret = ele_msg_send_rcv(priv->priv_dev_ctx,
			       tx_msg,
			       V2X_PWR_STATE_MSG_SZ,
			       rx_msg,
			       V2X_PWR_STATE_RSP_MSG_SZ);
	if (ret < 0)
		goto exit;

	ret = se_val_rsp_hdr_n_status(priv,
				      rx_msg,
				      V2X_PWR_STATE,
				      V2X_PWR_STATE_RSP_MSG_SZ,
				      true);
	if (ret == -EPERM) {
		switch (rx_msg->data[0]) {
		case V2X_PERM_DENIED_FAIL_IND:
			dev_err(priv->dev,
				"TRNG is active or HSM/SHE session is remained open.");
			break;
		case V2X_INVAL_OPS_FAIL_IND:
			dev_err(priv->dev,
				"Invalid Action.");
			break;
		default:
			dev_err(priv->dev,
				"V2X Power Ops failed[0x%x].",
				RES_STATUS(rx_msg->data[0]));
		}
	}
exit:
	return ret;
}

int v2x_debug_dump(struct se_if_priv *priv)
{
	struct se_api_msg *tx_msg __free(kfree) = NULL;
	struct se_api_msg *rx_msg __free(kfree) = NULL;
	bool keep_logging;
	u8 dump_data[408];
	u8 fmt_str[256];
	int fmt_str_idx;
	int rcv_dbg_wd_ct;
	int msg_ex_cnt;
	int ret = 0;
	int w_ct;

	if (!priv) {
		ret = -EINVAL;
		goto exit;
	}

	tx_msg = kzalloc(V2X_DBG_DUMP_MSG_SZ, GFP_KERNEL);
	if (!tx_msg) {
		ret = -ENOMEM;
		goto exit;
	}

	rx_msg = kzalloc(V2X_DBG_DUMP_RSP_MSG_SZ, GFP_KERNEL);
	if (!rx_msg) {
		ret = -ENOMEM;
		goto exit;
	}

	ret = se_fill_cmd_msg_hdr(priv,
				  &tx_msg->header,
				  V2X_DBG_DUMP_REQ,
				  V2X_DBG_DUMP_MSG_SZ,
				  true);
	if (ret)
		goto exit;

	tx_msg->header.tag = V2X_DEBUG_MU_MSG_CMD_TAG;
	tx_msg->header.ver = V2X_DEBUG_MU_MSG_VERS;
	tx_msg->data[0] = 0x1;
	msg_ex_cnt = 0;
	do {
		w_ct = 0;
		fmt_str_idx = 0;

		ret = ele_msg_send_rcv(priv->priv_dev_ctx,
				tx_msg,
				V2X_DBG_DUMP_MSG_SZ,
				rx_msg,
				V2X_DBG_DUMP_RSP_MSG_SZ);
		if (ret < 0)
			goto exit;

		ret = se_val_rsp_hdr_n_status(priv,
				rx_msg,
				V2X_DBG_DUMP_REQ,
				V2X_DBG_DUMP_RSP_MSG_SZ,
				true);
		if (!ret) {
			rcv_dbg_wd_ct = rx_msg->header.size - V2X_NON_DUMP_BUFFER_SZ;
			memcpy(fmt_str, FW_DBG_DUMP_FIXED_STR, strlen(FW_DBG_DUMP_FIXED_STR));
			fmt_str_idx += strlen(FW_DBG_DUMP_FIXED_STR);
			for (w_ct = 0; w_ct < rcv_dbg_wd_ct; w_ct++) {
				fmt_str[fmt_str_idx] = '0';
				fmt_str_idx++;
				fmt_str[fmt_str_idx] = 'x';
				fmt_str_idx++;
				fmt_str[fmt_str_idx] = '%';
				fmt_str_idx++;
				fmt_str[fmt_str_idx] = '0';
				fmt_str_idx++;
				fmt_str[fmt_str_idx] = '8';
				fmt_str_idx++;
				fmt_str[fmt_str_idx] = 'x';
				fmt_str_idx++;
				fmt_str[fmt_str_idx] = ' ';
				fmt_str_idx++;
				if (w_ct % 2) {
					memcpy(fmt_str + fmt_str_idx,
					       FW_DBG_DUMP_FIXED_STR,
					       strlen(FW_DBG_DUMP_FIXED_STR));
					fmt_str_idx += strlen(FW_DBG_DUMP_FIXED_STR);
				}
			}
			keep_logging = (rx_msg->header.size < (V2X_DBG_DUMP_RSP_MSG_SZ >> 2)) ?
					false : true;
			keep_logging = keep_logging ? (msg_ex_cnt > V2X_MAX_DBG_DMP_PKT ? false : true) : false;

			/*
			 * Number of spaces = rcv_dbg_wd_ct
			 * DBG dump length in bytes = rcv_dbg_wd_ct * 4
			 *
			 * Since, one byte is represented as 2 character,
			 * DBG Dump string-length = rcv_dbg_wd_ct * 8
			 * Fixed string's string-length =
			 * 			strlen(FW_DBG_DUMP_FIXED_STR) * rcv_dbg_wd_ct
			 *
			 * Total dump_data length = Number of spaces +
			 *  			    DBG Dump string' string-length +
			 *  			    Fixed string's string-length
			 *
			 * Total dump_data length = rcv_dbg_wd_ct + (rcv_dbg_wd_ct * 8) +
			 *  			    strlen(FW_DBG_DUMP_FIXED_STR) * rcv_dbg_wd_ct
			 */

			snprintf(dump_data,
				 ((rcv_dbg_wd_ct * 9) +
				  (strlen(FW_DBG_DUMP_FIXED_STR) * rcv_dbg_wd_ct)),
				 fmt_str,
				 rx_msg->data[1], rx_msg->data[2],
				 rx_msg->data[3], rx_msg->data[4],
				 rx_msg->data[5], rx_msg->data[6],
				 rx_msg->data[7], rx_msg->data[8],
				 rx_msg->data[9], rx_msg->data[10],
				 rx_msg->data[11], rx_msg->data[12],
				 rx_msg->data[13], rx_msg->data[14],
				 rx_msg->data[15], rx_msg->data[16],
				 rx_msg->data[17], rx_msg->data[18],
				 rx_msg->data[19], rx_msg->data[20],
				 rx_msg->data[21], rx_msg->data[22]);

			dev_err(priv->dev, "%s", dump_data);
		} else {
			dev_err(priv->dev, "Dump_Debug_Buffer Error: %x.", ret);
			break;
		}
		msg_ex_cnt++;
	} while (keep_logging);

exit:
	return ret;
}
