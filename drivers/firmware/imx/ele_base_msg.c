// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2024 NXP
 */

#include <linux/types.h>

#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/genalloc.h>
#include <linux/firmware/imx/se_api.h>

#include "ele_base_msg.h"
#include "ele_common.h"
#include "se_msg_sqfl_ctrl.h"

#define FW_DBG_DUMP_FIXED_STR		"\nELEX: "

int ele_get_info(struct se_if_priv *priv, struct ele_dev_info *s_info)
{
	struct se_api_msg *tx_msg __free(kfree) = NULL;
	struct se_api_msg *rx_msg __free(kfree) = NULL;
	dma_addr_t get_info_addr = 0;
	u32 *get_info_data = NULL;
	int ret = 0;

	if (!priv) {
		ret = -EINVAL;
		return ret;
	}

	memset(s_info, 0x0, sizeof(*s_info));

	if (priv->mem_pool)
		get_info_data = gen_pool_dma_alloc(priv->mem_pool,
						   ELE_GET_INFO_BUFF_SZ,
						   &get_info_addr);
	else
		get_info_data = dma_alloc_coherent(priv->dev,
						   ELE_GET_INFO_BUFF_SZ,
						   &get_info_addr,
						   GFP_KERNEL);
	if (!get_info_data) {
		ret = -ENOMEM;
		dev_dbg(priv->dev,
			"%s: Failed to allocate get_info_addr.\n",
			__func__);
		return ret;
	}

	tx_msg = kzalloc(ELE_GET_INFO_REQ_MSG_SZ, GFP_KERNEL);
	if (!tx_msg) {
		ret = -ENOMEM;
		goto exit;
	}

	rx_msg = kzalloc(ELE_GET_INFO_RSP_MSG_SZ, GFP_KERNEL);
	if (!rx_msg) {
		ret = -ENOMEM;
		goto exit;
	}

	ret = se_fill_cmd_msg_hdr(priv,
				      (struct se_msg_hdr *)&tx_msg->header,
				      ELE_GET_INFO_REQ,
				      ELE_GET_INFO_REQ_MSG_SZ,
				      true);
	if (ret)
		goto exit;

	tx_msg->data[0] = upper_32_bits(get_info_addr);
	tx_msg->data[1] = lower_32_bits(get_info_addr);
	tx_msg->data[2] = sizeof(*s_info);
	ret = ele_msg_send_rcv(priv->priv_dev_ctx,
			       tx_msg,
			       ELE_GET_INFO_REQ_MSG_SZ,
			       rx_msg,
			       ELE_GET_INFO_RSP_MSG_SZ);
	if (ret < 0)
		goto exit;

	ret = se_val_rsp_hdr_n_status(priv,
				      rx_msg,
				      ELE_GET_INFO_REQ,
				      ELE_GET_INFO_RSP_MSG_SZ,
				      true);

	memcpy(s_info, get_info_data, sizeof(*s_info));

exit:
	if (priv->mem_pool)
		gen_pool_free(priv->mem_pool,
			      (u64) get_info_data,
			      ELE_GET_INFO_BUFF_SZ);
	else
		dma_free_coherent(priv->dev,
				  ELE_GET_INFO_BUFF_SZ,
				  get_info_data,
				  get_info_addr);

	return ret;
}

int ele_fetch_soc_info(struct se_if_priv *priv, void *data)
{
	int err;

	err = ele_get_info(priv, data);
	if (err < 0)
		return err;

	return err;
}

int ele_ping(struct se_if_priv *priv)
{
	struct se_api_msg *tx_msg __free(kfree) = NULL;
	struct se_api_msg *rx_msg __free(kfree) = NULL;
	int ret = 0;

	if (!priv) {
		ret = -EINVAL;
		goto exit;
	}

	tx_msg = kzalloc(ELE_PING_REQ_SZ, GFP_KERNEL);
	if (!tx_msg) {
		ret = -ENOMEM;
		goto exit;
	}

	rx_msg = kzalloc(ELE_PING_RSP_SZ, GFP_KERNEL);
	if (!rx_msg) {
		ret = -ENOMEM;
		goto exit;
	}

	ret = se_fill_cmd_msg_hdr(priv,
				      (struct se_msg_hdr *)&tx_msg->header,
				      ELE_PING_REQ, ELE_PING_REQ_SZ, true);
	if (ret) {
		dev_err(priv->dev, "Error: se_fill_cmd_msg_hdr failed.\n");
		goto exit;
	}

	ret = ele_msg_send_rcv(priv->priv_dev_ctx,
			       tx_msg,
			       ELE_PING_REQ_SZ,
			       rx_msg,
			       ELE_PING_RSP_SZ);
	if (ret < 0)
		goto exit;

	ret = se_val_rsp_hdr_n_status(priv,
				      rx_msg,
				      ELE_PING_REQ,
				      ELE_PING_RSP_SZ,
				      true);
exit:
	return ret;
}

int ele_service_swap(struct se_if_priv *priv,
		     phys_addr_t addr,
		     u32 addr_size, u16 flag)
{
	struct se_api_msg *tx_msg __free(kfree) = NULL;
	struct se_api_msg *rx_msg __free(kfree) = NULL;
	int ret = 0;

	if (!priv) {
		ret = -EINVAL;
		goto exit;
	}

	tx_msg = kzalloc(ELE_SERVICE_SWAP_REQ_MSG_SZ, GFP_KERNEL);
	if (!tx_msg) {
		ret = -ENOMEM;
		goto exit;
	}

	rx_msg = kzalloc(ELE_SERVICE_SWAP_RSP_MSG_SZ, GFP_KERNEL);
	if (!rx_msg) {
		ret = -ENOMEM;
		goto exit;
	}

	ret = se_fill_cmd_msg_hdr(priv,
				      (struct se_msg_hdr *)&tx_msg->header,
				      ELE_SERVICE_SWAP_REQ,
				      ELE_SERVICE_SWAP_REQ_MSG_SZ, true);
	if (ret)
		goto exit;

	tx_msg->data[0] = flag;
	tx_msg->data[1] = addr_size;
	tx_msg->data[2] = ELE_NONE_VAL;
	tx_msg->data[3] = lower_32_bits(addr);
	tx_msg->data[4] = se_add_msg_crc((uint32_t *)&tx_msg[0],
						 ELE_SERVICE_SWAP_REQ_MSG_SZ);
	ret = ele_msg_send_rcv(priv->priv_dev_ctx,
			       tx_msg,
			       ELE_SERVICE_SWAP_REQ_MSG_SZ,
			       rx_msg,
			       ELE_SERVICE_SWAP_RSP_MSG_SZ);
	if (ret < 0)
		goto exit;

	ret = se_val_rsp_hdr_n_status(priv,
				      rx_msg,
				      ELE_SERVICE_SWAP_REQ,
				      ELE_SERVICE_SWAP_RSP_MSG_SZ,
				      true);
	if (ret)
		goto exit;

	if (flag == ELE_IMEM_EXPORT)
		ret = rx_msg->data[1];
	else
		ret = 0;

exit:

	return ret;
}

int ele_fw_authenticate(struct se_if_priv *priv, phys_addr_t addr)
{
	struct se_api_msg *tx_msg __free(kfree) = NULL;
	struct se_api_msg *rx_msg __free(kfree) = NULL;
	int ret = 0;

	if (!priv) {
		ret = -EINVAL;
		goto exit;
	}

	tx_msg = kzalloc(ELE_FW_AUTH_REQ_SZ, GFP_KERNEL);
	if (!tx_msg) {
		ret = -ENOMEM;
		goto exit;
	}

	rx_msg = kzalloc(ELE_FW_AUTH_RSP_MSG_SZ, GFP_KERNEL);
	if (!rx_msg) {
		ret = -ENOMEM;
		goto exit;
	}

	ret = se_fill_cmd_msg_hdr(priv,
				  (struct se_msg_hdr *)&tx_msg->header,
				  ELE_FW_AUTH_REQ,
				  ELE_FW_AUTH_REQ_SZ,
				  true);
	if (ret)
		goto exit;

	tx_msg->data[1] = upper_32_bits(addr);
	tx_msg->data[0] = lower_32_bits(addr);
	tx_msg->data[2] = addr;

	ret = ele_msg_send_rcv(priv->priv_dev_ctx,
			       tx_msg,
			       ELE_FW_AUTH_REQ_SZ,
			       rx_msg,
			       ELE_FW_AUTH_RSP_MSG_SZ);
	if (ret < 0)
		goto exit;

	ret = se_val_rsp_hdr_n_status(priv,
				      rx_msg,
				      ELE_FW_AUTH_REQ,
				      ELE_FW_AUTH_RSP_MSG_SZ,
				      true);
exit:
	return ret;
}

/*
 * ele_start_rng() - prepare and send the command to start
 *                   initialization of the ELE RNG context
 *
 * returns:  0 on success.
 */
int ele_start_rng(struct se_if_priv *priv)
{
	struct se_api_msg *tx_msg __free(kfree) = NULL;
	struct se_api_msg *rx_msg __free(kfree) = NULL;
	int ret = 0;

	if (!priv) {
		ret = -EINVAL;
		goto exit;
	}

	tx_msg = kzalloc(ELE_START_RNG_REQ_MSG_SZ, GFP_KERNEL);
	if (!tx_msg) {
		ret = -ENOMEM;
		goto exit;
	}

	rx_msg = kzalloc(ELE_START_RNG_RSP_MSG_SZ, GFP_KERNEL);
	if (!rx_msg) {
		ret = -ENOMEM;
		goto exit;
	}

	ret = se_fill_cmd_msg_hdr(priv,
				  (struct se_msg_hdr *)&tx_msg->header,
				  ELE_START_RNG_REQ,
				  ELE_START_RNG_REQ_MSG_SZ,
				  true);
	if (ret)
		goto exit;

	ret = ele_msg_send_rcv(priv->priv_dev_ctx,
			       tx_msg,
			       ELE_START_RNG_REQ_MSG_SZ,
			       rx_msg,
			       ELE_START_RNG_RSP_MSG_SZ);
	if (ret < 0)
		goto exit;

	ret = se_val_rsp_hdr_n_status(priv,
				      rx_msg,
				      ELE_START_RNG_REQ,
				      ELE_START_RNG_RSP_MSG_SZ,
				      true);
exit:
	return ret;
}

int ele_write_fuse(struct se_if_priv *priv, uint16_t fuse_index,
		   u32 value, bool block)
{
	struct se_api_msg *tx_msg __free(kfree) = NULL;
	struct se_api_msg *rx_msg __free(kfree) = NULL;
	int ret = 0;

	if (!priv) {
		ret = -EINVAL;
		goto exit;
	}

	tx_msg = kzalloc(ELE_WRITE_FUSE_REQ_MSG_SZ, GFP_KERNEL);
	if (!tx_msg) {
		ret = -ENOMEM;
		goto exit;
	}

	rx_msg = kzalloc(ELE_WRITE_FUSE_RSP_MSG_SZ, GFP_KERNEL);
	if (!rx_msg) {
		ret = -ENOMEM;
		goto exit;
	}

	ret = se_fill_cmd_msg_hdr(priv,
				  (struct se_msg_hdr *)&tx_msg->header,
				  ELE_WRITE_FUSE,
				  ELE_WRITE_FUSE_REQ_MSG_SZ,
				  true);
	if (ret)
		goto exit;

	tx_msg->data[0] = (32 << 16) | (fuse_index << 5);
	if (block)
		tx_msg->data[0] |= BIT(31);

	tx_msg->data[1] = value;

	ret = ele_msg_send_rcv(priv->priv_dev_ctx,
			       tx_msg,
			       ELE_WRITE_FUSE_REQ_MSG_SZ,
			       rx_msg,
			       ELE_WRITE_FUSE_RSP_MSG_SZ);
	if (ret < 0)
		goto exit;

	ret = se_val_rsp_hdr_n_status(priv,
				      rx_msg,
				      ELE_WRITE_FUSE,
				      ELE_WRITE_FUSE_RSP_MSG_SZ,
				      true);
exit:
	return ret;
}

/**
 * imx_se_write_fuse() - API to request SE-FW to write to fuses.
 * @void *se_if_data: refs to data attached to the se interface.
 * @uint16_t fuse_index: Fuse identifier to write to.
 * @u32 value: unsigned integer value that to be written to the fuse.
 * @bool block: Flag to check if it is a block.
 *
 * Secure-enclave like EdgeLock Enclave, manages the fuse. This API
 * requests the FW to read the common fuses. FW responds with the read
 * values.
 *
 * Context:
 *
 * Return value:
 *   0,   means success.
 *   < 0, means failure.
 */
int imx_se_write_fuse(void *se_if_data, uint16_t fuse_index,
		   u32 value, bool block)
{
	return ele_write_fuse((struct se_if_priv *)se_if_data, fuse_index,
				value, block);
}
EXPORT_SYMBOL_GPL(imx_se_write_fuse);

int read_common_fuse(struct se_if_priv *priv,
		     uint16_t fuse_id, u32 *value)
{
	struct se_api_msg *tx_msg __free(kfree) = NULL;
	struct se_api_msg *rx_msg __free(kfree) = NULL;
	int rx_msg_sz = ELE_READ_FUSE_RSP_MSG_SZ;
	int ret = 0;

	if (!priv) {
		ret = -EINVAL;
		goto exit;
	}

	tx_msg = kzalloc(ELE_READ_FUSE_REQ_MSG_SZ, GFP_KERNEL);
	if (!tx_msg) {
		ret = -ENOMEM;
		goto exit;
	}

	if (fuse_id == OTP_UNIQ_ID)
		rx_msg_sz = ELE_READ_FUSE_OTP_UNQ_ID_RSP_MSG_SZ;

	rx_msg = kzalloc(rx_msg_sz, GFP_KERNEL);
	if (!rx_msg) {
		ret = -ENOMEM;
		goto exit;
	}

	ret = se_fill_cmd_msg_hdr(priv, (struct se_msg_hdr *)&tx_msg->header,
				  ELE_READ_FUSE_REQ, ELE_READ_FUSE_REQ_MSG_SZ,
				  true);
	if (ret) {
		dev_err(priv->dev, "Error: se_fill_cmd_msg_hdr failed.\n");
		goto exit;
	}

	tx_msg->data[0] = fuse_id;

	ret = ele_msg_send_rcv(priv->priv_dev_ctx,
			       tx_msg,
			       ELE_READ_FUSE_REQ_MSG_SZ,
			       rx_msg,
			       rx_msg_sz);
	if (ret < 0)
		goto exit;

	ret = se_val_rsp_hdr_n_status(priv,
				      rx_msg,
				      ELE_READ_FUSE_REQ,
				      rx_msg_sz,
				      true);
	if (ret)
		goto exit;

	switch (fuse_id) {
	case OTP_UNIQ_ID:
		value[0] = rx_msg->data[1];
		value[1] = rx_msg->data[2];
		value[2] = rx_msg->data[3];
		value[3] = rx_msg->data[4];
		break;
	default:
		value[0] = rx_msg->data[1];
		break;
	}

exit:
	return ret;
}

/**
 * imx_se_read_fuse() - API to request SE-FW to read the fuse(s) value.
 * @void *se_if_data: refs to data attached to the se interface.
 * @uint16_t fuse_id: Fuse identifier to read.
 * @u32 *value: unsigned integer array to store the fused-values.
 *
 * Secure-enclave like EdgeLock Enclave, manages the fuse. This API
 * requests the FW to read the common fuses. FW responds with the read
 * values.
 *
 * Context:
 *
 * Return value:
 *   0,   means success.
 *   < 0, means failure.
 */
int imx_se_read_fuse(void *se_if_data,
		     uint16_t fuse_id, u32 *value)
{
	return read_common_fuse((struct se_if_priv *)se_if_data, fuse_id, value);
}
EXPORT_SYMBOL_GPL(imx_se_read_fuse);

int ele_voltage_change_req(struct se_if_priv *priv, bool start)
{
	struct se_api_msg *tx_msg __free(kfree) = NULL;
	struct se_api_msg *rx_msg __free(kfree) = NULL;
	uint8_t cmd = start ? ELE_VOLT_CHANGE_START_REQ : ELE_VOLT_CHANGE_FINISH_REQ;
	int ret = 0;

	if (!priv) {
		ret = -EINVAL;
		goto exit;
	}

	tx_msg = kzalloc(ELE_VOLT_CHANGE_REQ_MSG_SZ, GFP_KERNEL);
	if (!tx_msg) {
		ret = -ENOMEM;
		goto exit;
	}

	rx_msg = kzalloc(ELE_VOLT_CHANGE_RSP_MSG_SZ, GFP_KERNEL);
	if (!rx_msg) {
		ret = -ENOMEM;
		goto exit;
	}

	ret = se_fill_cmd_msg_hdr(priv,
				  (struct se_msg_hdr *)&tx_msg->header,
				  cmd,
				  ELE_VOLT_CHANGE_REQ_MSG_SZ,
				  true);
	if (ret)
		goto exit;

	se_continue_to_enforce_msg_seq_flow(&priv->se_msg_sq_ctl,
					    tx_msg);

	ret = ele_msg_send_rcv(priv->priv_dev_ctx,
			       tx_msg,
			       ELE_VOLT_CHANGE_REQ_MSG_SZ,
			       rx_msg,
			       ELE_VOLT_CHANGE_RSP_MSG_SZ);
	if (ret < 0)
		goto exit;

	ret = se_val_rsp_hdr_n_status(priv,
				      rx_msg,
				      cmd,
				      ELE_VOLT_CHANGE_RSP_MSG_SZ,
				      true);
exit:
	return ret;
}

/**
 * imx_se_voltage_change_req() - API to request change in voltage.
 * @void *se_if_data: refs to data attached to the se interface.
 * @bool start: if true, trigger the change in voltage.
 *              if false, finish the change in voltage.
 *
 * Secure-enclave like EdgeLock Enclave, manages the fuse. This API
 * requests the FW to read the common fuses. FW responds with the read
 * values.
 *
 * Context:
 *
 * Return value:
 *   0,   means success.
 *   < 0, means failure.
 */
int imx_se_voltage_change_req(void *se_if_data, bool start)
{
	struct se_if_priv *priv = se_if_data;
	int ret;

	if (start)
		se_start_enforce_msg_seq_flow(&priv->se_msg_sq_ctl);

	ret = ele_voltage_change_req(priv, start);

	if (start == false)
		se_halt_to_enforce_msg_seq_flow(&priv->se_msg_sq_ctl);

	return ret;
}
EXPORT_SYMBOL_GPL(imx_se_voltage_change_req);

int ele_get_v2x_fw_state(struct se_if_priv *priv, uint32_t *state)
{
	struct se_api_msg *tx_msg __free(kfree) = NULL;
	struct se_api_msg *rx_msg __free(kfree) = NULL;
	int ret = 0;

	if (!priv) {
		ret = -EINVAL;
		goto exit;
	}

	tx_msg = kzalloc(ELE_GET_STATE_REQ_SZ, GFP_KERNEL);
	if (!tx_msg) {
		ret = -ENOMEM;
		goto exit;
	}

	rx_msg = kzalloc(ELE_GET_STATE_RSP_SZ, GFP_KERNEL);
	if (!rx_msg) {
		ret = -ENOMEM;
		goto exit;
	}

	ret = se_fill_cmd_msg_hdr(priv,
				  (struct se_msg_hdr *)&tx_msg->header,
				  ELE_GET_STATE,
				  ELE_GET_STATE_REQ_SZ,
				  true);
	if (ret)
		goto exit;

	ret = ele_msg_send_rcv(priv->priv_dev_ctx,
			       tx_msg,
			       ELE_GET_STATE_REQ_SZ,
			       rx_msg,
			       ELE_GET_STATE_RSP_SZ);
	if (ret < 0)
		goto exit;

	ret = se_val_rsp_hdr_n_status(priv,
				      rx_msg,
				      ELE_GET_STATE,
				      ELE_GET_STATE_RSP_SZ,
				      true);
	if (!ret)
		*state = 0xFF & rx_msg->data[1];
exit:
	return ret;
}

int ele_v2x_fw_authenticate(struct se_if_priv *priv, phys_addr_t addr)
{
	struct se_api_msg *tx_msg __free(kfree) = NULL;
	struct se_api_msg *rx_msg __free(kfree) = NULL;
	int ret = 0;

	if (!priv) {
		ret = -EINVAL;
		goto exit;
	}

	tx_msg = kzalloc(ELE_V2X_FW_AUTH_REQ_SZ, GFP_KERNEL);
	if (!tx_msg) {
		ret = -ENOMEM;
		goto exit;
	}

	rx_msg = kzalloc(ELE_V2X_FW_AUTH_RSP_MSG_SZ, GFP_KERNEL);
	if (!rx_msg) {
		ret = -ENOMEM;
		goto exit;
	}

	ret = se_fill_cmd_msg_hdr(priv,
				  (struct se_msg_hdr *)&tx_msg->header,
				  ELE_V2X_FW_AUTH_REQ,
				  ELE_V2X_FW_AUTH_REQ_SZ,
				  true);
	if (ret)
		goto exit;

	tx_msg->data[1] = upper_32_bits(addr);
	tx_msg->data[0] = lower_32_bits(addr);
	tx_msg->data[2] = addr;

	ret = ele_msg_send_rcv(priv->priv_dev_ctx,
			       tx_msg,
			       ELE_V2X_FW_AUTH_REQ_SZ,
			       rx_msg,
			       ELE_V2X_FW_AUTH_RSP_MSG_SZ);
	if (ret < 0)
		goto exit;

	ret = se_val_rsp_hdr_n_status(priv,
				      rx_msg,
				      ELE_V2X_FW_AUTH_REQ,
				      ELE_V2X_FW_AUTH_RSP_MSG_SZ,
				      true);
exit:
	return ret;
}

int ele_debug_dump(struct se_if_priv *priv)
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

	tx_msg = kzalloc(ELE_DEBUG_DUMP_REQ_SZ, GFP_KERNEL);
	if (!tx_msg) {
		ret = -ENOMEM;
		goto exit;
	}

	rx_msg = kzalloc(ELE_DEBUG_DUMP_RSP_SZ, GFP_KERNEL);
	if (!rx_msg) {
		ret = -ENOMEM;
		goto exit;
	}

	ret = se_fill_cmd_msg_hdr(priv,
				  &tx_msg->header,
				  ELE_DEBUG_DUMP_REQ,
				  ELE_DEBUG_DUMP_REQ_SZ,
				  true);
	if (ret)
		goto exit;

	msg_ex_cnt = 0;
	do {
		w_ct = 0;
		fmt_str_idx = 0;
		memset(rx_msg, 0xCC, ELE_DEBUG_DUMP_RSP_SZ);

		ret = ele_msg_send_rcv(priv->priv_dev_ctx,
				       tx_msg,
				       ELE_DEBUG_DUMP_REQ_SZ,
				       rx_msg,
				       ELE_DEBUG_DUMP_RSP_SZ);
		if (ret < 0)
			goto exit;

		ret = se_val_rsp_hdr_n_status(priv,
					      rx_msg,
					      ELE_DEBUG_DUMP_REQ,
					      ELE_DEBUG_DUMP_RSP_SZ,
					      true);
		if (!ret) {
			rcv_dbg_wd_ct = rx_msg->header.size - ELE_NON_DUMP_BUFFER_SZ;
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
			keep_logging = (rx_msg->header.size < (ELE_DEBUG_DUMP_RSP_SZ >> 2)) ?
					false : true;
			keep_logging = keep_logging ?
						(msg_ex_cnt > ELE_MAX_DBG_DMP_PKT ? false : true) :
						false;
			/*
			 * Number of spaces = rcv_dbg_wd_ct
			 * DBG dump length in bytes = rcv_dbg_wd_ct * 4
			 *
			 * Since, one byte is represented as 2 character,
			 * DBG Dump string-length = rcv_dbg_wd_ct * 8
			 * Fixed string's string-length =
			 *                      strlen(FW_DBG_DUMP_FIXED_STR) * rcv_dbg_wd_ct
			 *
			 * Total dump_data length = Number of spaces +
			 *                          DBG Dump string' string-length +
			 *                          Fixed string's string-length
			 *
			 * Total dump_data length = rcv_dbg_wd_ct + (rcv_dbg_wd_ct * 8) +
			 *                          strlen(FW_DBG_DUMP_FIXED_STR) * rcv_dbg_wd_ct
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
