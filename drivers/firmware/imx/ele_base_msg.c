// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2024 NXP
 */

#include <linux/types.h>

#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/genalloc.h>

#include "ele_base_msg.h"
#include "ele_common.h"

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

int ele_fetch_soc_info(struct se_if_priv *priv, u16 *soc_rev, u64 *serial_num)
{
	struct ele_dev_info s_info = {0};
	int err;

	err = ele_get_info(priv, &s_info);
	if (err < 0)
		return err;

	if (soc_rev)
		*soc_rev = s_info.d_info.soc_rev;
	if (serial_num)
		*serial_num = GET_SERIAL_NUM_FROM_UID(s_info.d_info.uid, MAX_UID_SIZE >> 2);

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
