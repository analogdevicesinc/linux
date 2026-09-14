// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2025 NXP
 */

#include <linux/types.h>

#include <linux/cleanup.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/genalloc.h>

#include "ele_base_msg.h"
#include "ele_common.h"

#define FW_DBG_DUMP_FIXED_STR		"ELE"

static void ele_get_info_cleanup(struct se_if_priv *priv, u32 *buf, dma_addr_t d_addr,
				 size_t size)
{
	if (priv->mem_pool)
		gen_pool_free(priv->mem_pool, (unsigned long)buf, size);
	else
		dma_free_coherent(priv->dev, size, buf, d_addr);
}

/**
 * ele_get_info() - retrieve SoC and firmware information from the ELE.
 * @priv: pointer to the SE interface private data.
 * @s_info: output buffer; filled with device info on success.
 *
 * Allocates a DMA-coherent bounce buffer (from the gen_pool if available,
 * otherwise from the DMA API), sends an ELE_GET_INFO_REQ command, and copies
 * the result into @s_info.
 *
 * Return: 0 on success, negative errno on failure.
 */
int ele_get_info(struct se_if_priv *priv, struct ele_dev_info *s_info)
{
	dma_addr_t get_info_addr = 0;
	void *get_info_data = NULL;
	u32 get_info_len;
	int ret;

	if (!priv)
		return -EINVAL;

	memset(s_info, 0x0, sizeof(*s_info));

	struct se_api_msg *tx_msg __free(kfree) =
		kzalloc(ELE_GET_INFO_REQ_MSG_SZ, GFP_KERNEL);
	if (!tx_msg)
		return -ENOMEM;

	struct se_api_msg *rx_msg __free(kfree) =
		kzalloc(ELE_GET_INFO_RSP_MSG_SZ, GFP_KERNEL);
	if (!rx_msg)
		return -ENOMEM;

	get_info_len = ELE_GET_INFO_BUFF_SZ;
	if (priv->mem_pool)
		get_info_data = gen_pool_dma_alloc(priv->mem_pool,
						   get_info_len,
						   &get_info_addr);
	else
		get_info_data = dma_alloc_coherent(priv->dev,
						   get_info_len,
						   &get_info_addr,
						   GFP_KERNEL);
	if (!get_info_data) {
		dev_err(priv->dev,
			"%s: Failed to allocate get_info_addr.\n", __func__);
		return -ENOMEM;
	}

	/* gen_pool_dma_alloc() does not zero the buffer. */
	memset(get_info_data, 0, get_info_len);

	se_fill_cmd_msg_hdr(priv, (struct se_msg_hdr *)&tx_msg->header,
			    ELE_GET_INFO_REQ, ELE_GET_INFO_REQ_MSG_SZ, true);

	tx_msg->data[0] = upper_32_bits(get_info_addr);
	tx_msg->data[1] = lower_32_bits(get_info_addr);
	tx_msg->data[2] = sizeof(*s_info);

	ret = ele_msg_send_rcv(priv->priv_dev_ctx, tx_msg, ELE_GET_INFO_REQ_MSG_SZ,
			       rx_msg, ELE_GET_INFO_RSP_MSG_SZ);
	if (ret < 0) {
		ele_get_info_cleanup(priv, get_info_data, get_info_addr, get_info_len);
		return ret;
	}

	ret = se_val_rsp_hdr_n_status(priv->priv_dev_ctx, rx_msg, ELE_GET_INFO_REQ,
				      ELE_GET_INFO_RSP_MSG_SZ,
				      priv->if_defs->base_api_ver);
	if (ret < 0) {
		ele_get_info_cleanup(priv, get_info_data, get_info_addr, get_info_len);
		return ret;
	}

	memcpy(s_info, get_info_data, sizeof(*s_info));

	ele_get_info_cleanup(priv, get_info_data, get_info_addr, get_info_len);

	return ret;
}

/**
 * ele_fetch_soc_info() - wrapper around ele_get_info() for generic callers.
 * @priv: pointer to the SE interface private data.
 * @data: output buffer of at least sizeof(struct ele_dev_info) bytes.
 *
 * Return: 0 on success, negative errno on failure.
 */
int ele_fetch_soc_info(struct se_if_priv *priv, void *data)
{
	return ele_get_info(priv, (struct ele_dev_info *)data);
}

/**
 * ele_ping() - send a ping command to the secure enclave.
 * @priv: pointer to the SE interface private data.
 *
 * Verifies that the secure enclave is alive and responsive.
 *
 * Return: 0 on success, negative errno on failure.
 */
int ele_ping(struct se_if_priv *priv)
{
	int ret;

	if (!priv)
		return -EINVAL;

	struct se_api_msg *tx_msg __free(kfree) = kzalloc(ELE_PING_REQ_SZ,
							  GFP_KERNEL);
	if (!tx_msg)
		return -ENOMEM;

	struct se_api_msg *rx_msg __free(kfree) = kzalloc(ELE_PING_RSP_SZ,
							  GFP_KERNEL);
	if (!rx_msg)
		return -ENOMEM;

	se_fill_cmd_msg_hdr(priv, (struct se_msg_hdr *)&tx_msg->header,
			    ELE_PING_REQ, ELE_PING_REQ_SZ, true);

	ret = ele_msg_send_rcv(priv->priv_dev_ctx, tx_msg, ELE_PING_REQ_SZ,
			       rx_msg, ELE_PING_RSP_SZ);
	if (ret < 0)
		return ret;

	ret = se_val_rsp_hdr_n_status(priv->priv_dev_ctx, rx_msg, ELE_PING_REQ,
				      ELE_PING_RSP_SZ,
				      priv->if_defs->base_api_ver);

	return ret;
}

/**
 * ele_service_swap() - issue an ELE service-swap (IMEM export/import) command.
 * @priv: pointer to the SE interface private data.
 * @addr: DMA address of the IMEM buffer; must fit in 32 bits.
 * @addr_size: size of the buffer at @addr in bytes.
 * @flag: ELE_IMEM_EXPORT or ELE_IMEM_IMPORT.
 *
 * Return: exported size in bytes (ELE_IMEM_EXPORT), 0 (ELE_IMEM_IMPORT),
 * or negative errno on failure.
 */
int ele_service_swap(struct se_if_priv *priv,
		     dma_addr_t addr,
		     u32 addr_size, u16 flag)
{
	int ret;

	if (!priv)
		return -EINVAL;

	if (upper_32_bits(addr)) {
		dev_err(priv->dev,
			"ELE service-swap address exceeds 32-bit range: %pad\n",
			&addr);
		return -ERANGE;
	}

	struct se_api_msg *tx_msg __free(kfree)	=
		kzalloc(ELE_SERVICE_SWAP_REQ_MSG_SZ, GFP_KERNEL);
	if (!tx_msg)
		return -ENOMEM;

	struct se_api_msg *rx_msg __free(kfree) =
		kzalloc(ELE_SERVICE_SWAP_RSP_MSG_SZ, GFP_KERNEL);
	if (!rx_msg)
		return -ENOMEM;

	se_fill_cmd_msg_hdr(priv, (struct se_msg_hdr *)&tx_msg->header,
			    ELE_SERVICE_SWAP_REQ, ELE_SERVICE_SWAP_REQ_MSG_SZ, true);

	tx_msg->data[0] = flag;
	tx_msg->data[1] = addr_size;
	tx_msg->data[2] = ELE_NONE_VAL;
	tx_msg->data[3] = lower_32_bits(addr);
	ret = se_update_msg_chksum((u32 *)&tx_msg[0], ELE_SERVICE_SWAP_REQ_MSG_SZ);
	if (ret)
		return -EINVAL;

	ret = ele_msg_send_rcv(priv->priv_dev_ctx, tx_msg, ELE_SERVICE_SWAP_REQ_MSG_SZ,
			       rx_msg, ELE_SERVICE_SWAP_RSP_MSG_SZ);
	if (ret < 0)
		return ret;

	ret = se_val_rsp_hdr_n_status(priv->priv_dev_ctx, rx_msg, ELE_SERVICE_SWAP_REQ,
				      ELE_SERVICE_SWAP_RSP_MSG_SZ,
				      priv->if_defs->base_api_ver);
	if (ret)
		return ret;

	if (flag == ELE_IMEM_EXPORT)
		ret = rx_msg->data[1];
	else
		ret = 0;

	return ret;
}

/**
 * ele_fw_authenticate() - authenticate a firmware container via the ELE.
 * @priv: pointer to the SE interface private data.
 * @contnr_addr: DMA address of the firmware container; must fit in 32 bits.
 * @img_addr: DMA address of the firmware image; must fit in 32 bits.
 *
 * Return: 0 on success, negative errno on failure.
 */
int ele_fw_authenticate(struct se_if_priv *priv, dma_addr_t contnr_addr,
			dma_addr_t img_addr)
{
	int ret;

	if (!priv)
		return -EINVAL;

	if (upper_32_bits(contnr_addr) || upper_32_bits(img_addr)) {
		dev_err(priv->dev, "Wrong address: %pad %pad\n", &contnr_addr, &img_addr);
		return -EINVAL;
	}

	struct se_api_msg *tx_msg __free(kfree)	=
		kzalloc(ELE_FW_AUTH_REQ_SZ, GFP_KERNEL);
	if (!tx_msg)
		return -ENOMEM;

	struct se_api_msg *rx_msg __free(kfree) =
		kzalloc(ELE_FW_AUTH_RSP_MSG_SZ, GFP_KERNEL);
	if (!rx_msg)
		return -ENOMEM;

	se_fill_cmd_msg_hdr(priv, (struct se_msg_hdr *)&tx_msg->header,
			    ELE_FW_AUTH_REQ, ELE_FW_AUTH_REQ_SZ, true);

	tx_msg->data[0] = lower_32_bits(contnr_addr);
	tx_msg->data[1] = 0;
	tx_msg->data[2] = lower_32_bits(img_addr);

	ret = ele_msg_send_rcv(priv->priv_dev_ctx, tx_msg, ELE_FW_AUTH_REQ_SZ, rx_msg,
			       ELE_FW_AUTH_RSP_MSG_SZ);
	if (ret < 0)
		return ret;

	ret = se_val_rsp_hdr_n_status(priv->priv_dev_ctx, rx_msg, ELE_FW_AUTH_REQ,
				      ELE_FW_AUTH_RSP_MSG_SZ,
				      priv->if_defs->base_api_ver);

	return ret;
}

/**
 * ele_debug_dump() - retrieve and log the ELE debug dump buffer.
 * @priv: pointer to the SE interface private data.
 *
 * Repeatedly issues ELE_DEBUG_DUMP_REQ commands and logs the responses via
 * dev_info() until no more data is available or the maximum packet count is
 * reached.
 *
 * Return: 0 on success, negative errno on failure.
 */
int ele_debug_dump(struct se_if_priv *priv)
{
	bool keep_logging;
	int msg_ex_cnt;
	int ret;
	int i;

	if (!priv)
		return -EINVAL;

	struct se_api_msg *tx_msg __free(kfree) = kzalloc(ELE_DEBUG_DUMP_REQ_SZ,
							  GFP_KERNEL);
	if (!tx_msg)
		return -ENOMEM;

	struct se_api_msg *rx_msg __free(kfree)	= kzalloc(ELE_DEBUG_DUMP_RSP_SZ,
							  GFP_KERNEL);
	if (!rx_msg)
		return -ENOMEM;

	se_fill_cmd_msg_hdr(priv, &tx_msg->header, ELE_DEBUG_DUMP_REQ,
			    ELE_DEBUG_DUMP_REQ_SZ, true);

	msg_ex_cnt = 0;
	do {
		memset(rx_msg, 0x0, ELE_DEBUG_DUMP_RSP_SZ);

		ret = ele_msg_send_rcv(priv->priv_dev_ctx, tx_msg, ELE_DEBUG_DUMP_REQ_SZ,
				       rx_msg, ELE_DEBUG_DUMP_RSP_SZ);
		if (ret < 0)
			return ret;

		ret = se_val_rsp_hdr_n_status(priv->priv_dev_ctx, rx_msg, ELE_DEBUG_DUMP_REQ,
					      ELE_DEBUG_DUMP_RSP_SZ,
					      priv->if_defs->base_api_ver);
		if (ret) {
			dev_err(priv->dev, "Dump_Debug_Buffer Error: %x.\n", ret);
			break;
		}
		keep_logging = (rx_msg->header.size >= (ELE_DEBUG_DUMP_RSP_SZ >> 2) &&
				msg_ex_cnt < ELE_MAX_DBG_DMP_PKT);

		rx_msg->header.size -= 2;

		if (rx_msg->header.size > 2)
			rx_msg->header.size--;

		for (i = 0; i < rx_msg->header.size; i += 2)
			dev_info(priv->dev, "%s%02x_%02x: 0x%08x 0x%08x\n",
				 FW_DBG_DUMP_FIXED_STR, msg_ex_cnt, i,
				 rx_msg->data[i + 1], rx_msg->data[i + 2]);

		msg_ex_cnt++;
	} while (keep_logging);

	return ret;
}
