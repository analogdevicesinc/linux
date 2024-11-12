// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright 2023-2024 NXP
 */

#include <linux/dma-mapping.h>
#include <linux/firmware/imx/se_api.h>

#include "ele_common.h"
#include "ele_fw_api.h"

struct ele_rng_msg_data {
	u16 rsv;
	u16 flags;
	u32 data[2];
};

int ele_init_fw(struct se_if_priv *priv)
{
	struct se_api_msg *tx_msg __free(kfree) = NULL;
	struct se_api_msg *rx_msg __free(kfree) = NULL;
	int ret = 0;

	if (!priv) {
		ret = -EINVAL;
		goto exit;
	}

	tx_msg = kzalloc(ELE_INIT_FW_REQ_SZ, GFP_KERNEL);
	if (!tx_msg) {
		ret = -ENOMEM;
		goto exit;
	}

	rx_msg = kzalloc(ELE_INIT_FW_RSP_SZ, GFP_KERNEL);
	if (!rx_msg) {
		ret = -ENOMEM;
		goto exit;
	}

	ret = se_fill_cmd_msg_hdr(priv,
				  (struct se_msg_hdr *)&tx_msg->header,
				  ELE_INIT_FW_REQ,
				  ELE_INIT_FW_REQ_SZ,
				  false);
	if (ret)
		goto exit;

	ret = ele_msg_send_rcv(priv->priv_dev_ctx,
			       tx_msg,
			       ELE_INIT_FW_REQ_SZ,
			       rx_msg,
			       ELE_INIT_FW_RSP_SZ);
	if (ret < 0)
		goto exit;

	ret = se_val_rsp_hdr_n_status(priv,
				      rx_msg,
				      ELE_INIT_FW_REQ,
				      ELE_INIT_FW_RSP_SZ,
				      false);
exit:
	return ret;
}

/*
 * ele_get_random() - prepare and send the command to proceed
 *                    with a random number generation operation
 *
 * returns:  size of the rondom number generated
 */
int ele_get_random(struct se_if_priv *priv,
		   void *data, size_t len)
{
	struct se_api_msg *tx_msg __free(kfree) = NULL;
	struct se_api_msg *rx_msg __free(kfree) = NULL;
	struct ele_rng_msg_data *rng_msg_data;
	dma_addr_t dst_dma;
	u8 *buf = NULL;
	int ret;

	if (!priv) {
		ret = -EINVAL;
		goto exit;
	}

	tx_msg = kzalloc(ELE_GET_RANDOM_REQ_SZ, GFP_KERNEL);
	if (!tx_msg) {
		ret = -ENOMEM;
		goto exit;
	}

	rx_msg = kzalloc(ELE_GET_RANDOM_RSP_SZ, GFP_KERNEL);
	if (!rx_msg) {
		ret = -ENOMEM;
		goto exit;
	}

	/* As per RBG3(RS) construction mentioned in NIST SP800-90C,
	 * CTR_DRBG generates 128(full entropy) bits after reseeding
	 * the CTR_DRBG with 256 bits of entropy. so splitting the
	 * user rng request in multiple of 128 bits & enforce reseed
	 * for every iteration.
	 */
	len = ELE_RNG_MAX_SIZE;
	buf = dma_alloc_coherent(priv->dev, len, &dst_dma, GFP_KERNEL);
	if (!buf) {
		dev_err(priv->dev, "Failed to map destination buffer memory.\n");
		ret = -ENOMEM;
		goto exit;
	}

	ret = se_fill_cmd_msg_hdr(priv,
				  (struct se_msg_hdr *)&tx_msg->header,
				  ELE_GET_RANDOM_REQ,
				  ELE_GET_RANDOM_REQ_SZ,
				  false);
	if (ret)
		goto exit;

	rng_msg_data = (struct ele_rng_msg_data *)tx_msg->data;
	/* bit 1(blocking reseed): wait for trng entropy,
	 * then reseed rng context.
	 */
	if (get_se_soc_id(priv) != SOC_ID_OF_IMX95)
		rng_msg_data->flags = BIT(1);

	rng_msg_data->data[0] = dst_dma;
	rng_msg_data->data[1] = len;

	ret = ele_msg_send_rcv(priv->priv_dev_ctx,
			       tx_msg,
			       ELE_GET_RANDOM_REQ_SZ,
			       rx_msg,
			       ELE_GET_RANDOM_RSP_SZ);
	if (ret < 0)
		goto exit;

	ret = se_val_rsp_hdr_n_status(priv,
				      rx_msg,
				      ELE_GET_RANDOM_REQ,
				      ELE_GET_RANDOM_RSP_SZ,
				      false);
	if (!ret) {
		memcpy(data, buf, len);
		ret = len;
	}
exit:
	if (buf)
		dma_free_coherent(priv->dev, len, buf, dst_dma);
	return ret;
}
