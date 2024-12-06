/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2024 NXP
 */


#ifndef __ELE_COMMON_H__
#define __ELE_COMMON_H__

#include "se_ctrl.h"

#define ELE_SUCCESS_IND			0xD6
#define ELE_ABORT_ERR_CODE		0xFF29

#define IMX_ELE_FW_DIR                 "imx/ele/"

uint32_t se_add_msg_crc(uint32_t *msg, uint32_t msg_len);
int ele_msg_rcv(struct se_if_device_ctx *dev_ctx,
		struct se_clbk_handle *se_clbk_hdl);
int ele_msg_send(struct se_if_device_ctx *dev_ctx,
		 void *tx_msg,
		 int tx_msg_sz);
int ele_msg_send_rcv(struct se_if_device_ctx *dev_ctx,
		     void *tx_msg,
		     int tx_msg_sz,
		     void *rx_msg,
		     int exp_rx_msg_sz);
void se_if_rx_callback(struct mbox_client *mbox_cl, void *msg);
int se_val_rsp_hdr_n_status(struct se_if_priv *priv,
			    struct se_api_msg *msg,
			    uint8_t msg_id,
			    uint8_t sz,
			    bool is_base_api);

/* Fill a command message header with a given command ID and length in bytes. */
static inline int se_fill_cmd_msg_hdr(struct se_if_priv *priv,
				      struct se_msg_hdr *hdr,
				      u8 cmd, u32 len,
				      bool is_base_api)
{
	hdr->tag = priv->if_defs->cmd_tag;
	hdr->ver = (is_base_api) ? priv->if_defs->base_api_ver : priv->if_defs->fw_api_ver;
	hdr->command = cmd;
	hdr->size = len >> 2;

	return 0;
}

int se_save_imem_state(struct se_if_priv *priv, struct se_imem_buf *imem);
int se_restore_imem_state(struct se_if_priv *priv, struct se_imem_buf *imem);

#endif /*__ELE_COMMON_H__ */
