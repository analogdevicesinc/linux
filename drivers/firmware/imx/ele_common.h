/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2025 NXP
 */

#ifndef __ELE_COMMON_H__
#define __ELE_COMMON_H__

#include "se_ctrl.h"

#define SE_RCV_MSG_DEFAULT_TIMEOUT_MS	3000
#define SE_RCV_MSG_LONG_TIMEOUT_MS	5000000

#define ELE_SUCCESS_IND			0xD6

#define IMX_ELE_FW_DIR                 "imx/ele/"

#define MAX_WORD_SIZE			0x20

void set_se_rcv_msg_timeout(struct se_if_device_ctx *dev_ctx, u32 val);
int se_update_msg_chksum(u32 *msg, u32 msg_len);

int ele_msg_rcv(struct se_if_device_ctx *dev_ctx, struct se_clbk_handle *se_clbk_hdl);

int ele_msg_send(struct se_if_device_ctx *dev_ctx, void *tx_msg, int tx_msg_sz);

int ele_msg_send_rcv(struct se_if_device_ctx *dev_ctx, void *tx_msg,
		     int tx_msg_sz, void *rx_msg, int exp_rx_msg_sz,
		     int *act_rx_msg_sz);

void se_if_rx_callback(struct mbox_client *mbox_cl, void *msg);

int se_val_rsp_hdr_n_status(struct se_if_device_ctx *dev_ctx, struct se_api_msg *msg,
			    u8 msg_id, u8 sz, u8 version);

/*
 * Header that prefixes the input buffer of commands whose descriptor uses
 * SE_CMD_ADDR_DEDUCE_SZ (e.g. ELE_OEM_AUTH_CONTAINER_REQ,
 * ELE_KEYSTORE_REPROV_ENABLE_REQ). The command payload carries only the start
 * address of that buffer, but its first four bytes follow this fixed layout:
 * the 16-bit length field gives the total byte count of the buffer, so
 * se_val_cmd_addrs() can bound-check the whole buffer, not just its start. The
 * length is little-endian on the wire; read it with le16_to_cpu().
 */
struct fw_tag_len_vers_info {
	u8 version;
	__le16 length;
	u8 tag;
} __packed;

/**
 * struct se_cmd_addr_field - One DMA address embedded in an ELE command
 *                            message payload.
 *
 * A number of ELE commands carry DMA physical addresses inside their message
 * payload. se_val_cmd_addrs() range-checks each such address against the
 * calling context's shared-memory window before the message reaches firmware.
 *
 * data[] index = message WORD index - 1, because the 4-byte se_msg_hdr is
 * message WORD 0 and se_api_msg.data[0] is message WORD 1.
 *
 * @lsb_idx: data[] index of the low 32 bits of the address.
 * @msb_idx: data[] index of the high 32 bits; valid only when @has_msb is
 *           set. Some base-API commands split the address into two words;
 *           FW-API commands do not.
 * @has_msb: true when the address occupies two words (@lsb_idx + @msb_idx).
 * @flag_idx: data[] index of the flag word that selects whether
 *            data[@lsb_idx] is a DMA address or an integer key identifier;
 *            %SE_CMD_ADDR_ALWAYS when the word is always a DMA address.
 * @flag_mask: the selecting flag bit, already shifted to its position inside
 *             the 32-bit little-endian flag word.
 * @is_addr_when_set: true when the word is a DMA address if the flag bit is
 *                    set; false when it is an address if the bit is clear
 *                    (inverse polarity, e.g. VERIFY_SIGN OPAQUE_KEY).
 * @size_idx: data[] index of the word carrying the length in bytes of the
 *            buffer at this address, or one of the size-source sentinels
 *            below. se_val_cmd_addrs() uses the resulting length to confirm
 *            the whole buffer [addr, addr + len) fits inside the
 *            shared-memory window, not just its start.
 *            %SE_CMD_ADDR_FIXED_SIZE when the length is a firmware-defined
 *            literal supplied in @buf_size.
 *            %SE_CMD_RCVR_ADDR_VAR_SIZE when the length is taken from
 *            se_if_priv.cmd_rcvr_var_size (command-receiver responses).
 *            %SE_CMD_ADDR_DEDUCE_SZ when no length word is carried in the
 *            payload, but the buffer starts with a struct fw_tag_len_vers_info
 *            header whose length field gives the total buffer byte count;
 *            se_val_cmd_addrs() reads that header to range-check the whole
 *            buffer.
 * @size_shift: right shift applied to the size word before masking, for a
 *              length packed into the high half of a word.
 * @size_mask: bitmask applied after @size_shift to extract the length from
 *             the message word (0xFFFFFFFF for a full 32-bit length, 0xFFFF
 *             for a u16, 0xFF for a u8). Used only when @size_idx names a
 *             payload word; zero for the sentinel values.
 * @buf_size: firmware-defined literal byte count. Used only when
 *            @size_idx == %SE_CMD_ADDR_FIXED_SIZE, where the whole buffer
 *            [addr, addr + @buf_size) must fit inside the shared-memory
 *            window. Left zero for every other @size_idx value.
 */
struct se_cmd_addr_field {
	u8 lsb_idx;
	u8 msb_idx;
	bool has_msb;
	u8 flag_idx;
	u32 flag_mask;
	bool is_addr_when_set;
	u8 size_idx;
	u8 size_shift;
	u32 size_mask;
	u32 buf_size;
};

#define SE_CMD_ADDR_ALWAYS		0xEFu
/* size is the literal in buf_size */
#define SE_CMD_ADDR_FIXED_SIZE		0xFDu
/* size from se_if_priv.cmd_rcvr_var_size */
#define SE_CMD_RCVR_ADDR_VAR_SIZE	0xFEu
/* size read from the buffer's fw_tag_len_vers_info header */
#define SE_CMD_ADDR_DEDUCE_SZ		0xFFu

int se_val_cmd_addrs(struct se_if_device_ctx *dev_ctx, struct se_api_msg *msg,
		     u32 tx_msg_sz, const struct se_cmd_addr_field *fields,
		     size_t count);

const struct se_cmd_addr_field *ele_fw_cmd_addr_fields(u8 cmd, size_t *count);
const struct se_cmd_addr_field *ele_fw_rsp_addr_fields(u8 cmd, size_t *count);
const struct se_cmd_addr_field *ele_base_cmd_addr_fields(u8 cmd, size_t *count);
/* Fill a command message header with a given command ID and length in bytes. */
static inline void se_fill_cmd_msg_hdr(struct se_if_priv *priv, struct se_msg_hdr *hdr,
				       u8 cmd, u32 len, bool is_base_api)
{
	hdr->tag = priv->if_defs->cmd_tag;
	hdr->ver = (is_base_api) ? priv->if_defs->base_api_ver : priv->if_defs->fw_api_ver;
	hdr->command = cmd;
	hdr->size = len >> 2;
}

int se_save_imem_state(struct se_if_priv *priv, struct se_imem_buf *imem);

int se_restore_imem_state(struct se_if_priv *priv, struct se_imem_buf *imem);

int se_chk_tx_rsp_msg_hdr(struct se_if_device_ctx *dev_ctx, struct se_msg_hdr *header,
			  u32 tx_msg_sz);
int se_chk_tx_cmd_msg_hdr(struct se_if_device_ctx *dev_ctx, struct se_msg_hdr *header,
			  u32 tx_msg_sz, u32 rx_msg_sz);

#endif /*__ELE_COMMON_H__ */
