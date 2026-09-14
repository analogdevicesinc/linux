// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2026 NXP
 */

#include <linux/types.h>

#include "ele_common.h"
#include "ele_base_msg.h"
#include "ele_fw_api.h"

/*
 * Base-API commands that embed one or more DMA physical addresses in their
 * payload. Unlike the FW-API crypto commands, GET_INFO and DEV_ATTEST split
 * their response-buffer address across two words: the high half is written
 * first (lower word index) and the low half next, so has_msb is set and the
 * msb_idx precedes the lsb_idx. GEN_KEY_BLOB uses single-word LSB addresses.
 * See struct se_cmd_addr_field in ele_common.h for the field semantics.
 */
static const struct se_cmd_addr_field ele_get_info_addr_fields[] = {
	/*
	 * rsp_data_addr_hi @ data[0], rsp_data_addr_lo @ data[1];
	 * buf_sz is a u16 in the low half of data[2].
	 */
	{ .lsb_idx = 1, .msb_idx = 0, .has_msb = true, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 2, .size_mask = 0xFFFFu },
};

static const struct se_cmd_addr_field ele_dev_attest_addr_fields[] = {
	/*
	 * rsp_data_addr_hi @ data[0], rsp_data_addr_lo @ data[1];
	 * buf_sz is a u16 in the low half of data[2].
	 */
	{ .lsb_idx = 1, .msb_idx = 0, .has_msb = true, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 2, .size_mask = 0xFFFFu },
};

static const struct se_cmd_addr_field ele_oem_auth_cntr_addr_fields[] = {
	/*
	 * Container Header address: a 64-bit physical address split across two
	 * words. data[0] holds the 32-bit MSB and data[1] holds the 32-bit LSB
	 * (ELE API spec Table 27, word size = 0x3, so the command is header +
	 * MSB + LSB only). No length word is carried in the payload; the buffer
	 * length is read from the fw_tag_len_vers_info header at its start
	 * (SE_CMD_ADDR_DEDUCE_SZ) so the whole buffer is range-checked.
	 */
	{ .lsb_idx = 1, .msb_idx = 0, .has_msb = true, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = SE_CMD_ADDR_DEDUCE_SZ },			/* container_hdr_addr */
};

/*
 * GENERATE ELE KEY BLOB command (ELE_GEN_KEY_BLOB_REQ, 0xAF).
 * ELE API spec Table 73, word size = 0x8 (header + 7 data words):
 *   data[0] = key_identifier
 *   data[1] = Reserved
 *   data[2] = load_address  (32-bit; must be 64-bit aligned)
 *   data[3] = Reserved
 *   data[4] = store_address (32-bit; must be 64-bit aligned)
 *   data[5] = Reserved[31:16] | max_export_size[15:0]
 *   data[6] = CRC
 *
 * load_addr points to the input: a blob header (8 bytes, Table 77) followed
 * by the plaintext payload. No size word is present in the message for this
 * input buffer. The maximum input size is determined by the largest supported
 * payload type: OTFAD key configuration (0x28 bytes per Table 79) plus the
 * 8-byte header gives 0x30 bytes. That is the literal upper bound placed in
 * buf_size and selected with SE_CMD_ADDR_FIXED_SIZE, so se_val_cmd_addrs()
 * can verify [load_addr, load_addr+0x30) lies within the shared-memory
 * window.
 */
#define OP_GEN_ELE_KEY_BLOB_INPUT_MAX_SZ	0x30  /* blob hdr (8) + OTFAD payload (0x28) */
static const struct se_cmd_addr_field ele_gen_key_blob_addr_fields[] = {
	{ .lsb_idx = 2, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = SE_CMD_ADDR_FIXED_SIZE,
	  .buf_size = OP_GEN_ELE_KEY_BLOB_INPUT_MAX_SZ },	/* load_address */
	/* store_address @ data[4]; max_export_size is u16 in low half of data[5] */
	{ .lsb_idx = 4, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 5, .size_mask = 0xFFFFu },		/* store_address */
};

/*
 * Return the address-field descriptor table for a base-API command, or NULL
 * when the command embeds no DMA addresses. count is set to the number of
 * entries.
 */
const struct se_cmd_addr_field *ele_base_cmd_addr_fields(u8 cmd, size_t *count)
{
	switch (cmd) {
	case ELE_OEM_AUTH_CONTAINER_REQ:
		*count = ARRAY_SIZE(ele_oem_auth_cntr_addr_fields);
		return ele_oem_auth_cntr_addr_fields;
	case ELE_GEN_KEY_BLOB_REQ:
		*count = ARRAY_SIZE(ele_gen_key_blob_addr_fields);
		return ele_gen_key_blob_addr_fields;
	case ELE_GET_INFO_REQ:
		*count = ARRAY_SIZE(ele_get_info_addr_fields);
		return ele_get_info_addr_fields;
	case ELE_DEV_ATTEST_REQ:
		*count = ARRAY_SIZE(ele_dev_attest_addr_fields);
		return ele_dev_attest_addr_fields;
	default:
		*count = 0;
		return NULL;
	}
}

/*
 * FW-API crypto commands that embed one or more DMA physical addresses in
 * their payload. On the PSA_COMPLIANT ABI most addresses are written by the
 * userspace library as a single little-endian 32-bit LSB word (the high half
 * is always zero), so has_msb is left false for those entries. A few commands
 * (pub-key-export 0x32, keystore reprov-enable 0x3F) carry an explicit ext/MSB
 * word ahead of the LSB word, matching the base-API two-word address layout;
 * their entries set has_msb = true so the MSB word is validated too. See
 * struct se_cmd_addr_field in ele_common.h for the field semantics.
 */
static const struct se_cmd_addr_field ele_pub_key_export_addr_fields[] = {
	/*
	 * out_key_addr: the recovered public key output buffer. Its high half
	 * out_key_addr_ext is data[2] and its low half out_key_addr is data[3];
	 * the library always writes it via set_phy_addr_to_words(), so it is
	 * always a DMA address. Its length is out_key_size, the u16 in the low
	 * half of data[4]. key_identifier (data[1]) is an integer, not an
	 * address.
	 */
	{ .lsb_idx = 3, .msb_idx = 2, .has_msb = true, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 4, .size_mask = 0xFFFFu },		/* out_key_addr */
};

static const struct se_cmd_addr_field ele_keystore_reprov_en_addr_fields[] = {
	/*
	 * Signed message address: a 64-bit physical address split across two
	 * words. data[0] holds the 32-bit MSB and data[1] holds the 32-bit LSB
	 * (ELE API spec Table 202, word size = 0x3, so the command is header +
	 * MSB + LSB only). No length word is carried in the payload; the buffer
	 * length is read from the fw_tag_len_vers_info header at its start
	 * (SE_CMD_ADDR_DEDUCE_SZ) so the whole buffer is range-checked.
	 */
	{ .lsb_idx = 1, .msb_idx = 0, .has_msb = true, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = SE_CMD_ADDR_DEDUCE_SZ },			/* signed_msg_addr */
};

static const struct se_cmd_addr_field ele_keygen_addr_fields[] = {
	/*
	 * key @ data[1]: a plaintext private-key output buffer only when the
	 * KEY_GENERATION PLAINTEXT_KEY flag (bit 3 of the flags byte in the low
	 * 8 bits of data[8]) is set; otherwise it is an integer key identifier.
	 * Its length is priv_key_sz, the u16 in the high half of data[8].
	 */
	{ .lsb_idx = 1, .flag_idx = 8, .flag_mask = 0x00000008u, .is_addr_when_set = true,
	  .size_idx = 8, .size_shift = 16, .size_mask = 0xFFFFu },	/* priv_key_addr */
	/*
	 * pub_key_addr @ data[9]: always a DMA address. Its length is
	 * pub_key_sz, the u16 in the low half of data[2].
	 */
	{ .lsb_idx = 9, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 2, .size_mask = 0xFFFFu },		/* pub_key_addr */
};

static const struct se_cmd_addr_field ele_key_exchange_addr_fields[] = {
	/*
	 * PSA_COMPLIANT key-exchange payload. key_management_handle is data[0]
	 * and flags/reserved is data[1]; the four buffer addresses that follow
	 * are each written unconditionally via set_phy_addr_to_words() (single
	 * LSB word, high half always zero), so all are always DMA addresses.
	 * Each address is immediately followed by its full u32 byte length.
	 */
	{ .lsb_idx = 2, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 3, .size_mask = 0xFFFFFFFFu },		/* in_content_addr */
	{ .lsb_idx = 4, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 5, .size_mask = 0xFFFFFFFFu },		/* in_pub_buffer_addr */
	{ .lsb_idx = 6, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 7, .size_mask = 0xFFFFFFFFu },		/* user_fixed_info_addr */
	{ .lsb_idx = 8, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 9, .size_mask = 0xFFFFFFFFu },		/* output_addr */
};

static const struct se_cmd_addr_field ele_key_import_addr_fields[] = {
	/*
	 * PSA_COMPLIANT key-import payload. key_management_handle is data[0]
	 * and flags/reserved is data[1]; the single input buffer address that
	 * follows is written unconditionally via set_phy_addr_to_words()
	 * (single LSB word, high half always zero), so it is always a DMA
	 * address. Its length is the full u32 input_size in data[3].
	 */
	{ .lsb_idx = 2, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 3, .size_mask = 0xFFFFFFFFu },		/* input_address */
};

/* ELE_MAC_REQ: MAC one-go operation. */
static const struct se_cmd_addr_field ele_mac_addr_fields[] = {
	/* key: plaintext-key buffer only when the MAC PLAINTEXT_KEY flag is set */
	{ .lsb_idx = 1, .flag_idx = 5, .flag_mask = 0x00080000u, .is_addr_when_set = true,
	  .size_idx = 7, .size_mask = 0xFFFFu },		/* key_size */
	{ .lsb_idx = 2, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 4, .size_mask = 0xFFFFFFFFu },		/* payload_address */
	{ .lsb_idx = 3, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 5, .size_mask = 0xFFFFu },		/* mac_address */
	{ .lsb_idx = 8, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 9, .size_mask = 0xFFFFu },		/* context_address */
};

/* ELE_CIPHER_REQ: symmetric cipher one-go operation. */
static const struct se_cmd_addr_field ele_cipher_addr_fields[] = {
	/* key: plaintext-key buffer only when the CIPHER PLAINTEXT_KEY flag is set */
	{ .lsb_idx = 1, .flag_idx = 3, .flag_mask = 0x00080000u, .is_addr_when_set = true,
	  .size_idx = 9, .size_mask = 0xFFFFu },		/* key_size */
	{ .lsb_idx = 2, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 3, .size_mask = 0xFFFFu },		/* iv_address */
	{ .lsb_idx = 5, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 7, .size_mask = 0xFFFFFFFFu },		/* input_address */
	{ .lsb_idx = 6, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 8, .size_mask = 0xFFFFFFFFu },		/* output_address */
	{ .lsb_idx = 10, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 11, .size_mask = 0xFFFFu },		/* context_address */
};

/*
 * AEAD encrypt/decrypt legacy command (ELE_AUTH_ENC_REQ, 0x64).
 * ELE API spec Table 287, word size = 0xD (header + 12 data words):
 *   data[0]  = cipher_handle
 *   data[1]  = key_identifier
 *   data[2]  = IV LSB address
 *   data[3]  = Reserved[31:24] | Flags[23:16] | IV_size[15:0]
 *   data[4]  = algorithm
 *   data[5]  = AAD LSB address
 *   data[6]  = Reserved[31:16] | AAD_size[15:0]
 *   data[7]  = Input LSB address
 *   data[8]  = Output LSB address
 *   data[9]  = Input size (u32)
 *   data[10] = Output size (u32)
 *   data[11] = CRC
 * IV size is the 16-bit low half of data[3]; AAD size is the 16-bit low half
 * of data[6]; input and output sizes are full u32 words.
 */
static const struct se_cmd_addr_field ele_auth_enc_addr_fields[] = {
	/* iv_address    (size[15:0]  @ data[3]) */
	{ .lsb_idx = 2, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 3, .size_mask = 0xFFFFu },
	/* aad_address   (size[15:0]  @ data[6]) */
	{ .lsb_idx = 5, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 6, .size_mask = 0xFFFFu },
	/* input_address (size[31:0]  @ data[9]) */
	{ .lsb_idx = 7, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 9, .size_mask = 0xFFFFFFFFu },
	/* output_address (size[31:0] @ data[10]) */
	{ .lsb_idx = 8, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 10, .size_mask = 0xFFFFFFFFu },
};

/* ELE_AUTH_ENC_NEW_REQ: AEAD encrypt/decrypt with internally-generated IV output. */
#define ELE_AUTH_ENC_IV_OUT_SIZE      12  /* firmware always writes exactly 12 bytes */
static const struct se_cmd_addr_field ele_auth_enc_new_addr_fields[] = {
	/* iv_address_in length is packed in the high half of the iv-size word */
	{ .lsb_idx = 3, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 2, .size_shift = 16, .size_mask = 0xFFFFu },	/* iv_address_in */
	/* iv_address_out has a fixed firmware-defined length, not carried in msg */
	{ .lsb_idx = 4, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = SE_CMD_ADDR_FIXED_SIZE,
	  .buf_size = ELE_AUTH_ENC_IV_OUT_SIZE },		/* iv_address_out */
	/* key: plaintext-key buffer only when the PLAINTEXT_KEY flag is set */
	{ .lsb_idx = 5, .flag_idx = 2, .flag_mask = 0x00000008u, .is_addr_when_set = true,
	  .size_idx = 7, .size_shift = 16, .size_mask = 0xFFFFu },	/* key_size */
	{ .lsb_idx = 6, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 7, .size_mask = 0xFFFFu },		/* tag_address */
	{ .lsb_idx = 8, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 9, .size_mask = 0xFFFFFFFFu },		/* aad_address */
	{ .lsb_idx = 10, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 11, .size_mask = 0xFFFFFFFFu },		/* input_address */
	{ .lsb_idx = 12, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 13, .size_mask = 0xFFFFFFFFu },		/* output_address */
	{ .lsb_idx = 14, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 15, .size_mask = 0xFFFFu },		/* context_address */
};

/* ELE_SIGNATURE_GENERATE_REQ: digital signature generation. */
static const struct se_cmd_addr_field ele_sign_gen_addr_fields[] = {
	/* key: plaintext-key buffer only when GENERATE_SIGN PLAINTEXT_KEY is set */
	{ .lsb_idx = 1, .flag_idx = 5, .flag_mask = 0x00080000u, .is_addr_when_set = true,
	  .size_idx = 8, .size_mask = 0xFFFFu },		/* priv_key_size */
	{ .lsb_idx = 2, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 4, .size_mask = 0xFFFFFFFFu },		/* message_addr */
	{ .lsb_idx = 3, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 5, .size_mask = 0xFFFFu },		/* signature_addr */
	{ .lsb_idx = 9, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 11, .size_mask = 0xFFFFu },		/* sm2_pub_key_addr */
	{ .lsb_idx = 10, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 11, .size_shift = 16,
	  .size_mask = 0xFFFFu },			/* sm2_id / ml_dsa_ctx addr */
};

static const struct se_cmd_addr_field ele_pub_key_attest_addr_fields[] = {
	/*
	 * PSA_COMPLIANT public-key-attestation payload. sig_gen_hdl is data[0],
	 * key_identifier data[1], key_attestation_id data[2], attest_algo
	 * data[3]. The two buffer addresses that follow are each written
	 * unconditionally via set_phy_addr_to_words() (single LSB word, high
	 * half always zero), so both are always DMA addresses. Each address is
	 * immediately followed by its full u32 byte length.
	 */
	{ .lsb_idx = 4, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 5, .size_mask = 0xFFFFFFFFu },		/* auth_challenge_addr */
	{ .lsb_idx = 6, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 7, .size_mask = 0xFFFFFFFFu },		/* certificate_addr */
};

/* ELE_SIGNATURE_VERIFY_REQ: digital signature verification. */
static const struct se_cmd_addr_field ele_verify_sign_addr_fields[] = {
	/* key: plaintext-key buffer unless the VERIFY_SIGN OPAQUE_KEY flag is set */
	{ .lsb_idx = 1, .flag_idx = 7, .flag_mask = 0x00000008u, .is_addr_when_set = false,
	  .size_idx = 5, .size_shift = 16, .size_mask = 0xFFFFu },	/* key_size */
	{ .lsb_idx = 2, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 4, .size_mask = 0xFFFFFFFFu },		/* msg_addr */
	{ .lsb_idx = 3, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 5, .size_mask = 0xFFFFu },		/* sig_addr */
	{ .lsb_idx = 10, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 11, .size_mask = 0xFFFFu },		/* sm2_id / ml_dsa_ctx addr */
};

static const struct se_cmd_addr_field ele_data_storage_addr_fields[] = {
	/*
	 * PSA_COMPLIANT data-storage payload. data_storage_handle is data[0],
	 * flags/reserved is data[1], data_id is data[2]. data_address (data[3])
	 * is the plaintext data buffer, written unconditionally via
	 * set_phy_addr_to_words() (single LSB word, high half always zero), so
	 * it is always a DMA address. Its length is the full u32 data_size in
	 * data[4].
	 */
	{ .lsb_idx = 3, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 4, .size_mask = 0xFFFFFFFFu },		/* data_address */
};

/* ELE_ASYMMETRIC_ENC_REQ: asymmetric encryption/decryption. */
static const struct se_cmd_addr_field ele_asym_enc_addr_fields[] = {
	/* key_id_addr: plaintext-key buffer only when the PLAINTEXT_KEY flag is set */
	{ .lsb_idx = 1, .flag_idx = 8, .flag_mask = 0x00000008u, .is_addr_when_set = true,
	  .size_idx = 10, .size_mask = 0xFFFFFFFFu },		/* input_plainkey_size */
	{ .lsb_idx = 2, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 5, .size_mask = 0xFFFFFFFFu },		/* plaintext_addr */
	{ .lsb_idx = 3, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 6, .size_mask = 0xFFFFFFFFu },		/* ciphertext_addr */
	{ .lsb_idx = 4, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 7, .size_mask = 0xFFFFFFFFu },		/* label_addr */
};

/* ELE_KEY_GENERIC_CRYPTO_REQ: generic crypto operation with a raw key. */
static const struct se_cmd_addr_field ele_key_generic_crypto_addr_fields[] = {
	/* key_address length is the u8 key_size in the third byte of the iv-size word */
	{ .lsb_idx = 1, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 3, .size_shift = 16, .size_mask = 0xFFu },	/* key_address */
	{ .lsb_idx = 2, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 3, .size_mask = 0xFFFFu },		/* iv_address */
	{ .lsb_idx = 4, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 5, .size_mask = 0xFFFFu },		/* aad_address */
	{ .lsb_idx = 6, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 8, .size_mask = 0xFFFFFFFFu },		/* input_address */
	{ .lsb_idx = 7, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 9, .size_mask = 0xFFFFFFFFu },		/* output_address */
};

/* ELE_GC_CIPHER_REQ: GC symmetric cipher operation. */
static const struct se_cmd_addr_field ele_gc_cipher_addr_fields[] = {
	{ .lsb_idx = 0, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 2, .size_mask = 0xFFFFFFFFu },		/* in_addr */
	{ .lsb_idx = 1, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 2, .size_mask = 0xFFFFFFFFu },		/* out_addr (shares data_size) */
	{ .lsb_idx = 3, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 4, .size_mask = 0xFFFFFFFFu },		/* key_addr */
	{ .lsb_idx = 5, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 6, .size_mask = 0xFFFFFFFFu },		/* iv_addr */
};

/* ELE_GC_AEAD_REQ: GC AEAD operation. */
static const struct se_cmd_addr_field ele_gc_aead_addr_fields[] = {
	{ .lsb_idx = 0, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 2, .size_mask = 0xFFFFFFFFu },		/* in_addr */
	{ .lsb_idx = 1, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 2, .size_mask = 0xFFFFFFFFu },		/* out_addr (shares data_size) */
	{ .lsb_idx = 3, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 4, .size_mask = 0xFFFFFFFFu },		/* key_addr */
	{ .lsb_idx = 5, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 6, .size_mask = 0xFFFFFFFFu },		/* nonce_addr */
	{ .lsb_idx = 7, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 8, .size_mask = 0xFFFFFFFFu },		/* aad_addr */
	{ .lsb_idx = 9, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 10, .size_mask = 0xFFFFFFFFu },		/* tag_addr */
};

/* ELE_GC_ACRYPTO_REQ: GC asymmetric crypto operation. */
static const struct se_cmd_addr_field ele_gc_acrypto_addr_fields[] = {
	{ .lsb_idx = 3, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 5, .size_mask = 0xFFFFFFFFu },		/* data_buff1_addr */
	{ .lsb_idx = 4, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 6, .size_mask = 0xFFFFFFFFu },		/* data_buff2_addr */
	{ .lsb_idx = 7, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 9, .size_mask = 0xFFFFu },		/* key_buff1_addr */
	{ .lsb_idx = 8, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 9, .size_shift = 16, .size_mask = 0xFFFFu },	/* key_buff2_addr */
	{ .lsb_idx = 12, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 13, .size_mask = 0xFFFFu },		/* rsa_label_addr */
};

/* ELE_GC_AKEY_GEN_REQ: GC asymmetric key generation. */
static const struct se_cmd_addr_field ele_gc_akey_gen_addr_fields[] = {
	{ .lsb_idx = 1, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 4, .size_mask = 0xFFFFu },		/* modulus_addr */
	{ .lsb_idx = 2, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 4, .size_shift = 16, .size_mask = 0xFFFFu },	/* priv_buff_addr */
	{ .lsb_idx = 3, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 5, .size_mask = 0xFFFFu },		/* pub_buff_addr */
};

/* ELE_HASH_ONE_GO_REQ: hash one-go operation. */
static const struct se_cmd_addr_field ele_hash_one_go_addr_fields[] = {
	{ .lsb_idx = 1, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 6, .size_shift = 16, .size_mask = 0xFFFFu },	/* ctx_addr */
	{ .lsb_idx = 2, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 4, .size_mask = 0xFFFFFFFFu },		/* input_addr */
	{ .lsb_idx = 3, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 5, .size_mask = 0xFFFFFFFFu },		/* output_addr */
};

static const struct se_cmd_addr_field ele_enc_data_storage_addr_fields[] = {
	/*
	 * PSA_COMPLIANT encrypted-data-storage payload. data_storage_handle is
	 * data[0] and data_id is data[1]. data_address (data[2]) is written
	 * unconditionally via set_phy_addr_to_words() (single LSB word, high
	 * half always zero), so it is always a DMA address; its length is the
	 * full u32 data_size in data[3]. iv_address (data[8]) is only written
	 * when an IV is supplied and is left zero otherwise, so it is an
	 * optional always-address handled by the zero-address skip; its length
	 * is the u16 iv_size in the low half of data[9].
	 */
	{ .lsb_idx = 2, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 3, .size_mask = 0xFFFFFFFFu },		/* data_address */
	{ .lsb_idx = 8, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 9, .size_mask = 0xFFFFu },		/* iv_address */
};

static const struct se_cmd_addr_field ele_rng_get_random_addr_fields[] = {
	/*
	 * PSA_COMPLIANT get-random payload. reserved/flags is data[0]; rnd_addr
	 * (data[1]) is the output buffer, written unconditionally via
	 * set_phy_addr_to_words() (single LSB word, high half always zero), so
	 * it is always a DMA address. Its length is the full u32 rnd_size in
	 * data[2].
	 */
	{ .lsb_idx = 1, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 2, .size_mask = 0xFFFFFFFFu },		/* rnd_addr */
};

static const struct se_cmd_addr_field ele_storage_master_import_addr_fields[] = {
	/*
	 * Storage master-import command (ELE_STORAGE_MASTER_IMPORT_REQ).
	 * Payload layout (data[] = message word minus header word 0):
	 *   data[0] = storage_handle
	 *   data[1] = key_store_address  (LSB; high half always zero)
	 *   data[2] = key_store_size
	 * The address is set unconditionally via set_phy_addr_to_words() and
	 * its length is the full u32 key_store_size.
	 */
	{ .lsb_idx = 1, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 2, .size_mask = 0xFFFFFFFFu },		/* key_store_address */
};

const struct se_cmd_addr_field *ele_fw_cmd_addr_fields(u8 cmd, size_t *count)
{
	switch (cmd) {
	case ELE_PUB_KEY_EXPORT_REQ:
		*count = ARRAY_SIZE(ele_pub_key_export_addr_fields);
		return ele_pub_key_export_addr_fields;
	case ELE_KEYSTORE_REPROV_ENABLE_REQ:
		*count = ARRAY_SIZE(ele_keystore_reprov_en_addr_fields);
		return ele_keystore_reprov_en_addr_fields;
	case ELE_KEYGEN_REQ:
		*count = ARRAY_SIZE(ele_keygen_addr_fields);
		return ele_keygen_addr_fields;
	case ELE_KEY_EXCHANGE_REQ:
		*count = ARRAY_SIZE(ele_key_exchange_addr_fields);
		return ele_key_exchange_addr_fields;
	case ELE_KEY_IMPORT_REQ:
		*count = ARRAY_SIZE(ele_key_import_addr_fields);
		return ele_key_import_addr_fields;
	case ELE_MAC_REQ:
		*count = ARRAY_SIZE(ele_mac_addr_fields);
		return ele_mac_addr_fields;
	case ELE_CIPHER_REQ:
		*count = ARRAY_SIZE(ele_cipher_addr_fields);
		return ele_cipher_addr_fields;
	case ELE_AUTH_ENC_REQ:
		*count = ARRAY_SIZE(ele_auth_enc_addr_fields);
		return ele_auth_enc_addr_fields;
	case ELE_AUTH_ENC_NEW_REQ:
		*count = ARRAY_SIZE(ele_auth_enc_new_addr_fields);
		return ele_auth_enc_new_addr_fields;
	case ELE_SIGNATURE_GENERATE_REQ:
		*count = ARRAY_SIZE(ele_sign_gen_addr_fields);
		return ele_sign_gen_addr_fields;
	case ELE_PUB_KEY_ATTEST_REQ:
		*count = ARRAY_SIZE(ele_pub_key_attest_addr_fields);
		return ele_pub_key_attest_addr_fields;
	case ELE_SIGNATURE_VERIFY_REQ:
		*count = ARRAY_SIZE(ele_verify_sign_addr_fields);
		return ele_verify_sign_addr_fields;
	case ELE_DATA_STORAGE_REQ:
		*count = ARRAY_SIZE(ele_data_storage_addr_fields);
		return ele_data_storage_addr_fields;
	case ELE_ENC_DATA_STORAGE_REQ:
		*count = ARRAY_SIZE(ele_enc_data_storage_addr_fields);
		return ele_enc_data_storage_addr_fields;
	case ELE_ASYMMETRIC_ENC_REQ:
		*count = ARRAY_SIZE(ele_asym_enc_addr_fields);
		return ele_asym_enc_addr_fields;
	case ELE_KEY_GENERIC_CRYPTO_REQ:
		*count = ARRAY_SIZE(ele_key_generic_crypto_addr_fields);
		return ele_key_generic_crypto_addr_fields;
	case ELE_GC_CIPHER_REQ:
		*count = ARRAY_SIZE(ele_gc_cipher_addr_fields);
		return ele_gc_cipher_addr_fields;
	case ELE_GC_AEAD_REQ:
		*count = ARRAY_SIZE(ele_gc_aead_addr_fields);
		return ele_gc_aead_addr_fields;
	case ELE_GC_ACRYPTO_REQ:
		*count = ARRAY_SIZE(ele_gc_acrypto_addr_fields);
		return ele_gc_acrypto_addr_fields;
	case ELE_GC_AKEY_GEN_REQ:
		*count = ARRAY_SIZE(ele_gc_akey_gen_addr_fields);
		return ele_gc_akey_gen_addr_fields;
	case ELE_HASH_ONE_GO_REQ:
		*count = ARRAY_SIZE(ele_hash_one_go_addr_fields);
		return ele_hash_one_go_addr_fields;
	case ELE_RNG_GET_RANDOM_REQ:
		*count = ARRAY_SIZE(ele_rng_get_random_addr_fields);
		return ele_rng_get_random_addr_fields;
	case ELE_STORAGE_MASTER_IMPORT_REQ:
		*count = ARRAY_SIZE(ele_storage_master_import_addr_fields);
		return ele_storage_master_import_addr_fields;
	default:
		*count = 0;
		return NULL;
	}
}

/*
 * FW API for Command Receiver.
 *
 * Storage master-export response (ELE_STORAGE_MASTER_EXPORT_REQ).
 * The cmd_receiver sends this response to firmware to supply the
 * output buffer address. Payload layout:
 *   data[0] = storage_handle
 *   data[1] = rsp_code
 *   data[2] = key_store_export_address  (LSB; high half always zero)
 * No length word is present in the response itself; the export size is
 * taken from the FW command received earlier (key_store_size) and stored
 * in se_if_priv.cmd_rcvr_var_size by cmd_receiver_specific_ops().
 * se_val_cmd_addrs() reads it when size_idx == SE_CMD_RCVR_ADDR_VAR_SIZE
 * to range-check the full response buffer before it is forwarded to FW.
 */
static const struct se_cmd_addr_field ele_storage_master_export_addr_fields[] = {
	{ .lsb_idx = 2, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = SE_CMD_RCVR_ADDR_VAR_SIZE },		/* key_store_export_address */
};

/*
 * Storage chunk-get response (ELE_STORAGE_CHUNK_GET_REQ).
 * The cmd_receiver fills in the chunk buffer address and its size so
 * firmware can DMA the chunk data into the kernel's coherent buffer.
 * Payload layout:
 *   data[0] = chunk_size
 *   data[1] = chunk_addr  (LSB; high half always zero)
 *   data[2] = rsp_code
 * The size word precedes the address in the message.
 */
static const struct se_cmd_addr_field ele_storage_chunk_get_addr_fields[] = {
	{ .lsb_idx = 1, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = 0, .size_mask = 0xFFFFFFFFu },		/* chunk_addr */
};

/*
 * Storage chunk-export response (ELE_STORAGE_CHUNK_EXPORT_REQ).
 * The cmd_receiver supplies the output buffer address. Payload layout:
 *   data[0] = rsp_code
 *   data[1] = chunk_export_address  (LSB; high half always zero)
 * No length word is present in the response itself; the export size is
 * taken from the FW command received earlier (chunk_size) and stored
 * in se_if_priv.cmd_rcvr_var_size by cmd_receiver_specific_ops().
 * se_val_cmd_addrs() reads it when size_idx == SE_CMD_RCVR_ADDR_VAR_SIZE
 * to range-check the full response buffer before it is forwarded to FW.
 */
static const struct se_cmd_addr_field ele_storage_chunk_export_addr_fields[] = {
	{ .lsb_idx = 1, .flag_idx = SE_CMD_ADDR_ALWAYS,
	  .size_idx = SE_CMD_RCVR_ADDR_VAR_SIZE },		/* chunk_export_address */
};

/*
 * Return the address-field descriptor table for a cmd_receiver response
 * message (rsp_tag), or NULL when the response embeds no DMA addresses.
 * count is set to the number of entries. Only the three storage responses
 * that supply a kernel buffer address to firmware are covered here;
 * ELE_STORAGE_EXPORT_FINISH_REQ, ELE_STORAGE_CHUNK_GET_DONE_REQ, and
 * ELE_STORAGE_CHUNK_DELETE_REQ carry no DMA addresses and return NULL.
 */
const struct se_cmd_addr_field *ele_fw_rsp_addr_fields(u8 cmd, size_t *count)
{
	switch (cmd) {
	case ELE_STORAGE_MASTER_EXPORT_REQ:
		*count = ARRAY_SIZE(ele_storage_master_export_addr_fields);
		return ele_storage_master_export_addr_fields;
	case ELE_STORAGE_CHUNK_GET_REQ:
		*count = ARRAY_SIZE(ele_storage_chunk_get_addr_fields);
		return ele_storage_chunk_get_addr_fields;
	case ELE_STORAGE_CHUNK_EXPORT_REQ:
		*count = ARRAY_SIZE(ele_storage_chunk_export_addr_fields);
		return ele_storage_chunk_export_addr_fields;
	default:
		*count = 0;
		return NULL;
	}
}
