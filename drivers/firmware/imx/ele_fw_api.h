/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2026 NXP
 */

#ifndef ELE_FW_API_H
#define ELE_FW_API_H
#include "se_ctrl.h"

#define ELE_SESSION_OPEN_REQ            0x10u
#define ELE_SESSION_OPEN_RSP_SZ         0x0Cu

#define ELE_SESSION_CLOSE_REQ_SZ	0x08u
#define ELE_SESSION_CLOSE_RSP_SZ	0x08u
#define ELE_SESSION_CLOSE_REQ           0x11u

/*
 * Session-scoped FW-API service, key-management and close command opcodes
 * (SAB command IDs, PSA_COMPLIANT message layout). These commands do not
 * embed DMA staging-buffer addresses that require range-checking; they are
 * defined here for completeness and for use by the command allow-list.
 * ELE_FW_GET_INFO_REQ is the FW-API get-info opcode and is intentionally
 * distinct from the base-API ELE_GET_INFO_REQ (0xda) in ele_base_msg.h.
 */
#define ELE_FW_GET_INFO_REQ             0x16u
#define ELE_KEY_STORE_OPEN_REQ          0x30u
#define ELE_KEY_STORE_CLOSE_REQ         0x31u
#define ELE_KEY_MGMT_OPEN_REQ           0x40u
#define ELE_KEY_MGMT_CLOSE_REQ          0x41u
#define ELE_MANAGE_KEY_GROUP_REQ        0x45u
#define ELE_GET_KEY_ATTR_REQ            0x4Cu
#define ELE_KEY_DELETE_REQ              0x4Eu
#define ELE_MAC_OPEN_REQ                0x50u
#define ELE_MAC_CLOSE_REQ               0x51u
#define ELE_CIPHER_OPEN_REQ             0x60u
#define ELE_CIPHER_CLOSE_REQ            0x61u
#define ELE_SIGNATURE_GENERATE_OPEN_REQ 0x70u
#define ELE_SIGNATURE_GENERATE_CLOSE_REQ 0x71u
#define ELE_SIGNATURE_VERIFY_OPEN_REQ   0x80u
#define ELE_SIGNATURE_VERIFY_CLOSE_REQ  0x81u
#define ELE_DATA_STORAGE_OPEN_REQ       0xA0u
#define ELE_DATA_STORAGE_CLOSE_REQ      0xA1u
#define ELE_DATA_DELETE_REQ             0xA4u

/*
 * FW-API crypto command opcodes that embed one or more DMA physical addresses
 * in their message payload. ele_uapi_allowed_fw_cmd() range-checks those
 * addresses against the calling context's shared-memory window before the
 * message is handed to firmware. Opcodes match the SAB command IDs emitted by
 * the userspace library (PSA_COMPLIANT message layout).
 */
#define ELE_PUB_KEY_EXPORT_REQ          0x32u
#define ELE_KEYSTORE_REPROV_ENABLE_REQ  0x3Fu
#define ELE_KEYGEN_REQ                  0x42u
#define ELE_KEY_EXCHANGE_REQ            0x47u
#define ELE_KEY_IMPORT_REQ              0x4Fu
#define ELE_KEY_IMPORT                  0x4Fu
#define ELE_MAC_REQ                     0x52u
#define ELE_CIPHER_REQ                  0x62u
#define ELE_AUTH_ENC_REQ                0x64u
#define ELE_AUTH_ENC_NEW_REQ            0x65u
#define ELE_SIGNATURE_GENERATE_REQ      0x72u
#define ELE_PUB_KEY_ATTEST_REQ          0x74u
#define ELE_SIGNATURE_VERIFY_REQ        0x82u
#define ELE_DATA_STORAGE_REQ            0xA2u
#define ELE_ENC_DATA_STORAGE_REQ        0xA3u
#define ELE_ASYMMETRIC_ENC_REQ          0x92u

#define ELE_KEY_GENERIC_CRYPTO_REQ      0xC2u
#define ELE_GC_CIPHER_REQ               0xC8u
#define ELE_GC_AEAD_REQ                 0xC9u
#define ELE_GC_ACRYPTO_REQ              0xCAu
#define ELE_GC_AKEY_GEN_REQ             0xCBu
#define ELE_HASH_ONE_GO_REQ             0xCCu
#define ELE_RNG_GET_RANDOM_REQ          0xCDu

#define ELE_STORAGE_OPEN_REQ            0xE0u
#define ELE_STORAGE_OPEN_RSP_SZ         0x0Cu

#define ELE_STORAGE_CLOSE_REQ_SZ	0x08u
#define ELE_STORAGE_CLOSE_RSP_SZ	0x08u
#define ELE_STORAGE_CLOSE_REQ           0xE1u

#define ELE_STORAGE_MASTER_IMPORT_REQ   0xE2u
#define ELE_STORAGE_MASTER_EXPORT_REQ   0xE3u
#define ELE_STORAGE_EXPORT_FINISH_REQ   0xE4u
#define ELE_STORAGE_CHUNK_EXPORT_REQ    0xE5u
#define ELE_STORAGE_CHUNK_GET_REQ       0xE6u
#define ELE_STORAGE_CHUNK_GET_DONE_REQ  0xE7u
#define ELE_STORAGE_CHUNK_DELETE_REQ    0xE9u
#define ELE_STORAGE_STATUS_REQ          0xEAu

int ele_uapi_allowed_fw_rsp(struct se_if_device_ctx *dev_ctx, struct se_msg_hdr *header,
			    u32 tx_msg_sz);
int ele_uapi_allowed_fw_cmd(struct se_if_device_ctx *dev_ctx, struct se_msg_hdr *header,
			    u32 tx_msg_sz, u32 rx_msg_sz);
int fw_api_specific_ops(struct se_if_device_ctx *dev_ctx, struct se_api_msg *rx_msg,
			bool is_cmd_interrupted);
void cmd_receiver_specific_ops(struct se_if_device_ctx *dev_ctx,
			       struct se_api_msg *rx_msg);
int se_close_session(struct se_if_device_ctx *dev_ctx, u32 session_hdl);
int se_close_storage(struct se_if_device_ctx *dev_ctx, u32 storage_hdl);

#endif /* ELE_FW_API_H */
