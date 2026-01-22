/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2025 NXP
 *
 * Header file for the EdgeLock Enclave Base API(s).
 */

#ifndef ELE_BASE_MSG_H
#define ELE_BASE_MSG_H

#include <linux/device.h>
#include <linux/types.h>

#include "se_ctrl.h"

#define ELE_NONE_VAL			0x0

#define ELE_GET_INFO_REQ		0xda
#define ELE_GET_INFO_REQ_MSG_SZ		0x10
#define ELE_GET_INFO_RSP_MSG_SZ		0x08

#define MAX_UID_SIZE                     (16)
#define DEV_GETINFO_ROM_PATCH_SHA_SZ     (32)
#define DEV_GETINFO_FW_SHA_SZ            (32)
#define DEV_GETINFO_OEM_SRKH_SZ          (64)
#define DEV_GETINFO_MIN_VER_MASK	0xff
#define DEV_GETINFO_MAJ_VER_MASK	0xff00
#define ELE_DEV_INFO_EXTRA_SZ		0x60

struct dev_info {
	u8  cmd;
	u8  ver;
	u16 length;
	u16 soc_id;
	u16 soc_rev;
	u16 lmda_val;
	u8  ssm_state;
	u8  dev_atts_api_ver;
	u8  uid[MAX_UID_SIZE];
	u8  sha_rom_patch[DEV_GETINFO_ROM_PATCH_SHA_SZ];
	u8  sha_fw[DEV_GETINFO_FW_SHA_SZ];
};

struct dev_addn_info {
	u8  oem_srkh[DEV_GETINFO_OEM_SRKH_SZ];
	u8  trng_state;
	u8  csal_state;
	u8  imem_state;
	u8  reserved2;
};

struct ele_dev_info {
	struct dev_info d_info;
	struct dev_addn_info d_addn_info;
};

#define ELE_GET_INFO_BUFF_SZ		(sizeof(struct ele_dev_info) \
						+ ELE_DEV_INFO_EXTRA_SZ)

#define GET_SERIAL_NUM_FROM_UID(x, uid_word_sz) ({\
	const u32 *__x = (const u32 *)(x); \
	size_t __sz = (uid_word_sz); \
	((u64)__x[__sz - 1] << 32) | __x[0]; \
	})

#define ELE_MAX_DBG_DMP_PKT		50
#define ELE_DEBUG_DUMP_REQ		0x21
#define ELE_DEBUG_DUMP_REQ_SZ		0x4
#define ELE_DEBUG_DUMP_RSP_SZ		0x5c

#define ELE_PING_REQ			0x01
#define ELE_PING_REQ_SZ			0x04
#define ELE_PING_RSP_SZ			0x08

#define ELE_SERVICE_SWAP_REQ		0xdf
#define ELE_SERVICE_SWAP_REQ_MSG_SZ	0x18
#define ELE_SERVICE_SWAP_RSP_MSG_SZ	0x0c
#define ELE_IMEM_SIZE			0x10000
#define ELE_IMEM_STATE_OK		0xca
#define ELE_IMEM_STATE_BAD		0xfe
#define ELE_IMEM_STATE_WORD		0x27
#define ELE_IMEM_STATE_MASK		0x00ff0000
#define ELE_IMEM_EXPORT			0x1
#define ELE_IMEM_IMPORT			0x2

#define ELE_FW_AUTH_REQ			0x02
#define ELE_FW_AUTH_REQ_SZ		0x10
#define ELE_FW_AUTH_RSP_MSG_SZ		0x08

int ele_get_info(struct se_if_priv *priv, struct ele_dev_info *s_info);
int ele_fetch_soc_info(struct se_if_priv *priv, void *data);
int ele_ping(struct se_if_priv *priv);
int ele_service_swap(struct se_if_priv *priv, phys_addr_t addr,
		     u32 addr_size, u16 flag);
int ele_fw_authenticate(struct se_if_priv *priv, phys_addr_t contnr_addr,
			phys_addr_t img_addr);
int ele_debug_dump(struct se_if_priv *priv);
#endif
