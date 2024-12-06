/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2024 NXP
 */

#ifndef SECO_H
#define SECO_H

#include "se_ctrl.h"

#define SCU_MEM_CFG			BIT(1)
#define SCU_SIGNED_MSG_CFG		BIT(2)
#define SECO_SUCCESS_IND		0x00

#define SOC_REV_A1			0xA100
#define SOC_REV_B0			0xB000
#define SOC_REV_C0			0xC000

#define SECURE_RAM_BASE_ADDRESS         (0x31800000ULL)
#define SECURE_RAM_BASE_ADDRESS_SCU	(0x20800000u)
#define SECURE_RAM_SIZE                 (0x10000ULL)

#define V2X_NON_FIPS			0x00000c00
#define SECO_NON_FIPS			0x00000018

#define IMX8DXL_DL1    0x1
#define IMX8DXL_DL2    0x2
#define IMX8DXL_DL3    0x4

struct seco_soc_info {
	u16 soc_id;
	u16 soc_rev;
	u16 board_type;
};

int seco_fetch_soc_info(struct se_if_priv *priv, void *data);
int imx_scu_init_fw(struct se_if_priv *priv);
int imx_scu_sec_mem_cfg(struct file *fp, uint32_t offset, uint32_t size);
int imx_scu_mem_access(struct se_if_device_ctx *dev_ctx);
int imx_scu_signed_msg(struct file *fp,
		       uint8_t *msg,
		       uint32_t size,
		       uint32_t *error);
#endif
