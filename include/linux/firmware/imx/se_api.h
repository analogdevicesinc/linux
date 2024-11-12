/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2024 NXP
 */

#ifndef __SE_API_H__
#define __SE_API_H__

#include <linux/types.h>

#define SOC_ID_OF_IMX8ULP		0x084D
#define SOC_ID_OF_IMX8DXL		0xE
#define SOC_ID_OF_IMX8QM		0x1
#define SOC_ID_OF_IMX8QXP		0x2
#define SOC_ID_OF_IMX93			0x9300
#define SOC_ID_OF_IMX95			0x9500

#define OTP_UNIQ_ID			0x01
#define OTFAD_CONFIG			0x2

void *imx_get_se_data_info(uint32_t soc_id, u32 idx);

int imx_se_write_fuse(void *se_if_data, uint16_t fuse_index,
		   u32 value, bool block);
int imx_se_voltage_change_req(void *se_if_data, bool start);
int imx_se_read_fuse(void *se_if_data,
		     uint16_t fuse_id, u32 *value);

#endif /* __SE_API_H__ */
