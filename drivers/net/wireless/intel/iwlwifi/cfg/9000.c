// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/*
 * Copyright (C) 2015-2017 Intel Deutschland GmbH
 * Copyright (C) 2018-2021, 2023, 2025 Intel Corporation
 */
#include <linux/module.h>
#include <linux/stringify.h>
#include "iwl-config.h"
#include "fw/file.h"
#include "iwl-prph.h"

/* Highest firmware API version supported */
#define IWL9000_UCODE_API_MAX	46

/* Lowest firmware API version supported */
#define IWL9000_UCODE_API_MIN	30

/* Memory offsets and lengths */
#define IWL9000_SMEM_OFFSET		0x400000
#define IWL9000_SMEM_LEN		0x68000

#define  IWL9000_FW_PRE "iwlwifi-9000-pu-b0-jf-b0"
#define  IWL9260_FW_PRE "iwlwifi-9260-th-b0-jf-b0"
#define IWL9000_MODULE_FIRMWARE(api) \
	IWL9000_FW_PRE "-" __stringify(api) ".ucode"
#define IWL9260_MODULE_FIRMWARE(api) \
	IWL9260_FW_PRE "-" __stringify(api) ".ucode"

static const struct iwl_family_base_params iwl9000_base = {
	.eeprom_size = OTP_LOW_IMAGE_SIZE_32K,
	.num_of_queues = 31,
	.max_tfd_queue_size = 256,
	.shadow_ram_support = true,
	.led_compensation = 57,
	.wd_timeout = IWL_LONG_WD_TIMEOUT,
	.max_event_log_size = 512,
	.shadow_reg_enable = true,
	.pcie_l1_allowed = true,
	.smem_offset = IWL9000_SMEM_OFFSET,
	.smem_len = IWL9000_SMEM_LEN,
	.features = IWL_TX_CSUM_NETIF_FLAGS | NETIF_F_RXCSUM,
	.apmg_not_supported = true,
	.mac_addr_from_csr = 0x380,
	.d3_debug_data_base_addr = 0x401000,
	.d3_debug_data_length = 92 * 1024,
	.nvm_hw_section_num = 10,
	.mon_smem_regs = {
		.write_ptr = {
			.addr = LDBG_M2S_BUF_WPTR,
			.mask = LDBG_M2S_BUF_WPTR_VAL_MSK,
		},
		.cycle_cnt = {
			.addr = LDBG_M2S_BUF_WRAP_CNT,
			.mask = LDBG_M2S_BUF_WRAP_CNT_VAL_MSK,
		},
	},
	.mon_dram_regs = {
		.write_ptr = {
			.addr = MON_BUFF_WRPTR_VER2,
			.mask = 0xffffffff,
		},
		.cycle_cnt = {
			.addr = MON_BUFF_CYCLE_CNT_VER2,
			.mask = 0xffffffff,
		},
	},
	.ucode_api_max = IWL9000_UCODE_API_MAX,
	.ucode_api_min = IWL9000_UCODE_API_MIN,
};

const struct iwl_mac_cfg iwl9000_mac_cfg = {
	.device_family = IWL_DEVICE_FAMILY_9000,
	.base = &iwl9000_base,
	.mq_rx_supported = true,
};

const struct iwl_mac_cfg iwl9560_mac_cfg = {
	.device_family = IWL_DEVICE_FAMILY_9000,
	.base = &iwl9000_base,
	.mq_rx_supported = true,
	.integrated = true,
	.xtal_latency = 650,
};

const struct iwl_mac_cfg iwl9560_long_latency_mac_cfg = {
	.device_family = IWL_DEVICE_FAMILY_9000,
	.base = &iwl9000_base,
	.mq_rx_supported = true,
	.integrated = true,
	.xtal_latency = 2820,
};

const struct iwl_mac_cfg iwl9560_shared_clk_mac_cfg = {
	.device_family = IWL_DEVICE_FAMILY_9000,
	.base = &iwl9000_base,
	.mq_rx_supported = true,
	.integrated = true,
	.xtal_latency = 670,
	.extra_phy_cfg_flags = FW_PHY_CFG_SHARED_CLK
};

MODULE_FIRMWARE(IWL9000_MODULE_FIRMWARE(IWL9000_UCODE_API_MAX));
MODULE_FIRMWARE(IWL9260_MODULE_FIRMWARE(IWL9000_UCODE_API_MAX));
