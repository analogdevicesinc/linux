/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 2026 Intel Corporation */

#ifndef IPU7_BOOT_H
#define IPU7_BOOT_H

#include "ipu7-fw-com.h"

#define IPU7_BOOT_MSG_VER_MAX_ENTRIES	3U

struct ipu7_boot_abi_version {
	u8 patch;
	u8 subminor;
	u8 minor;
	u8 major;
};

struct ipu7_boot_abi_msg_versions {
	u8 num_versions;
	u8 reserved[3];
	struct ipu7_boot_abi_version versions[IPU7_BOOT_MSG_VER_MAX_ENTRIES];
};

struct ipu7_boot_abi_cfg {
	u32 length;
	struct ipu7_boot_abi_version config_version;
	struct ipu7_boot_abi_msg_versions client_version_support;
	u32 pkg_dir;
	u32 subsys_config;
	u32 uc_tile_frequency;
	u16 checksum;
	u8 uc_tile_frequency_units;
	u8 padding[1];
	u32 reserved[58];
	struct ipu7_fw_com_config fw_com_config;
} __packed;

int ipu6_ipu7_init_boot_config(struct ipu6_bus_device *adev,
			       struct ipu7_fw_com_queue_config *qconfigs,
			       int num_queues, u32 uc_freq,
			       dma_addr_t subsys_config, u8 major);
void ipu6_ipu7_release_boot_config(struct ipu6_bus_device *adev);
int ipu6_ipu7_boot_start_fw(const struct ipu6_bus_device *adev);
int ipu6_ipu7_boot_stop_fw(const struct ipu6_bus_device *adev);

#endif
