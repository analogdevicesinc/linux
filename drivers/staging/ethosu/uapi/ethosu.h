/*
 * Copyright (c) 2020-2022 Arm Limited.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU licence.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#ifndef ETHOSU_H
#define ETHOSU_H

/****************************************************************************
 * Includes
 ****************************************************************************/

#include <linux/ioctl.h>
#include <linux/types.h>

#ifdef __cplusplus
namespace EthosU {
#endif

/****************************************************************************
 * Defines
 ****************************************************************************/

#define ETHOSU_IOCTL_BASE               0x01
#define ETHOSU_IO(nr)                   _IO(ETHOSU_IOCTL_BASE, nr)
#define ETHOSU_IOR(nr, type)            _IOR(ETHOSU_IOCTL_BASE, nr, type)
#define ETHOSU_IOW(nr, type)            _IOW(ETHOSU_IOCTL_BASE, nr, type)
#define ETHOSU_IOWR(nr, type)           _IOWR(ETHOSU_IOCTL_BASE, nr, type)

#define ETHOSU_IOCTL_PING               ETHOSU_IO(0x00)
#define ETHOSU_IOCTL_VERSION_REQ        ETHOSU_IO(0x01)
#define ETHOSU_IOCTL_CAPABILITIES_REQ   ETHOSU_IOR(0x02, \
						   struct ethosu_uapi_device_capabilities)
#define ETHOSU_IOCTL_BUFFER_CREATE      ETHOSU_IOR(0x10, \
						   struct ethosu_uapi_buffer_create)
#define ETHOSU_IOCTL_BUFFER_SET         ETHOSU_IOR(0x11, \
						   struct ethosu_uapi_buffer)
#define ETHOSU_IOCTL_BUFFER_GET         ETHOSU_IOW(0x12, \
						   struct ethosu_uapi_buffer)
#define ETHOSU_IOCTL_NETWORK_CREATE     ETHOSU_IOR(0x20, \
						   struct ethosu_uapi_network_create)
#define ETHOSU_IOCTL_NETWORK_INFO       ETHOSU_IOR(0x21, \
						   struct ethosu_uapi_network_info)
#define ETHOSU_IOCTL_INFERENCE_CREATE   ETHOSU_IOR(0x30, \
						   struct ethosu_uapi_inference_create)
#define ETHOSU_IOCTL_INFERENCE_STATUS   ETHOSU_IOR(0x31, \
						   struct ethosu_uapi_result_status)
#define ETHOSU_IOCTL_INFERENCE_CANCEL   ETHOSU_IOR(0x32, \
						   struct ethosu_uapi_cancel_inference_status)

/* Maximum number of IFM/OFM file descriptors per network */
#define ETHOSU_FD_MAX                   16

/* Maximum number of dimensions for input and output */
#define ETHOSU_DIM_MAX                   8

/* Maximum number of PMUs available */
#define ETHOSU_PMU_EVENT_MAX             4

/****************************************************************************
 * Types
 ****************************************************************************/

/**
 * enum ethosu_uapi_status - Status
 */
enum ethosu_uapi_status {
	ETHOSU_UAPI_STATUS_OK,
	ETHOSU_UAPI_STATUS_ERROR,
	ETHOSU_UAPI_STATUS_RUNNING,
	ETHOSU_UAPI_STATUS_REJECTED,
	ETHOSU_UAPI_STATUS_ABORTED,
	ETHOSU_UAPI_STATUS_ABORTING,
};

/**
 * struct ethosu_uapi_buffer_create - Create buffer request
 * @capacity:	Maximum capacity of the buffer
 */
struct ethosu_uapi_buffer_create {
	__u32 capacity;
};

/**
 * struct ethosu_uapi_buffer - Buffer descriptor
 * @offset:	Offset to where the data starts
 * @size:	Size of the data
 *
 * 'offset + size' must not exceed the capacity of the buffer.
 */
struct ethosu_uapi_buffer {
	__u32 offset;
	__u32 size;
};

/**
 * enum ethosu_uapi_network_create - Network buffer type.
 * @ETHOSU_UAPI_NETWORK_BUFFER:	Network is stored in a buffer handle.
 * @ETHOSU_UAPI_NETWORK_INDEX:	Network is built into firmware and referenced by
 *                              index.
 */
enum ethosu_uapi_network_type {
	ETHOSU_UAPI_NETWORK_BUFFER = 1,
	ETHOSU_UAPI_NETWORK_INDEX
};

/**
 * struct ethosu_uapi_network_create - Create network request
 * @type:	Buffer type. See @ethosu_uapi_network_type.
 * @fd:		Buffer file descriptor
 * @index:	Buffer index compiled into firmware binary.
 */
struct ethosu_uapi_network_create {
	u32 type;
	union {
		__u32 fd;
		__u32 index;
	};
};

/**
 * struct ethosu_uapi_network_info - Network info
 * @desc:		Network description
 * @ifm_count:		Number of IFM buffers
 * @ifm_size:		IFM buffer sizes
 * @ifm_types:          IFM data types
 * @ifm_offset:         IFM data offset in arena
 * @ifm_dims:           IFM buffer dimensions
 * @ifm_shapes:         IFM buffer shapes
 * @ofm_count:		Number of OFM buffers
 * @ofm_size:		OFM buffer sizes
 * @ofm_offset:         OFM data offset in arena
 * @ofm_dims:           OFM buffer dimensions
 * @ofm_shapes:         OFM buffer shapes
 */
struct ethosu_uapi_network_info {
	char  desc[32];
	__u32 is_vela;
	__u32 ifm_count;
	__u32 ifm_size[ETHOSU_FD_MAX];
	__u32 ifm_types[ETHOSU_FD_MAX];
	__u32 ifm_offset[ETHOSU_FD_MAX];
	__u32 ifm_dims[ETHOSU_FD_MAX];
	__u32 ifm_shapes[ETHOSU_FD_MAX][ETHOSU_DIM_MAX];
	__u32 ofm_count;
	__u32 ofm_size[ETHOSU_FD_MAX];
	__u32 ofm_types[ETHOSU_FD_MAX];
	__u32 ofm_offset[ETHOSU_FD_MAX];
	__u32 ofm_dims[ETHOSU_FD_MAX];
	__u32 ofm_shapes[ETHOSU_FD_MAX][ETHOSU_DIM_MAX];
};

/**
 * struct ethosu_uapi_pmu_config - Configure performance counters
 * @events:             Array of counters to configure, set to non-zero for
 *                      each counter to enable corresponding event.
 * @cycle_count:        Set to enable the cycle counter.
 */
struct ethosu_uapi_pmu_config {
	__u32 events[ETHOSU_PMU_EVENT_MAX];
	__u32 cycle_count;
};

/**
 * struct ethosu_uapi_pmu_counts - Status of performance counters
 * @events:             Count for respective configured events.
 * @cycle_count:        Count for cycle counter.
 */
struct ethosu_uapi_pmu_counts {
	__u32 events[ETHOSU_PMU_EVENT_MAX];
	__u64 cycle_count;
};

/**
 * struct ethosu_uapi_device_hw_id - Device hardware identification
 * @version_status:            Version status
 * @version_minor:             Version minor
 * @version_major:             Version major
 * @product_major:             Product major
 * @arch_patch_rev:            Architecture version patch
 * @arch_minor_rev:            Architecture version minor
 * @arch_major_rev:            Architecture version major
 */
struct ethosu_uapi_device_hw_id {
	__u32 version_status;
	__u32 version_minor;
	__u32 version_major;
	__u32 product_major;
	__u32 arch_patch_rev;
	__u32 arch_minor_rev;
	__u32 arch_major_rev;
};

/**
 * struct ethosu_uapi_device_hw_cfg - Device hardware configuration
 * @macs_per_cc:               MACs per clock cycle
 * @cmd_stream_version:        NPU command stream version
 * @custom_dma:                Custom DMA enabled
 */
struct ethosu_uapi_device_hw_cfg {
	__u32 macs_per_cc;
	__u32 cmd_stream_version;
	__u32 custom_dma;
};

/**
 * struct ethosu_uapi_device_capabilities - Device capabilities
 * @hw_id:                     Hardware identification
 * @hw_cfg:                    Hardware configuration
 * @driver_patch_rev:          Driver version patch
 * @driver_minor_rev:          Driver version minor
 * @driver_major_rev:          Driver version major
 */
struct ethosu_uapi_device_capabilities {
	struct ethosu_uapi_device_hw_id  hw_id;
	struct ethosu_uapi_device_hw_cfg hw_cfg;
	__u32                            driver_patch_rev;
	__u32                            driver_minor_rev;
	__u32                            driver_major_rev;
};

/**
 * enum ethosu_uapi_inference_type - Inference type
 */
enum ethosu_uapi_inference_type {
	ETHOSU_UAPI_INFERENCE_MODEL = 0,
	ETHOSU_UAPI_INFERENCE_OP
};

/**
 * struct ethosu_uapi_inference_create - Create network request
 * @ifm_count:		Number of IFM file descriptors
 * @ifm_fd:		IFM buffer file descriptors
 * @ofm_count:		Number of OFM file descriptors
 * @ofm_fd:		OFM buffer file descriptors
 */
struct ethosu_uapi_inference_create {
	__u32                         ifm_count;
	__u32                         ifm_fd[ETHOSU_FD_MAX];
	__u32                         ofm_count;
	__u32                         ofm_fd[ETHOSU_FD_MAX];
	enum ethosu_uapi_inference_type inference_type;
	struct ethosu_uapi_pmu_config pmu_config;
};

/**
 * struct ethosu_uapi_result_status - Status of inference
 * @status	Status of run inference.
 * @pmu_config	Configured performance counters.
 * @pmu_count	Perfomance counters values, when status is
 *              ETHOSU_UAPI_STATUS_OK.
 */
struct ethosu_uapi_result_status {
	enum ethosu_uapi_status       status;
	struct ethosu_uapi_pmu_config pmu_config;
	struct ethosu_uapi_pmu_counts pmu_count;
};

/**
 * struct ethosu_uapi_cancel_status - Status of inference cancellation.
 * @status	OK if inference cancellation was performed, ERROR otherwise.
 */
struct ethosu_uapi_cancel_inference_status {
	enum ethosu_uapi_status status;
};

#ifdef __cplusplus
} /* namespace EthosU */
#endif
#endif
