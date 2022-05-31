/*
 * (C) COPYRIGHT 2020 ARM Limited. All rights reserved.
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
#define ETHOSU_IOCTL_CAPABILITIES_REQ   ETHOSU_IO(0x02)
#define ETHOSU_IOCTL_BUFFER_CREATE      ETHOSU_IOR(0x10, \
						   struct ethosu_uapi_buffer_create)
#define ETHOSU_IOCTL_BUFFER_SET         ETHOSU_IOR(0x11, \
						   struct ethosu_uapi_buffer)
#define ETHOSU_IOCTL_BUFFER_GET         ETHOSU_IOW(0x12, \
						   struct ethosu_uapi_buffer)
#define ETHOSU_IOCTL_NETWORK_CREATE     ETHOSU_IOR(0x20, \
						   struct ethosu_uapi_network_create)
#define ETHOSU_IOCTL_INFERENCE_CREATE   ETHOSU_IOR(0x30, \
						   struct ethosu_uapi_inference_create)
#define ETHOSU_IOCTL_INFERENCE_STATUS   ETHOSU_IOR(0x31, \
						   struct ethosu_uapi_result_status)

/* Maximum number of IFM/OFM file descriptors per network */
#define ETHOSU_FD_MAX                   16

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
	ETHOSU_UAPI_STATUS_ERROR
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
 * struct ethosu_uapi_network_create - Create network request
 * @fd:		Buffer file descriptor
 */
struct ethosu_uapi_network_create {
	__u32 fd;
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
 * struct ethosu_uapi_capabilities - Device capabilities
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

#ifdef __cplusplus
} /* namespace EthosU */
#endif
#endif
