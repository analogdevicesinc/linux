/* SPDX-License-Identifier: GPL-2.0 */
/*
 *
 * (C) COPYRIGHT 2015, 2020-2021 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU license.
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
 */

#ifndef _UAPI_KBASE_HWCNT_READER_H_
#define _UAPI_KBASE_HWCNT_READER_H_

#include <stddef.h>
#include <linux/types.h>

/* The ids of ioctl commands. */
#define KBASE_HWCNT_READER 0xBE
#define KBASE_HWCNT_READER_GET_HWVER       _IOR(KBASE_HWCNT_READER, 0x00, __u32)
#define KBASE_HWCNT_READER_GET_BUFFER_SIZE _IOR(KBASE_HWCNT_READER, 0x01, __u32)
#define KBASE_HWCNT_READER_DUMP            _IOW(KBASE_HWCNT_READER, 0x10, __u32)
#define KBASE_HWCNT_READER_CLEAR           _IOW(KBASE_HWCNT_READER, 0x11, __u32)
#define KBASE_HWCNT_READER_GET_BUFFER      _IOC(_IOC_READ, KBASE_HWCNT_READER, 0x20,\
		offsetof(struct kbase_hwcnt_reader_metadata, cycles))
#define KBASE_HWCNT_READER_GET_BUFFER_WITH_CYCLES      _IOR(KBASE_HWCNT_READER, 0x20,\
		struct kbase_hwcnt_reader_metadata)
#define KBASE_HWCNT_READER_PUT_BUFFER      _IOC(_IOC_WRITE, KBASE_HWCNT_READER, 0x21,\
		offsetof(struct kbase_hwcnt_reader_metadata, cycles))
#define KBASE_HWCNT_READER_PUT_BUFFER_WITH_CYCLES      _IOW(KBASE_HWCNT_READER, 0x21,\
		struct kbase_hwcnt_reader_metadata)
#define KBASE_HWCNT_READER_SET_INTERVAL    _IOW(KBASE_HWCNT_READER, 0x30, __u32)
#define KBASE_HWCNT_READER_ENABLE_EVENT    _IOW(KBASE_HWCNT_READER, 0x40, __u32)
#define KBASE_HWCNT_READER_DISABLE_EVENT   _IOW(KBASE_HWCNT_READER, 0x41, __u32)
#define KBASE_HWCNT_READER_GET_API_VERSION _IOW(KBASE_HWCNT_READER, 0xFF, __u32)
#define KBASE_HWCNT_READER_GET_API_VERSION_WITH_FEATURES \
		_IOW(KBASE_HWCNT_READER, 0xFF, \
		     struct kbase_hwcnt_reader_api_version)

/**
 * struct kbase_hwcnt_reader_metadata_cycles - GPU clock cycles
 * @top:           the number of cycles associated with the main clock for the
 *                 GPU
 * @shader_cores:  the cycles that have elapsed on the GPU shader cores
 */
struct kbase_hwcnt_reader_metadata_cycles {
	__u64 top;
	__u64 shader_cores;
};

/**
 * struct kbase_hwcnt_reader_metadata - hwcnt reader sample buffer metadata
 * @timestamp:  time when sample was collected
 * @event_id:   id of an event that triggered sample collection
 * @buffer_idx: position in sampling area where sample buffer was stored
 * @cycles:     the GPU cycles that occurred since the last sample
 */
struct kbase_hwcnt_reader_metadata {
	__u64 timestamp;
	__u32 event_id;
	__u32 buffer_idx;
	struct kbase_hwcnt_reader_metadata_cycles cycles;
};

/**
 * enum base_hwcnt_reader_event - hwcnt dumping events
 * @BASE_HWCNT_READER_EVENT_MANUAL:   manual request for dump
 * @BASE_HWCNT_READER_EVENT_PERIODIC: periodic dump
 * @BASE_HWCNT_READER_EVENT_PREJOB:   prejob dump request
 * @BASE_HWCNT_READER_EVENT_POSTJOB:  postjob dump request
 * @BASE_HWCNT_READER_EVENT_COUNT:    number of supported events
 */
enum base_hwcnt_reader_event {
	BASE_HWCNT_READER_EVENT_MANUAL,
	BASE_HWCNT_READER_EVENT_PERIODIC,
	BASE_HWCNT_READER_EVENT_PREJOB,
	BASE_HWCNT_READER_EVENT_POSTJOB,
	BASE_HWCNT_READER_EVENT_COUNT
};

#define KBASE_HWCNT_READER_API_VERSION_NO_FEATURE (0)
#define KBASE_HWCNT_READER_API_VERSION_FEATURE_CYCLES_TOP (1 << 0)
#define KBASE_HWCNT_READER_API_VERSION_FEATURE_CYCLES_SHADER_CORES (1 << 1)
/**
 * struct kbase_hwcnt_reader_api_version - hwcnt reader API version
 * @version:  API version
 * @features: available features in this API version
 */
struct kbase_hwcnt_reader_api_version {
	__u32 version;
	__u32 features;
};

#endif /* _UAPI_KBASE_HWCNT_READER_H_ */

