/* SPDX-License-Identifier: GPL-2.0 OR MIT */
/*
 * Copyright 2026 Advanced Micro Devices, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#ifndef AMDGPU_UALINK_H
#define AMDGPU_UALINK_H

#include <linux/uuid.h>
#include <linux/bitmap.h>
#include <linux/kobject.h>

struct amdgpu_device;

#define AMDGPU_UALINK_ACCEL_MAX 256
#define AMDGPU_UALINK_LOCAL_ACCELS_MAX 8
#define AMDGPU_UALINK_STATIONS_MAX 64

enum amdgpu_ualink_conn_state {
	AMDGPU_UALINK_CONN_NOT_READY			= 0,
	AMDGPU_UALINK_CONN_IN_PROGRESS			= 1,
	AMDGPU_UALINK_CONN_ESTABLISHED			= 2
};

enum amdgpu_ualink_type {
	AMDGPU_UALINK_TYPE_UALOE = 0,
	AMDGPU_UALINK_TYPE_UALINK = 1,
	AMDGPU_UALINK_TYPE_MAX
};

enum amdgpu_ualink_accel_state {
	AMDGPU_UALINK_ACCEL_STATE_UNCONFIGURED = 0,
	AMDGPU_UALINK_ACCEL_STATE_CONFIGURED,
	AMDGPU_UALINK_ACCEL_STATE_READY,
	AMDGPU_UALINK_ACCEL_STATE_ACTIVE,
	AMDGPU_UALINK_ACCEL_STATE_ERROR,
	AMDGPU_UALINK_ACCEL_STATE_MAX
};

enum amdgpu_ualink_addr_mode {
	AMDGPU_UALINK_ADDR_MODE_SOURCE_ALIAS = 0,
	AMDGPU_UALINK_ADDR_MODE_SOURCE_IDENT = 1,
	AMDGPU_UALINK_ADDR_MODE_MAX
};

/* Physical pod info shared between query and setup API */
struct amdgpu_ualink_ppod_info {
	u32 accel_id;
	u32 bandwidth;
	u32 latency;
	u32 size;
	uuid_t id;
};

/* Virtual pod info shared between query and config API */
struct amdgpu_ualink_vpod_info {
	u32 id;
	u32 size;
	u32 addr_mode;
	DECLARE_BITMAP(active_accel_bits, AMDGPU_UALINK_ACCEL_MAX);
};

/* Query current UALink physical and virtual pod info */
struct amdgpu_ualink_info {
	struct kobject kobj;
	enum amdgpu_ualink_type link_type;
	enum amdgpu_ualink_accel_state accel_state;
	struct amdgpu_ualink_ppod_info ppod;
	struct amdgpu_ualink_vpod_info vpod;
	/* Local accelerator array in the vpod derived from
	 * vpod.active_accel_bits, in arbitrary order
	 */
	u32 n_local_accels;
	u32 local_accels[AMDGPU_UALINK_LOCAL_ACCELS_MAX];
};
#define to_ualink_info(ko) container_of(ko, struct amdgpu_ualink_info, kobj)

/* UALink physical pod setup */
struct amdgpu_ualink_ppod_setup {
	struct kobject kobj;
	struct amdgpu_ualink_ppod_info ppod;
	/* Local accelerator array indexed by socket ID */
	u32 n_local_accels;
	u32 local_accels[AMDGPU_UALINK_LOCAL_ACCELS_MAX];
};
#define to_ualink_ppod_setup(ko) container_of(ko, struct amdgpu_ualink_ppod_setup, kobj)

/* UAlink virtual pod config */
struct amdgpu_ualink_vpod_config {
	struct kobject kobj;
	struct amdgpu_ualink_vpod_info vpod;
};
#define to_ualink_vpod_config(ko) container_of(ko, struct amdgpu_ualink_vpod_config, kobj)

/* UALink station configuration */
struct amdgpu_ualink_station_config {
	struct kobject kobj;
	/* Station configuration flags
	 * bits [3:0]: PortPerStation (PPS) - 1, 2, or 4
	 * bits [7:4]: Reserved
	 */
	u8 flags;
	u8 n_stations;
	/* bitmap or enabled lanes for each station in logical station order */
	u8 lane_en_bitmap[AMDGPU_UALINK_STATIONS_MAX];
};
#define to_ualink_station_config(ko) container_of(ko, struct amdgpu_ualink_station_config, kobj)

struct amdgpu_ualink_connection {
	struct completion hello_done;
	struct mutex lock;
	u32 generation_count;
	enum amdgpu_ualink_conn_state state;
};

struct amdgpu_ualink_mgr {
	u32 psp_if_ver;
	struct amdgpu_ualink_info *info;
	struct amdgpu_ualink_ppod_setup *setup;
	struct amdgpu_ualink_vpod_config *config;
	struct amdgpu_ualink_station_config *stations;

	/* Xarray to store info about exported BOs */
	struct xarray	exp_xa;

	/* Xarray to store info about imported ualink handles */
	struct xarray	imp_xa;

	/* Xarray to store handles to be deleted */
	struct xarray	handle_invalid_xa;

	/* Array to store connection state per remote GPU */
	struct amdgpu_ualink_connection conn_state[AMDGPU_UALINK_ACCEL_MAX];

	/* List of exported ualink handles per GPU. Used to quickly traverse
	 * the list of all handles exported to a remote GPU.
	 */
	struct list_head exp_handles_list[AMDGPU_UALINK_ACCEL_MAX];

	/* List of imported ualink handles per GPU. Used to quickly traverse
	 * the list of all handles imported from a remote GPU.
	 */
	struct list_head imp_handles_list[AMDGPU_UALINK_ACCEL_MAX];

	/* WQ to manage revocation of exported memory. */
	struct workqueue_struct *npa_wq;
};

int amdgpu_ualink_sysfs_init(struct amdgpu_device *adev);
void amdgpu_ualink_sysfs_fini(struct amdgpu_device *adev);
int amdgpu_ualink_init(struct amdgpu_device *adev);
void amdgpu_ualink_fini(struct amdgpu_device *adev);
int amdgpu_ualink_manager_start(struct amdgpu_device *adev);
void amdgpu_ualink_manager_stop(struct amdgpu_device *adev);
#endif
