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

struct amdgpu_ualink_mgr {
	struct amdgpu_ualink_info *info;
	struct amdgpu_ualink_ppod_setup *setup;
};

int amdgpu_ualink_sysfs_init(struct amdgpu_device *adev);
void amdgpu_ualink_sysfs_fini(struct amdgpu_device *adev);

#endif
