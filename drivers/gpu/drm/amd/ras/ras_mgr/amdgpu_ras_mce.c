// SPDX-License-Identifier: MIT
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

#include "amdgpu.h"
#include "amdgpu_ras_mgr.h"
#include "amdgpu_ras_mce.h"
#include <asm/mce.h>

static int amdgpu_sys_mce_notifier(struct notifier_block *nb, unsigned long val, void *data);
typedef int (*mce_notifier)(struct amdgpu_device *dev,
			unsigned int id, unsigned long val, void *data);
struct ras_mce_dev {
	void *dev;
	mce_notifier dev_notifier;
};

struct ras_mce_mgr {
	struct ras_mce_dev devs[MAX_GPU_INSTANCE];
	atomic_t dev_count;
	atomic_t ref_count;
	struct notifier_block nb;
	mce_notifier notifier;
};

struct ras_mce_mgr  g_ras_mce_mgr = {
	.nb = {
		.notifier_call = amdgpu_sys_mce_notifier,
		.priority = MCE_PRIO_UC,
	},
};

static int amdgpu_sys_mce_notifier(struct notifier_block *nb, unsigned long val, void *data)
{
	struct ras_mce_mgr *mce_mgr = container_of(nb, struct ras_mce_mgr, nb);
	struct ras_mce_dev *mce_dev;
	int i;

	if (!data)
		return NOTIFY_DONE;

	for (i = 0; i < atomic_read(&mce_mgr->dev_count); i++) {
		mce_dev = &mce_mgr->devs[i];
		if (mce_dev->dev && mce_dev->dev_notifier)
			mce_dev->dev_notifier(mce_dev->dev, i, val, data);
	}

	return NOTIFY_DONE;
}

static int amdgpu_ras_register_mce_notifier(struct amdgpu_device *adev, mce_notifier notifier)
{
	struct ras_mce_mgr *mce_mgr = &g_ras_mce_mgr;
	struct ras_mce_dev *mce_dev = NULL;
	int idx, i;

	if (!adev || !notifier)
		return -EINVAL;

	if (atomic_read(&mce_mgr->dev_count) >= ARRAY_SIZE(mce_mgr->devs))
		return -ENOENT;

	for (i = 0; i < atomic_read(&mce_mgr->dev_count); i++) {
		if (mce_mgr->devs[i].dev == adev)
			return -EEXIST;
	}

	idx = atomic_fetch_inc(&mce_mgr->dev_count);
	mce_dev = &mce_mgr->devs[idx];
	mce_dev->dev = adev;
	mce_dev->dev_notifier = notifier;
	atomic_inc(&mce_mgr->ref_count);

	if (!idx)
		mce_register_decode_chain(&mce_mgr->nb);

	return 0;
}

static int amdgpu_ras_unregister_mce_notifier(struct amdgpu_device *adev)
{
	struct ras_mce_mgr *mce_mgr = &g_ras_mce_mgr;
	struct ras_mce_dev *mce_dev;
	int i;

	if (!adev)
		return -EINVAL;

	for (i = 0; i < atomic_read(&mce_mgr->dev_count); i++) {
		mce_dev = &mce_mgr->devs[i];
		if (mce_dev->dev == adev) {
			mce_dev->dev = NULL;
			mce_dev->dev_notifier = NULL;
			atomic_dec(&mce_mgr->ref_count);
			break;
		}
	}

	if (atomic_read(&mce_mgr->ref_count) <= 0) {
		atomic_set(&mce_mgr->dev_count, 0);
		mce_unregister_decode_chain(&mce_mgr->nb);
	}

	return 0;
}

static int amdgpu_ras_mce_notifier(struct amdgpu_device *adev,
			unsigned int id, unsigned long val, void *data)
{
	return 0;
}

int amdgpu_ras_mce_hw_init(struct amdgpu_device *adev)
{
	return 0;
}

int amdgpu_ras_mce_hw_fini(struct amdgpu_device *adev)
{
	return 0;
}

int amdgpu_ras_mce_sw_init(struct amdgpu_device *adev)
{
	amdgpu_ras_register_mce_notifier(adev, amdgpu_ras_mce_notifier);
	return 0;
}

int amdgpu_ras_mce_sw_fini(struct amdgpu_device *adev)
{
	amdgpu_ras_unregister_mce_notifier(adev);
	return 0;
}
