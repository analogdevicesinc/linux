// SPDX-License-Identifier: GPL-2.0 OR MIT
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

#include "amdgpu_ualink.h"
#include "amdgpu_xgmi.h"
#include "amdgpu.h"
#include <linux/sysfs.h>
#include <linux/string.h>


/****************************************************************************
 * UALink info and configuration in sysfs
 *
 * Using kobj_attribute and not device_attribute here because the UALink
 * attributes are part of their own nested kobjects and not the amdgpu
 * device itself or a subdevice.
 ****************************************************************************/

#define UALINK_VALUE_SHOW(prefix, name, field, format)			\
static ssize_t ualink_##prefix##_##name##_show(struct kobject *kobj,	\
					       struct kobj_attribute *attr,\
					       char *buf)		\
{									\
	struct amdgpu_ualink_##prefix *info = to_ualink_##prefix(kobj);	\
									\
	return sysfs_emit(buf, format"\n", info->field);		\
}

static ssize_t show_idbits(unsigned long *bits, unsigned int nbits, char *buf)
{
	ssize_t len = 0;
	unsigned int id;

	for_each_set_bit(id, bits, nbits)
		len += sysfs_emit_at(buf, len, "%u ", id);
	if (len)
		len--;
	return len + sysfs_emit_at(buf, len, "\n");
}
#define UALINK_IDBITS_SHOW(prefix, name, field)				\
static ssize_t ualink_##prefix##_##name##_show(struct kobject *kobj,	\
					       struct kobj_attribute *attr,\
					       char *buf)		\
{									\
	struct amdgpu_ualink_##prefix *info = to_ualink_##prefix(kobj);	\
									\
	return show_idbits(info->field, sizeof(info->field)*8, buf);	\
}

static ssize_t show_idarray(const u32 *array, u32 size, char *buf)
{
	ssize_t len = 0;
	unsigned int i;

	for (i = 0; i < size; i++)
		len += sysfs_emit_at(buf, len, "%u ", array[i]);
	if (len)
		len--;
	return len + sysfs_emit_at(buf, len, "\n");
}
#define UALINK_IDARRAY_SHOW(prefix, name, field, size)			\
static ssize_t ualink_##prefix##_##name##_show(struct kobject *kobj,	\
					       struct kobj_attribute *attr,\
					       char *buf)		\
{									\
	struct amdgpu_ualink_##prefix *info = to_ualink_##prefix(kobj);	\
									\
	return show_idarray(info->field, info->size, buf);		\
}

#define UALINK_UUID_SHOW(prefix, name, field)				\
static ssize_t ualink_##prefix##_##name##_show(struct kobject *kobj,	\
					       struct kobj_attribute *attr,\
					       char *buf)		\
{									\
	struct amdgpu_ualink_##prefix *info = to_ualink_##prefix(kobj);	\
									\
	return sysfs_emit(buf, "%pU\n", &info->field);			\
}

static ssize_t show_enum(u32 x, const char * const values[], unsigned int n,
			 char *buf)
{
	return sysfs_emit(buf, "%s\n", x < n && values[x] ? values[x] : "invalid");
}
#define UALINK_ENUM_SHOW(prefix, name, field)				\
static ssize_t ualink_##prefix##_##name##_show(struct kobject *kobj,	\
					       struct kobj_attribute *attr,\
					       char *buf)		\
{									\
	struct amdgpu_ualink_##prefix *info = to_ualink_##prefix(kobj);	\
									\
	return show_enum(info->field, ualink_##name##_values,		\
			 ARRAY_SIZE(ualink_##name##_values), buf);	\
}

static const char * const ualink_link_type_values[] = {
	"UALoE", "UALink"
};
static const char * const ualink_addr_mode_values[] = {
	"source-aliasing", "source-identification"
};
static const char * const ualink_accel_state_values[] = {
	"unconfigured", "configured", "ready", "active", "error"
};

UALINK_ENUM_SHOW(info, link_type, link_type);
UALINK_VALUE_SHOW(info, accel_id,  ppod.accel_id,  "%u");
UALINK_VALUE_SHOW(info, bandwidth, ppod.bandwidth, "%u");
UALINK_VALUE_SHOW(info, latency,   ppod.latency,   "%u");
UALINK_UUID_SHOW(info, ppod_id, ppod.id);
UALINK_VALUE_SHOW(info, ppod_size, ppod.size,      "%u");
UALINK_VALUE_SHOW(info, vpod_id,   vpod.id,        "%u");
UALINK_VALUE_SHOW(info, vpod_size, vpod.size,      "%u");
UALINK_IDBITS_SHOW(info, vpod_active_accels, vpod.active_accel_bits);
UALINK_ENUM_SHOW(info, addr_mode, vpod.addr_mode);
UALINK_ENUM_SHOW(info, accel_state, accel_state);
UALINK_IDARRAY_SHOW(info, local_accels, local_accels, n_local_accels);

#define UALINK_INFO_ATTR(name) __ATTR(name, 0444, ualink_info_##name##_show, NULL)
static struct kobj_attribute ualink_info_link_type = UALINK_INFO_ATTR(link_type);
static struct kobj_attribute ualink_info_accel_id  = UALINK_INFO_ATTR(accel_id);
static struct kobj_attribute ualink_info_bandwidth = UALINK_INFO_ATTR(bandwidth);
static struct kobj_attribute ualink_info_latency   = UALINK_INFO_ATTR(latency);
static struct kobj_attribute ualink_info_ppod_id   = UALINK_INFO_ATTR(ppod_id);
static struct kobj_attribute ualink_info_ppod_size = UALINK_INFO_ATTR(ppod_size);
static struct kobj_attribute ualink_info_vpod_id   = UALINK_INFO_ATTR(vpod_id);
static struct kobj_attribute ualink_info_vpod_size = UALINK_INFO_ATTR(vpod_size);
static struct kobj_attribute ualink_info_vpod_active_accels = UALINK_INFO_ATTR(vpod_active_accels);
static struct kobj_attribute ualink_info_addr_mode = UALINK_INFO_ATTR(addr_mode);
static struct kobj_attribute ualink_info_accel_state = UALINK_INFO_ATTR(accel_state);
static struct kobj_attribute ualink_info_local_accels = UALINK_INFO_ATTR(local_accels);

static const struct attribute *ualink_info_attrs[] = {
	&ualink_info_link_type.attr,
	&ualink_info_accel_id.attr,
	&ualink_info_bandwidth.attr,
	&ualink_info_latency.attr,
	&ualink_info_ppod_id.attr,
	&ualink_info_ppod_size.attr,
	&ualink_info_vpod_id.attr,
	&ualink_info_vpod_size.attr,
	&ualink_info_vpod_active_accels.attr,
	&ualink_info_addr_mode.attr,
	&ualink_info_accel_state.attr,
	&ualink_info_local_accels.attr,
	NULL
};

static void ualink_info_release(struct kobject *kobj)
{
	struct amdgpu_ualink_info *info = to_ualink_info(kobj);

	kfree(info);
}

static const struct kobj_type ualink_info_ktype = {
	.release = ualink_info_release,
	.sysfs_ops = &kobj_sysfs_ops
};

int amdgpu_ualink_sysfs_init(struct amdgpu_device *adev)
{
	struct amdgpu_ualink_info *info = NULL;
	int r;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->ppod.accel_id = 0xffffffff;
	info->ppod.bandwidth = 0xffffffff;
	info->ppod.latency = 0xffffffff;
	info->vpod.id = 0xffffffff;
	info->vpod.addr_mode = AMDGPU_UALINK_ADDR_MODE_MAX;

	r = kobject_init_and_add(&info->kobj, &ualink_info_ktype,
				 &adev->dev->kobj, "ualink");
	if (r)
		goto err_put_info;
	r = sysfs_create_files(&info->kobj, ualink_info_attrs);
	if (r)
		goto err_del_info;

	adev->ualink.info = info;

	return r;

err_del_info:
	kobject_del(&info->kobj);
err_put_info:
	kobject_put(&info->kobj);
	return r;
}

void amdgpu_ualink_sysfs_fini(struct amdgpu_device *adev)
{
	if (adev->ualink.info) {
		sysfs_remove_files(&adev->ualink.info->kobj,
				   ualink_info_attrs);
		kobject_del(&adev->ualink.info->kobj);
		kobject_put(&adev->ualink.info->kobj);
		adev->ualink.info = NULL;
	}
}
