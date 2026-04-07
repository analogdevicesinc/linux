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

/* UALink configuration APIs are being deprecated before upstreaming and are
 * not enabled by default. They are replaced by a unified in-band/side-band
 * approach through the IFoE driver. Querying of the current configuration
 * will continue to be supported in sysfs.
 *
 * Uncomment the following line to enable the deprecated configuration APIs
 * for testing purposes only. The code to support these APIs will be removed
 * in a future patch.
 */
/* #define UALINK_ENABLE_DEPRECATED_CONFIG_SYSFS */

#define UALINK_VALUE_SHOW(prefix, name, field, format)			\
static ssize_t ualink_##prefix##_##name##_show(struct kobject *kobj,	\
					       struct kobj_attribute *attr,\
					       char *buf)		\
{									\
	struct amdgpu_ualink_##prefix *info = to_ualink_##prefix(kobj);	\
									\
	return sysfs_emit(buf, format"\n", info->field);		\
}
#ifdef UALINK_ENABLE_DEPRECATED_CONFIG_SYSFS
#define UALINK_VALUE_STORE(prefix, name, field, type, base)		\
static ssize_t ualink_##prefix##_##name##_store(struct kobject *kobj,	\
						struct kobj_attribute *attr,\
						const char *buf, size_t count)\
{									\
	struct amdgpu_ualink_##prefix *info = to_ualink_##prefix(kobj);	\
	int r;								\
									\
	r = kstrto##type(buf, base, &info->field);			\
	if (r < 0)							\
		return r;						\
	return count;							\
}
#endif

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

#ifdef UALINK_ENABLE_DEPRECATED_CONFIG_SYSFS
static ssize_t store_idarray(u32 *array, u32 *size, u32 max_id, u32 max_size,
			     const char *buf, size_t count)
{
	char *dup_buf, *str, *tok;
	unsigned int index = 0;
	ssize_t r;
	u32 id;

	*size = 0;
	if (!count)
		return 0;

	dup_buf = kstrndup(buf, count, GFP_KERNEL);
	if (unlikely(!dup_buf))
		return -ENOMEM;
	str = dup_buf;
	do {
		str += strspn(str, " ");
		tok = strsep(&str, ", ");
		r = kstrtou32(tok, 10, &id);
		if (r < 0)
			goto err;
		if (index >= max_size) {
			r = -ENOSPC;
			goto err;
		}
		if (id >= max_id) {
			r = -ERANGE;
			goto err;
		}
		array[index++] = id;
	} while (str);
	kfree(dup_buf);

	*size = index;
	return count;

err:
	kfree(dup_buf);
	return r;
}
#define UALINK_IDARRAY_STORE(prefix, name, field, size, max_id)		\
static ssize_t ualink_##prefix##_##name##_store(struct kobject *kobj,	\
						struct kobj_attribute *attr,\
						const char *buf, size_t count)\
{									\
	struct amdgpu_ualink_##prefix *info = to_ualink_##prefix(kobj);	\
	u32 new_array[ARRAY_SIZE(info->field)];				\
	u32 new_size;							\
	ssize_t r = store_idarray(new_array, &new_size, max_id,		\
				  ARRAY_SIZE(new_array), buf, count);	\
									\
	if (r >= 0) {							\
		memcpy(info->field, new_array, sizeof(new_array));	\
		info->size = new_size;					\
	}								\
	return r;							\
}
#endif

#define UALINK_UUID_SHOW(prefix, name, field)				\
static ssize_t ualink_##prefix##_##name##_show(struct kobject *kobj,	\
					       struct kobj_attribute *attr,\
					       char *buf)		\
{									\
	struct amdgpu_ualink_##prefix *info = to_ualink_##prefix(kobj);	\
									\
	return sysfs_emit(buf, "%pU\n", &info->field);			\
}
#ifdef UALINK_ENABLE_DEPRECATED_CONFIG_SYSFS
#define UALINK_UUID_STORE(prefix, name, field)				\
static ssize_t ualink_##prefix##_##name##_store(struct kobject *kobj,	\
						struct kobj_attribute *attr,\
						const char *buf, size_t count)\
{									\
	struct amdgpu_ualink_##prefix *info = to_ualink_##prefix(kobj);	\
	int r;								\
									\
	r = uuid_parse(buf, &info->field);				\
	return r ? r : count;						\
}
#endif

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

#ifdef UALINK_ENABLE_DEPRECATED_CONFIG_SYSFS
UALINK_VALUE_SHOW(ppod_setup, accel_id,  ppod.accel_id,  "%u");
UALINK_VALUE_SHOW(ppod_setup, bandwidth, ppod.bandwidth, "%u");
UALINK_VALUE_SHOW(ppod_setup, latency,   ppod.latency,   "%u");
UALINK_UUID_SHOW(ppod_setup, ppod_id, ppod.id);
UALINK_VALUE_SHOW(ppod_setup, ppod_size, ppod.size,      "%u");
UALINK_IDARRAY_SHOW(ppod_setup, local_accels, local_accels, n_local_accels);

UALINK_VALUE_STORE(ppod_setup, accel_id,  ppod.accel_id,  u32, 10);
UALINK_VALUE_STORE(ppod_setup, bandwidth, ppod.bandwidth, u32, 10);
UALINK_VALUE_STORE(ppod_setup, latency,   ppod.latency,   u32, 10);
UALINK_UUID_STORE(ppod_setup, ppod_id,   ppod.id);
UALINK_VALUE_STORE(ppod_setup, ppod_size, ppod.size,      u32, 10);
UALINK_IDARRAY_STORE(ppod_setup, local_accels, local_accels, n_local_accels,
		     AMDGPU_UALINK_ACCEL_MAX);

static enum amdgpu_ualink_accel_state
check_ppod_state(struct amdgpu_device *adev,
		 const struct amdgpu_ualink_ppod_setup *setup)
{
	if (setup->ppod.size <= 0 || setup->ppod.size > AMDGPU_UALINK_ACCEL_MAX) {
		dev_dbg(adev->dev, "pPod size %u out of range [1..%u]\n",
			setup->ppod.size, AMDGPU_UALINK_ACCEL_MAX);
		return AMDGPU_UALINK_ACCEL_STATE_UNCONFIGURED;
	}
	if (setup->ppod.accel_id >= setup->ppod.size) {
		dev_dbg(adev->dev, "Accelerator ID %u greater or equal pPod size %u\n",
			setup->ppod.accel_id, setup->ppod.size);
		return AMDGPU_UALINK_ACCEL_STATE_UNCONFIGURED;
	}
	return AMDGPU_UALINK_ACCEL_STATE_CONFIGURED;
}

static ssize_t ualink_ppod_setup_commit_store(struct kobject *kobj,
					      struct kobj_attribute *attr,
					      const char *buf, size_t count)
{
	struct amdgpu_ualink_ppod_setup *setup = to_ualink_ppod_setup(kobj);
	struct amdgpu_ualink_info *info = to_ualink_info(kobj->parent);
	struct device *dev = kobj_to_dev(info->kobj.parent);
	struct drm_device *ddev = dev_get_drvdata(dev);
	struct amdgpu_device *adev = drm_to_adev(ddev);

	if (!sysfs_streq(buf, "true"))
		return -EINVAL;

	/* TODO: Send configuration to ASP */

	info->ppod = setup->ppod;
	info->accel_state = check_ppod_state(adev, setup);

	/* TODO: If accel_state was ACTIVE, reset all connections */

	return count;
}

#define UALINK_PPOD_SETUP_ATTR(name) __ATTR(name, 0600,			\
		ualink_ppod_setup_##name##_show,			\
		ualink_ppod_setup_##name##_store)
static struct kobj_attribute ualink_ppod_setup_accel_id  = UALINK_PPOD_SETUP_ATTR(accel_id);
static struct kobj_attribute ualink_ppod_setup_bandwidth = UALINK_PPOD_SETUP_ATTR(bandwidth);
static struct kobj_attribute ualink_ppod_setup_latency   = UALINK_PPOD_SETUP_ATTR(latency);
static struct kobj_attribute ualink_ppod_setup_ppod_id   = UALINK_PPOD_SETUP_ATTR(ppod_id);
static struct kobj_attribute ualink_ppod_setup_ppod_size = UALINK_PPOD_SETUP_ATTR(ppod_size);
static struct kobj_attribute ualink_ppod_setup_local_accels = UALINK_PPOD_SETUP_ATTR(local_accels);
static struct kobj_attribute ualink_ppod_setup_commit = __ATTR(commit, 0200, NULL,
							       ualink_ppod_setup_commit_store);

static const struct attribute *ualink_ppod_setup_attrs[] = {
	&ualink_ppod_setup_accel_id.attr,
	&ualink_ppod_setup_bandwidth.attr,
	&ualink_ppod_setup_latency.attr,
	&ualink_ppod_setup_ppod_id.attr,
	&ualink_ppod_setup_ppod_size.attr,
	&ualink_ppod_setup_local_accels.attr,
	&ualink_ppod_setup_commit.attr,
	NULL
};
#endif

static void ualink_ppod_setup_release(struct kobject *kobj)
{
	struct amdgpu_ualink_ppod_setup *setup = to_ualink_ppod_setup(kobj);

	kfree(setup);
}

static const struct kobj_type ualink_ppod_setup_ktype = {
	.release = ualink_ppod_setup_release,
	.sysfs_ops = &kobj_sysfs_ops
};

int amdgpu_ualink_sysfs_init(struct amdgpu_device *adev)
{
	struct amdgpu_ualink_ppod_setup *ppod_setup = NULL;
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

	ppod_setup = kzalloc(sizeof(*ppod_setup), GFP_KERNEL);
	if (!ppod_setup) {
		r = -ENOMEM;
		goto err_remove_info_files;
	}
	r = kobject_init_and_add(&ppod_setup->kobj, &ualink_ppod_setup_ktype,
				 &info->kobj, "setup");
	if (r)
		goto err_put_ppod_setup;
#ifdef UALINK_ENABLE_DEPRECATED_CONFIG_SYSFS
	r = sysfs_create_files(&ppod_setup->kobj, ualink_ppod_setup_attrs);
	if (r)
		goto err_del_ppod_setup;
#endif

	adev->ualink.setup = ppod_setup;
	adev->ualink.info = info;

	return r;

#ifdef UALINK_ENABLE_DEPRECATED_CONFIG_SYSFS
err_del_ppod_setup:
	kobject_del(&ppod_setup->kobj);
#endif
err_put_ppod_setup:
	kobject_put(&ppod_setup->kobj);
err_remove_info_files:
	sysfs_remove_files(&info->kobj, ualink_info_attrs);
err_del_info:
	kobject_del(&info->kobj);
err_put_info:
	kobject_put(&info->kobj);
	return r;
}

void amdgpu_ualink_sysfs_fini(struct amdgpu_device *adev)
{
	if (adev->ualink.setup) {
#ifdef UALINK_ENABLE_DEPRECATED_CONFIG_SYSFS
		sysfs_remove_files(&adev->ualink.setup->kobj,
				   ualink_ppod_setup_attrs);
#endif
		kobject_del(&adev->ualink.setup->kobj);
		kobject_put(&adev->ualink.setup->kobj);
		adev->ualink.setup = NULL;
	}
	if (adev->ualink.info) {
		sysfs_remove_files(&adev->ualink.info->kobj,
				   ualink_info_attrs);
		kobject_del(&adev->ualink.info->kobj);
		kobject_put(&adev->ualink.info->kobj);
		adev->ualink.info = NULL;
	}
}
