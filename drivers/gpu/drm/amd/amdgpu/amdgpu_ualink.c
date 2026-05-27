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

#include <linux/xarray.h>
#include <drm/drm_mm.h>
#include "amdgpu.h"
#include "amdgpu_ualink.h"
#include "amdgpu_xgmi.h"
#include <linux/sysfs.h>
#include <linux/string.h>

int amdgpu_ualink_init(struct amdgpu_device *adev)
{
	int r;

	/* UALink relies on PSP services. If the PSP IP block is not present
	 * just skip UALink initialization.
	 */
	if (!amdgpu_device_ip_get_ip_block(adev, AMD_IP_BLOCK_TYPE_PSP)) {
		adev->ualink.psp_if_ver = 0xffffffff;
		return 0;
	}

	r = psp_ual_get_interface_version(&adev->psp, &adev->ualink.psp_if_ver);
	if (r) {
		adev->ualink.psp_if_ver = 0xffffffff;
		dev_err(adev->dev,
			"UALink interface version detection failed: %d", r);
		return r;
	}
	dev_info(adev->dev, "Found UALink interface version 0x%x\n",
		 adev->ualink.psp_if_ver);

	/* Query initial configuration from ASP */
	r = psp_ual_query_info(&adev->psp, adev->ualink.psp_if_ver,
			       adev->ualink.info);
	if (r) {
		dev_err(adev->dev,
			"Failed to query initial UALink config: %d\n", r);
		return r;
	}

	return 0;
}

void amdgpu_ualink_fini(struct amdgpu_device *adev)
{
	/* empty */
}

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

#ifdef UALINK_ENABLE_DEPRECATED_CONFIG_SYSFS
static ssize_t store_idbits(unsigned long *bits, unsigned int nbits,
			    const char *buf, size_t count)
{
	char *dup_buf, *str, *tok;
	ssize_t r;
	u32 id;

	bitmap_zero(bits, nbits);
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
		if (id >= nbits) {
			r = -ERANGE;
			goto err;
		}
		set_bit(id, bits);
	} while (str);
	kfree(dup_buf);

	return count;

err:
	kfree(dup_buf);
	return r;
}
#define UALINK_IDBITS_STORE(prefix, name, field, nbits)			\
static ssize_t ualink_##prefix##_##name##_store(struct kobject *kobj,	\
						struct kobj_attribute *attr,\
						const char *buf, size_t count)\
{									\
	struct amdgpu_ualink_##prefix *info = to_ualink_##prefix(kobj);	\
	DECLARE_BITMAP(bits, nbits);					\
	ssize_t r = store_idbits(bits, nbits, buf, count);		\
									\
	if (r >= 0)							\
		bitmap_copy(info->field, bits, nbits);			\
	return r;							\
}
#endif

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

#ifdef UALINK_ENABLE_DEPRECATED_CONFIG_SYSFS
static ssize_t store_enum(u32 *x, const char * const values[], unsigned int n,
			  const char *buf, size_t count)
{
	unsigned int i;

	for (i = 0; i < n; i++) {
		if (values[i] && sysfs_streq(buf, values[i])) {
			*x = i;
			return count;
		}
	}
	return -EINVAL;
}
#define UALINK_ENUM_STORE(prefix, name, field)				\
static ssize_t ualink_##prefix##_##name##_store(struct kobject *kobj,	\
						struct kobj_attribute *attr,\
						const char *buf, size_t count)\
{									\
	struct amdgpu_ualink_##prefix *info = to_ualink_##prefix(kobj);	\
									\
	return store_enum(&info->field, ualink_##name##_values,		\
			 ARRAY_SIZE(ualink_##name##_values),		\
			 buf, count);					\
}
#endif
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
static void deactivate_accelerator(struct amdgpu_device *adev);

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
	int r;

	if (!sysfs_streq(buf, "true"))
		return -EINVAL;

	r = psp_ual_set_ppod_config(&adev->psp, adev->ualink.psp_if_ver,
				    setup);
	if (r)
		return r;
	r = psp_ual_query_info(&adev->psp, adev->ualink.psp_if_ver, info);
	if (r)
		return r;

	/*
	 * Hold mgpu_info.mutex to serialize with activate_local_vpod() in
	 * vpod_config_commit_store which also holds this lock. Without it,
	 * deactivate_accelerator() can race with activate_accelerator(),
	 * causing concurrent vm_fini / vm_init on the same NPA VM.
	 */
	mutex_lock(&mgpu_info.mutex);
	if (info->accel_state >= AMDGPU_UALINK_ACCEL_STATE_READY)
		deactivate_accelerator(adev);

	info->accel_state = check_ppod_state(adev, setup);
	mutex_unlock(&mgpu_info.mutex);

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

#ifdef UALINK_ENABLE_DEPRECATED_CONFIG_SYSFS
UALINK_VALUE_SHOW(vpod_config, vpod_id,   vpod.id,   "%u");
UALINK_VALUE_SHOW(vpod_config, vpod_size, vpod.size, "%u");
UALINK_IDBITS_SHOW(vpod_config, vpod_active_accels, vpod.active_accel_bits);
UALINK_ENUM_SHOW(vpod_config, addr_mode, vpod.addr_mode);

UALINK_VALUE_STORE(vpod_config, vpod_id,   vpod.id,   u32, 10);
UALINK_VALUE_STORE(vpod_config, vpod_size, vpod.size, u32, 10);
UALINK_IDBITS_STORE(vpod_config, vpod_active_accels, vpod.active_accel_bits,
		    AMDGPU_UALINK_ACCEL_MAX);
UALINK_ENUM_STORE(vpod_config, addr_mode, vpod.addr_mode);

static bool check_vpod_info(struct amdgpu_device *adev,
			    const struct amdgpu_ualink_info *info)
{
	unsigned int weight;

	if (info->accel_state < AMDGPU_UALINK_ACCEL_STATE_CONFIGURED) {
		dev_dbg(adev->dev, "pPod is not yet configured\n");
		return false;
	}
	if (info->vpod.id >= AMDGPU_UALINK_ACCEL_MAX) {
		dev_dbg(adev->dev, "vPod ID %u out of range [0..%u]\n",
			info->vpod.id, AMDGPU_UALINK_ACCEL_MAX - 1);
		return false;
	}
	if (info->vpod.size == 0 || info->vpod.size > info->ppod.size) {
		dev_dbg(adev->dev, "vPod size %u out of range [1..%u]\n",
			info->vpod.size, info->ppod.size);
		return false;
	}
	if (info->vpod.addr_mode >= AMDGPU_UALINK_ADDR_MODE_MAX) {
		dev_dbg(adev->dev, "Invalid addr mode %u\n", info->vpod.id);
		return false;
	}
	weight = bitmap_weight(info->vpod.active_accel_bits, AMDGPU_UALINK_ACCEL_MAX);
	if (weight != info->vpod.size) {
		dev_dbg(adev->dev, "vPod size doesn't match vpod_active_accels list: %u != %u\n",
			info->vpod.size, weight);
		return false;
	}
	if (!test_bit(info->ppod.accel_id, info->vpod.active_accel_bits)) {
		dev_dbg(adev->dev, "Accelerator ID %u not listed in vpod_active_accels\n",
			info->ppod.accel_id);
		return false;
	}

	return true;
}

static struct amdgpu_device *find_peer_adev(unsigned int accel_id)
{
	unsigned int i;

	for (i = 0; i < mgpu_info.num_gpu; i++) {
		struct amdgpu_device *peer_adev = mgpu_info.gpu_ins[i].adev;

		if (peer_adev->ualink.info &&
		    peer_adev->ualink.info->ppod.accel_id == accel_id)
			return peer_adev;
	}

	return NULL;
}

static bool check_local_vpod_integrity(struct amdgpu_device *adev)
{
	struct amdgpu_ualink_info *info = adev->ualink.info;
	struct amdgpu_ualink_info *peer_info;
	struct amdgpu_device *peer_adev;
	unsigned int accel_id;
	unsigned int i;

	if (!check_vpod_info(adev, info))
		return false;

	/* Check that all local accelerators listed in vpod_active_accels have
	 * matching pod IDs
	 */
	for_each_set_bit(accel_id, info->vpod.active_accel_bits, AMDGPU_UALINK_ACCEL_MAX) {

		if (accel_id == info->ppod.accel_id)
			continue;

		peer_adev = find_peer_adev(accel_id);
		if (!peer_adev)
			continue;
		peer_info = peer_adev->ualink.info;

		if (peer_info->vpod.id != info->vpod.id) {
			dev_dbg(adev->dev, "Peer %u vpod_id doesn't match: %u != %u",
				accel_id, peer_info->vpod.id, info->vpod.id);
			return false;
		}
		if (!uuid_equal(&peer_info->ppod.id, &info->ppod.id)) {
			dev_dbg(adev->dev, "Peer %u ppod_id doesn't match: %pU != %pU",
				accel_id, &peer_info->ppod.id, &info->ppod.id);
			return false;
		}
	}

	/* Derive local accels from pod IDs of GPUs in mgpu_info */
	info->n_local_accels = 0;
	for (i = 0; i < mgpu_info.num_gpu &&
		    info->n_local_accels < AMDGPU_UALINK_LOCAL_ACCELS_MAX;
	     i++) {
		peer_adev = mgpu_info.gpu_ins[i].adev;
		peer_info = peer_adev->ualink.info;

		if (peer_adev == adev ||
		    (peer_info && peer_info->vpod.id == info->vpod.id &&
		     uuid_equal(&peer_info->ppod.id, &info->ppod.id)))
			info->local_accels[info->n_local_accels++] =
				peer_info->ppod.accel_id;
	}

	/* Then check consistency of the vpod information on all those GPUs */
	for (i = 0; i < info->n_local_accels; i++) {
		unsigned int j;

		for (j = i + 1; j < info->n_local_accels; j++) {
			if (info->local_accels[j] == accel_id) {
				dev_dbg(adev->dev,
					"Accelerator ID %u is not unique among local GPUs\n",
					accel_id);
				return false;
			}
		}

		accel_id = info->local_accels[i];

		/* Skip this GPU, we are looking for our peers */
		if (accel_id == info->ppod.accel_id)
			continue;

		peer_adev = find_peer_adev(accel_id);
		if (WARN_ON(!peer_adev || !peer_adev->ualink.info))
			/* info->local_accels we just built is corrupted? */
			return false;
		peer_info = peer_adev->ualink.info;

		/* Check peer vpod info and consistency */
		if (!check_vpod_info(peer_adev, peer_info))
			return false;

		if (peer_info->ppod.size != info->ppod.size) {
			dev_dbg(adev->dev, "Peer %u ppod_size doesn't match: %u != %u\n",
				accel_id, peer_info->ppod.size, info->ppod.size);
			return false;
		}
		if (peer_info->vpod.size != info->vpod.size) {
			dev_dbg(adev->dev, "Peer %u vpod_size doesn't match: %u != %u\n",
				accel_id, peer_info->vpod.size, info->vpod.size);
			return false;
		}
		if (peer_info->vpod.addr_mode != info->vpod.addr_mode) {
			dev_dbg(adev->dev, "Peer %u addr_mode doesn't match: %u != %u\n",
				accel_id, peer_info->vpod.addr_mode, info->vpod.addr_mode);
			return false;
		}
		if (!bitmap_equal(peer_info->vpod.active_accel_bits,
				  info->vpod.active_accel_bits, AMDGPU_UALINK_ACCEL_MAX)) {
			dev_dbg(adev->dev, "Peer %u vpod_active_accels don't match\n",
				accel_id);
			return false;
		}

		/* Update peer's local accelerator array */
		peer_info->n_local_accels = info->n_local_accels;
		memcpy(peer_info->local_accels, info->local_accels,
		       sizeof(info->local_accels));
	}
	return true;
}

static void activate_accelerator(struct amdgpu_device *adev)
{
	int r;

	if (adev->ualink.info->accel_state >= AMDGPU_UALINK_ACCEL_STATE_READY)
		return;

	/* Enable incoming NPA address translation with NPA VMID */
	r = psp_ual_set_npa_config(&adev->psp, adev->ualink.psp_if_ver,
				   adev->vm_manager.npa_vmid, true);
	if (r) {
		dev_err(adev->dev, "Failed to set NPA config\n");
		return;
	}

	r = amdgpu_ualink_manager_start(adev);
	if (r) {
		dev_err(adev->dev, "Failed to start UALink manager\n");
		return;
	}

	adev->ualink.info->accel_state = AMDGPU_UALINK_ACCEL_STATE_READY;
}

static void deactivate_accelerator(struct amdgpu_device *adev)
{
	if (adev->ualink.info->accel_state < AMDGPU_UALINK_ACCEL_STATE_READY)
		return;

	/* Disable incoming NPA address translation with NPA VMID */
	psp_ual_set_npa_config(&adev->psp, adev->ualink.psp_if_ver,
			       adev->vm_manager.npa_vmid, false);
	/* ignore return value */
	adev->ualink.info->accel_state = AMDGPU_UALINK_ACCEL_STATE_CONFIGURED;

	amdgpu_ualink_manager_stop(adev);
}

static void activate_local_vpod(struct amdgpu_device *adev)
{
	struct amdgpu_ualink_info *info = adev->ualink.info;
	struct amdgpu_device *peer_adev;
	unsigned int accel_id;
	unsigned int i;

	for (i = 0; i < info->n_local_accels; i++) {
		accel_id = info->local_accels[i];

		peer_adev = find_peer_adev(accel_id);
		if (WARN_ON(!peer_adev || !peer_adev->ualink.info))
			/* info->local_accels is corrupted? */
			continue;

		activate_accelerator(peer_adev);
	}
}

static ssize_t ualink_vpod_config_commit_store(struct kobject *kobj,
					       struct kobj_attribute *attr,
					       const char *buf, size_t count)
{
	struct amdgpu_ualink_vpod_config *config = to_ualink_vpod_config(kobj);
	struct amdgpu_ualink_info *info = to_ualink_info(kobj->parent);
	struct device *dev = kobj_to_dev(info->kobj.parent);
	struct drm_device *ddev = dev_get_drvdata(dev);
	struct amdgpu_device *adev = drm_to_adev(ddev);
	int r;

	if (!sysfs_streq(buf, "true"))
		return -EINVAL;
	if (info->accel_state < AMDGPU_UALINK_ACCEL_STATE_CONFIGURED) {
		dev_dbg(adev->dev, "Ualink ppod is not yet configured\n");
		return -EINVAL;
	}

	r = psp_ual_set_vpod_config(&adev->psp, adev->ualink.psp_if_ver,
				    config);
	if (r)
		return r;
	r = psp_ual_query_info(&adev->psp, adev->ualink.psp_if_ver, info);
	if (r)
		return r;

	/* The integrity check makes sure each new GPU is consistent with the
	 * other GPUs already in the vPod. All known local GPUs can become
	 * "ready" at the same time.
	 *
	 * Misconfiguration of one GPU does not reduce the state of other GPUs
	 * already in the vPod.
	 */
	mutex_lock(&mgpu_info.mutex);
	if (check_local_vpod_integrity(adev))
		activate_local_vpod(adev);
	else if (info->accel_state >= AMDGPU_UALINK_ACCEL_STATE_CONFIGURED)
		deactivate_accelerator(adev);
	mutex_unlock(&mgpu_info.mutex);

	/* TODO: Update KFD topology for in-domain link */

	/* TODO: If state was ACTIVE:
	 * - If addr_mode changed, reset all connections, reset state to READY
	 * - If accelerators were removed, reset those links, but keep state ACTIVE
	 * - If accelerators were added, keep state ACTIVE
	 */

	return count;
}

#define UALINK_VPOD_CONFIG_ATTR(name) __ATTR(name, 0600,		\
		ualink_vpod_config_##name##_show,			\
		ualink_vpod_config_##name##_store)
static struct kobj_attribute ualink_vpod_config_vpod_id   = UALINK_VPOD_CONFIG_ATTR(vpod_id);
static struct kobj_attribute ualink_vpod_config_vpod_size = UALINK_VPOD_CONFIG_ATTR(vpod_size);
static struct kobj_attribute ualink_vpod_config_vpod_active_accels =
						UALINK_VPOD_CONFIG_ATTR(vpod_active_accels);
static struct kobj_attribute ualink_vpod_config_addr_mode = UALINK_VPOD_CONFIG_ATTR(addr_mode);
static struct kobj_attribute ualink_vpod_config_commit = __ATTR(commit, 0200, NULL,
								ualink_vpod_config_commit_store);

static const struct attribute *ualink_vpod_config_attrs[] = {
	&ualink_vpod_config_vpod_id.attr,
	&ualink_vpod_config_vpod_size.attr,
	&ualink_vpod_config_vpod_active_accels.attr,
	&ualink_vpod_config_addr_mode.attr,
	&ualink_vpod_config_commit.attr,
	NULL
};
#endif

static void ualink_vpod_config_release(struct kobject *kobj)
{
	struct amdgpu_ualink_vpod_config *config = to_ualink_vpod_config(kobj);

	kfree(config);
}

static const struct kobj_type ualink_vpod_config_ktype = {
	.release = ualink_vpod_config_release,
	.sysfs_ops = &kobj_sysfs_ops
};

#ifdef UALINK_ENABLE_DEPRECATED_CONFIG_SYSFS
UALINK_VALUE_SHOW(station_config, flags, flags, "0x%x");

UALINK_VALUE_STORE(station_config, flags, flags, u8, 16);

static size_t lane_bitmap_to_string(char *str,
		const struct amdgpu_ualink_station_config *stations)
{
	size_t i;

	for (i = 0; i < stations->n_stations; i++)
		str[i] = stations->lane_en_bitmap[i] +
			(stations->lane_en_bitmap[i] < 10 ? '0' : 'a' - 10);
	str[i] = '\0';

	return i;
}
static ssize_t
ualink_station_config_lane_en_bitmap_show(struct kobject *kobj,
					  struct kobj_attribute *attr,
					  char *buf)
{
	struct amdgpu_ualink_station_config *stations = to_ualink_station_config(kobj);
	char str[AMDGPU_UALINK_STATIONS_MAX + 1];

	lane_bitmap_to_string(str, stations);
	return sysfs_emit(buf, "%s\n", str);
}
static ssize_t
ualink_station_config_lane_en_bitmap_store(struct kobject *kobj,
					   struct kobj_attribute *attr,
					   const char *buf, size_t count)
{
	struct amdgpu_ualink_station_config *stations = to_ualink_station_config(kobj);
	u8 lane_en_bitmap[AMDGPU_UALINK_STATIONS_MAX];
	u32 n_stations = 0;
	bool end = false;
	size_t i;

	for (i = 0; i < count; i++) {
		/* Accept any number of \n in the end */
		if (buf[i] == '\n') {
			end = true;
			continue;
		}
		/* Don't accept any more data after \n */
		if (end)
			return -EINVAL;
		/* Don't accept more data than the array size */
		if (i >= sizeof(lane_en_bitmap))
			return -ENOSPC;
		/* Accept hex digits */
		if (buf[i] >= '0' && buf[i] <= '9')
			lane_en_bitmap[n_stations++] = buf[i] - '0';
		else if (buf[i] >= 'a' && buf[i] <= 'f')
			lane_en_bitmap[n_stations++] = buf[i] + 10 - 'a';
		else if (buf[i] >= 'A' && buf[i] <= 'F')
			lane_en_bitmap[n_stations++] = buf[i] + 10 - 'A';
		else
			return -EINVAL;
	}

	memcpy(stations->lane_en_bitmap, lane_en_bitmap, sizeof(lane_en_bitmap));
	stations->n_stations = n_stations;
	return i;
}

static ssize_t ualink_station_config_commit_store(struct kobject *kobj,
						  struct kobj_attribute *attr,
						  const char *buf, size_t count)
{
	struct amdgpu_ualink_station_config *stations = to_ualink_station_config(kobj);
	struct amdgpu_ualink_info *info = to_ualink_info(kobj->parent);
	struct device *dev = kobj_to_dev(info->kobj.parent);
	struct drm_device *ddev = dev_get_drvdata(dev);
	struct amdgpu_device *adev = drm_to_adev(ddev);
	int r;

	/* DF reconfiguration does not interact with accelerator state */

	if (!sysfs_streq(buf, "true"))
		return -EINVAL;

	r = psp_ual_set_station_config(&adev->psp, adev->ualink.psp_if_ver, stations);
	if (r)
		return r;
	r = psp_ual_query_info(&adev->psp, adev->ualink.psp_if_ver, info);
	if (r)
		return r;

	return count;
}

#define UALINK_STATION_CONFIG_ATTR(name) __ATTR(name, 0600,		\
		ualink_station_config_##name##_show,			\
		ualink_station_config_##name##_store)
static struct kobj_attribute ualink_station_config_flags  = UALINK_STATION_CONFIG_ATTR(flags);
static struct kobj_attribute ualink_station_config_lane_en_bitmap =
							UALINK_STATION_CONFIG_ATTR(lane_en_bitmap);
static struct kobj_attribute ualink_station_config_commit =
				__ATTR(commit, 0200, NULL, ualink_station_config_commit_store);

static const struct attribute *ualink_station_config_attrs[] = {
	&ualink_station_config_flags.attr,
	&ualink_station_config_lane_en_bitmap.attr,
	&ualink_station_config_commit.attr,
	NULL
};
#endif

static void ualink_station_config_release(struct kobject *kobj)
{
	struct amdgpu_ualink_station_config *stations = to_ualink_station_config(kobj);

	kfree(stations);
}

static const struct kobj_type ualink_station_config_ktype = {
	.release = ualink_station_config_release,
	.sysfs_ops = &kobj_sysfs_ops
};

int amdgpu_ualink_sysfs_init(struct amdgpu_device *adev)
{
	struct amdgpu_ualink_station_config *stations = NULL;
	struct amdgpu_ualink_vpod_config *vpod_config = NULL;
	struct amdgpu_ualink_ppod_setup *ppod_setup = NULL;
	struct amdgpu_ualink_info *info = NULL;
	int r;

	if (!amdgpu_device_ip_get_ip_block(adev, AMD_IP_BLOCK_TYPE_PSP))
		return 0;

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

	vpod_config = kzalloc(sizeof(*vpod_config), GFP_KERNEL);
	if (!vpod_config) {
		r = -ENOMEM;
		goto err_remove_ppod_setup_files;
	}
	r = kobject_init_and_add(&vpod_config->kobj, &ualink_vpod_config_ktype,
				 &info->kobj, "config");
	if (r)
		goto err_put_vpod_config;
#ifdef UALINK_ENABLE_DEPRECATED_CONFIG_SYSFS
	r = sysfs_create_files(&vpod_config->kobj, ualink_vpod_config_attrs);
	if (r)
		goto err_del_vpod_config;
#endif

	stations = kzalloc(sizeof(*stations), GFP_KERNEL);
	if (!stations) {
		r = -ENOMEM;
		goto err_remove_vpod_config_files;
	}
	r = kobject_init_and_add(&stations->kobj, &ualink_station_config_ktype,
				 &info->kobj, "stations");
	if (r)
		goto err_put_stations;
#ifdef UALINK_ENABLE_DEPRECATED_CONFIG_SYSFS
	r = sysfs_create_files(&stations->kobj, ualink_station_config_attrs);
	if (r)
		goto err_del_stations;
#endif

	adev->ualink.stations = stations;
	adev->ualink.config = vpod_config;
	adev->ualink.setup = ppod_setup;
	adev->ualink.info = info;

	return r;

#ifdef UALINK_ENABLE_DEPRECATED_CONFIG_SYSFS
err_del_stations:
	kobject_del(&stations->kobj);
#endif
err_put_stations:
	kobject_put(&stations->kobj);
err_remove_vpod_config_files:
#ifdef UALINK_ENABLE_DEPRECATED_CONFIG_SYSFS
	sysfs_remove_files(&vpod_config->kobj, ualink_vpod_config_attrs);
err_del_vpod_config:
#endif
	kobject_del(&vpod_config->kobj);
err_put_vpod_config:
	kobject_put(&vpod_config->kobj);
err_remove_ppod_setup_files:
#ifdef UALINK_ENABLE_DEPRECATED_CONFIG_SYSFS
	sysfs_remove_files(&ppod_setup->kobj, ualink_ppod_setup_attrs);
err_del_ppod_setup:
#endif
	kobject_del(&ppod_setup->kobj);
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
	if (adev->ualink.stations) {
#ifdef UALINK_ENABLE_DEPRECATED_CONFIG_SYSFS
		sysfs_remove_files(&adev->ualink.stations->kobj,
				   ualink_station_config_attrs);
#endif
		kobject_del(&adev->ualink.stations->kobj);
		kobject_put(&adev->ualink.stations->kobj);
		adev->ualink.stations = NULL;
	}
	if (adev->ualink.config) {
#ifdef UALINK_ENABLE_DEPRECATED_CONFIG_SYSFS
		sysfs_remove_files(&adev->ualink.config->kobj,
				   ualink_vpod_config_attrs);
#endif
		kobject_del(&adev->ualink.config->kobj);
		kobject_put(&adev->ualink.config->kobj);
		adev->ualink.config = NULL;
	}
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

static int amdgpu_ualink_npa_alloc_va(struct amdgpu_device *adev,
			       struct drm_mm_node *mm_node,
			       u64 va, u64 range_start,
			       u64 range_end, int size)
{
	struct amdgpu_ualink_npa_mm *npa_mm = &adev->ualink.npa_mm;
	u64 alignment = 0;
	int ret;

	mutex_lock(&npa_mm->mm_lock);
	if (va) {
		mm_node->start = va;
		mm_node->size = size;
		ret = drm_mm_reserve_node(&npa_mm->mm, mm_node);
	} else {
		if (!range_start && !range_end) {
			range_start = npa_mm->va_start;
			range_end = npa_mm->va_start + npa_mm->va_size;
		}

		/* If size is greater than 2MB (or 512 pages),
		 * align it to 2MB (or 512 pages) granularity.
		 */
		if (size >= 0x200)
			alignment = 0x200;

		ret = drm_mm_insert_node_in_range(&npa_mm->mm, mm_node, size,
						  alignment, 0, range_start,
						  range_end, 0);
	}
	mutex_unlock(&npa_mm->mm_lock);

	if (ret)
		dev_err(adev->dev, "Failed to allocate NPA address\n");

	return ret;
}

static void amdgpu_ualink_npa_free_va(struct amdgpu_device *adev,
			       struct drm_mm_node *mm_node)
{
	struct amdgpu_ualink_npa_mm *npa_mm = &adev->ualink.npa_mm;

	mutex_lock(&npa_mm->mm_lock);
	drm_mm_remove_node(mm_node);
	mutex_unlock(&npa_mm->mm_lock);
}

static void amdgpu_ualink_npa_mm_init(struct amdgpu_device *adev)
{
	struct amdgpu_ualink_npa_mm *npa_mm = &adev->ualink.npa_mm;
	u32 addr_mode = adev->ualink.info->vpod.addr_mode;
	u64 va_size;

	mutex_init(&npa_mm->mm_lock);

	if (addr_mode == AMDGPU_UALINK_ADDR_MODE_SOURCE_IDENT) {
		/* 51 bit address space in Source-Identification mode.
		 * 39 bits in page granularity.
		 */
		npa_mm->va_start = 0;
		va_size = GENMASK_ULL(38, 0);
	} else {
		/* 41 bit address space in Source-Alias mode.
		 * 29 bits in page granularity.
		 */
		/* First 8MB is reserved for Metadata NPAs */
		npa_mm->va_start = 0x800;
		va_size = GENMASK_ULL(28, 0);
	}

	npa_mm->va_size = va_size - npa_mm->va_start;
	drm_mm_init(&npa_mm->mm, npa_mm->va_start, npa_mm->va_size);
}

static void amdgpu_ualink_npa_mm_fini(struct amdgpu_device *adev)
{
	mutex_destroy(&adev->ualink.npa_mm.mm_lock);
	drm_mm_takedown(&adev->ualink.npa_mm.mm);
}

int amdgpu_ualink_manager_start(struct amdgpu_device *adev)
{
	int i, r;

	r = amdgpu_vm_init(adev, &adev->ualink.npa_vm, 0);
	if (r)
		goto out;

	/* For using CPU for page table updates. */
	r = amdgpu_vm_make_compute(adev, &adev->ualink.npa_vm);
	if (r)
		goto uninit_vm;

	adev->ualink.npa_wq = alloc_workqueue("NPA WQ", WQ_UNBOUND, 0);
	if (unlikely(!adev->ualink.npa_wq)) {
		dev_err(adev->dev, "Failed to allocate NPA WQ\n");
		r = -ENOMEM;
		goto uninit_vm;
	}

	/* Map this VM to NPA VMID */
	adev->mmhub.funcs->setup_vm_pt_regs(adev, adev->vm_manager.npa_vmid,
			amdgpu_gmc_pd_addr(adev->ualink.npa_vm.root.bo));

	xa_init_flags(&adev->ualink.exp_xa, XA_FLAGS_LOCK_BH);
	xa_init_flags(&adev->ualink.imp_xa, XA_FLAGS_LOCK_BH);
	xa_init_flags(&adev->ualink.handle_invalid_xa, XA_FLAGS_LOCK_BH);

	for (i = 0; i < AMDGPU_UALINK_ACCEL_MAX; i++) {
		init_completion(&adev->ualink.conn_state[i].hello_done);
		mutex_init(&adev->ualink.conn_state[i].lock);
		adev->ualink.conn_state[i].state = AMDGPU_UALINK_CONN_NOT_READY;
		INIT_LIST_HEAD(&adev->ualink.exp_handles_list[i]);
		INIT_LIST_HEAD(&adev->ualink.imp_handles_list[i]);
	}

	amdgpu_ualink_npa_mm_init(adev);

	return 0;

uninit_vm:
	amdgpu_vm_fini(adev, &adev->ualink.npa_vm);
out:
	return r;
}

void amdgpu_ualink_manager_stop(struct amdgpu_device *adev)
{
	int i;

	adev->mmhub.funcs->setup_vm_pt_regs(adev, adev->vm_manager.npa_vmid, 0);
	amdgpu_ualink_npa_mm_fini(adev);

	xa_destroy(&adev->ualink.exp_xa);
	xa_destroy(&adev->ualink.imp_xa);
	xa_destroy(&adev->ualink.handle_invalid_xa);

	for (i = 0; i < AMDGPU_UALINK_ACCEL_MAX; i++)
		mutex_destroy(&adev->ualink.conn_state[i].lock);

	destroy_workqueue(adev->ualink.npa_wq);
	amdgpu_vm_fini(adev, &adev->ualink.npa_vm);
}
