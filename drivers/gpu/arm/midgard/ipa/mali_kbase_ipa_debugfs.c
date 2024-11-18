// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2017-2022 ARM Limited. All rights reserved.
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

#include <linux/debugfs.h>
#include <linux/version_compat_defs.h>
#include <linux/list.h>
#include <linux/mutex.h>

#include "mali_kbase.h"
#include "mali_kbase_ipa.h"
#include "mali_kbase_ipa_debugfs.h"

struct kbase_ipa_model_param {
	char *name;
	union {
		void *voidp;
		s32 *s32p;
		char *str;
	} addr;
	size_t size;
	enum kbase_ipa_model_param_type type;
	struct kbase_ipa_model *model;
	struct list_head link;
};

static int param_int_get(void *data, u64 *val)
{
	struct kbase_ipa_model_param *param = data;

	mutex_lock(&param->model->kbdev->ipa.lock);
	*(s64 *) val = *param->addr.s32p;
	mutex_unlock(&param->model->kbdev->ipa.lock);

	return 0;
}

static int param_int_set(void *data, u64 val)
{
	struct kbase_ipa_model_param *param = data;
	struct kbase_ipa_model *model = param->model;
	s64 sval = (s64) val;
	s32 old_val;
	int err = 0;

	if (sval < S32_MIN || sval > S32_MAX)
		return -ERANGE;

	mutex_lock(&param->model->kbdev->ipa.lock);
	old_val = *param->addr.s32p;
	*param->addr.s32p = val;
	err = kbase_ipa_model_recalculate(model);
	if (err < 0)
		*param->addr.s32p = old_val;
	mutex_unlock(&param->model->kbdev->ipa.lock);

	return err;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_s32, param_int_get, param_int_set, "%lld\n");

static ssize_t param_string_get(struct file *file, char __user *user_buf,
				size_t count, loff_t *ppos)
{
	struct kbase_ipa_model_param *param = file->private_data;
	ssize_t ret;
	size_t len;

	mutex_lock(&param->model->kbdev->ipa.lock);
	len = strnlen(param->addr.str, param->size - 1) + 1;
	ret = simple_read_from_buffer(user_buf, count, ppos,
				      param->addr.str, len);
	mutex_unlock(&param->model->kbdev->ipa.lock);

	return ret;
}

static ssize_t param_string_set(struct file *file, const char __user *user_buf,
				size_t count, loff_t *ppos)
{
	struct kbase_ipa_model_param *param = file->private_data;
	struct kbase_ipa_model *model = param->model;
	char *old_str = NULL;
	ssize_t ret = count;
	size_t buf_size;
	int err;

	mutex_lock(&model->kbdev->ipa.lock);

	if (count > param->size) {
		ret = -EINVAL;
		goto end;
	}

	old_str = kstrndup(param->addr.str, param->size, GFP_KERNEL);
	if (!old_str) {
		ret = -ENOMEM;
		goto end;
	}

	buf_size = min(param->size - 1, count);
	if (copy_from_user(param->addr.str, user_buf, buf_size)) {
		ret = -EFAULT;
		goto end;
	}

	param->addr.str[buf_size] = '\0';

	err = kbase_ipa_model_recalculate(model);
	if (err < 0) {
		u32 string_len = strscpy(param->addr.str, old_str, param->size);

		string_len += sizeof(char);
		/* Make sure that the source string fit into the buffer. */
		KBASE_DEBUG_ASSERT(string_len <= param->size);
		CSTD_UNUSED(string_len);

		ret = err;
	}

end:
	kfree(old_str);
	mutex_unlock(&model->kbdev->ipa.lock);

	return ret;
}

static const struct file_operations fops_string = {
	.owner = THIS_MODULE,
	.read = param_string_get,
	.write = param_string_set,
	.open = simple_open,
	.llseek = default_llseek,
};

int kbase_ipa_model_param_add(struct kbase_ipa_model *model, const char *name,
			      void *addr, size_t size,
			      enum kbase_ipa_model_param_type type)
{
	struct kbase_ipa_model_param *param;

	param = kzalloc(sizeof(*param), GFP_KERNEL);

	if (!param)
		return -ENOMEM;

	/* 'name' is stack-allocated for array elements, so copy it into
	 * heap-allocated storage
	 */
	param->name = kstrdup(name, GFP_KERNEL);

	if (!param->name) {
		kfree(param);
		return -ENOMEM;
	}

	param->addr.voidp = addr;
	param->size = size;
	param->type = type;
	param->model = model;

	list_add(&param->link, &model->params);

	return 0;
}

void kbase_ipa_model_param_free_all(struct kbase_ipa_model *model)
{
	struct kbase_ipa_model_param *param_p, *param_n;

	list_for_each_entry_safe(param_p, param_n, &model->params, link) {
		list_del(&param_p->link);
		kfree(param_p->name);
		kfree(param_p);
	}
}

static int force_fallback_model_get(void *data, u64 *val)
{
	struct kbase_device *kbdev = data;

	mutex_lock(&kbdev->ipa.lock);
	*val = kbdev->ipa.force_fallback_model;
	mutex_unlock(&kbdev->ipa.lock);

	return 0;
}

static int force_fallback_model_set(void *data, u64 val)
{
	struct kbase_device *kbdev = data;

	mutex_lock(&kbdev->ipa.lock);
	kbdev->ipa.force_fallback_model = (val ? true : false);
	mutex_unlock(&kbdev->ipa.lock);

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(force_fallback_model,
		force_fallback_model_get,
		force_fallback_model_set,
		"%llu\n");

static int current_power_get(void *data, u64 *val)
{
	struct kbase_device *kbdev = data;
	struct devfreq *df = kbdev->devfreq;
	u32 power;

	kbase_pm_context_active(kbdev);
	/* The current model assumes that there's no more than one voltage
	 * regulator currently available in the system.
	 */
	kbase_get_real_power(df, &power,
		kbdev->current_nominal_freq,
		(kbdev->current_voltages[0] / 1000));
	kbase_pm_context_idle(kbdev);

	*val = power;

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(current_power, current_power_get, NULL, "%llu\n");

static void kbase_ipa_model_debugfs_init(struct kbase_ipa_model *model)
{
	struct list_head *it;
	struct dentry *dir;

	lockdep_assert_held(&model->kbdev->ipa.lock);

	dir = debugfs_create_dir(model->ops->name,
				 model->kbdev->mali_debugfs_directory);

	if (IS_ERR_OR_NULL(dir)) {
		dev_err(model->kbdev->dev,
			"Couldn't create mali debugfs %s directory",
			model->ops->name);
		return;
	}

	list_for_each(it, &model->params) {
		struct kbase_ipa_model_param *param =
				list_entry(it,
					   struct kbase_ipa_model_param,
					   link);
		const struct file_operations *fops = NULL;

		switch (param->type) {
		case PARAM_TYPE_S32:
			fops = &fops_s32;
			break;
		case PARAM_TYPE_STRING:
			fops = &fops_string;
			break;
		}

		if (unlikely(!fops)) {
			dev_err(model->kbdev->dev,
				"Type not set for %s parameter %s\n",
				model->ops->name, param->name);
		} else {
			debugfs_create_file(param->name, 0644,
					    dir, param, fops);
		}
	}
}

void kbase_ipa_model_param_set_s32(struct kbase_ipa_model *model,
	const char *name, s32 val)
{
	struct kbase_ipa_model_param *param;

	mutex_lock(&model->kbdev->ipa.lock);

	list_for_each_entry(param, &model->params, link) {
		if (!strcmp(param->name, name)) {
			if (param->type == PARAM_TYPE_S32) {
				*param->addr.s32p = val;
			} else {
				dev_err(model->kbdev->dev,
					"Wrong type for %s parameter %s\n",
					model->ops->name, param->name);
			}
			break;
		}
	}

	mutex_unlock(&model->kbdev->ipa.lock);
}
KBASE_EXPORT_TEST_API(kbase_ipa_model_param_set_s32);

void kbase_ipa_debugfs_init(struct kbase_device *kbdev)
{
	mutex_lock(&kbdev->ipa.lock);

	if (kbdev->ipa.configured_model != kbdev->ipa.fallback_model)
		kbase_ipa_model_debugfs_init(kbdev->ipa.configured_model);
	kbase_ipa_model_debugfs_init(kbdev->ipa.fallback_model);

	debugfs_create_file("ipa_current_power", 0444,
		kbdev->mali_debugfs_directory, kbdev, &current_power);
	debugfs_create_file("ipa_force_fallback_model", 0644,
		kbdev->mali_debugfs_directory, kbdev, &force_fallback_model);

	mutex_unlock(&kbdev->ipa.lock);
}
