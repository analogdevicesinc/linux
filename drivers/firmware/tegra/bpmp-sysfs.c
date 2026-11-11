// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2026, NVIDIA CORPORATION.
 */

#include <linux/device.h>
#include <linux/kobject.h>
#include <linux/kstrtox.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/sysfs.h>

#include <soc/tegra/bpmp.h>
#include <soc/tegra/bpmp-abi.h>

#include "bpmp-private.h"

struct tegra_bpmp_mbwt_attr {
	struct kobj_attribute attr;
	struct tegra_bpmp_mbwt_sysfs *mbwt;
	struct kobject *kobj;
	unsigned int instance;
	unsigned int vc_type;
};

struct tegra_bpmp_mbwt_sysfs {
	struct tegra_bpmp *bpmp;
	struct kobject *root;
	struct kobject **groups;
	struct tegra_bpmp_mbwt_attr *attrs;
	unsigned int num_groups;
	unsigned int num_attrs;
	/* Serializes bandwidth requests to firmware. */
	struct mutex lock;
};

static struct tegra_bpmp_mbwt_attr *
tegra_bpmp_mbwt_attr_from_kobj_attr(struct kobj_attribute *attr)
{
	return container_of(attr, struct tegra_bpmp_mbwt_attr, attr);
}

static ssize_t tegra_bpmp_mbwt_show(struct kobject *kobj,
				    struct kobj_attribute *attr, char *buf)
{
	struct tegra_bpmp_mbwt_attr *mbwt_attr;
	struct tegra_bpmp_mbwt_sysfs *mbwt;
	unsigned int bandwidth;
	int err;

	mbwt_attr = tegra_bpmp_mbwt_attr_from_kobj_attr(attr);
	mbwt = mbwt_attr->mbwt;

	mutex_lock(&mbwt->lock);
	err = tegra_bpmp_mbwt_get(mbwt->bpmp, mbwt_attr->instance,
				  mbwt_attr->vc_type, &bandwidth);
	mutex_unlock(&mbwt->lock);
	if (err)
		return err;

	return sysfs_emit(buf, "%u\n", bandwidth);
}

static ssize_t tegra_bpmp_mbwt_store(struct kobject *kobj,
				     struct kobj_attribute *attr,
				     const char *buf, size_t count)
{
	struct tegra_bpmp_mbwt_attr *mbwt_attr;
	struct tegra_bpmp_mbwt_sysfs *mbwt;
	unsigned int bandwidth;
	int err;

	err = kstrtou32(buf, 0, &bandwidth);
	if (err)
		return err;

	mbwt_attr = tegra_bpmp_mbwt_attr_from_kobj_attr(attr);
	mbwt = mbwt_attr->mbwt;

	mutex_lock(&mbwt->lock);
	err = tegra_bpmp_mbwt_set(mbwt->bpmp, mbwt_attr->instance,
				  mbwt_attr->vc_type, bandwidth);
	mutex_unlock(&mbwt->lock);
	if (err)
		return err;

	return count;
}

static void tegra_bpmp_mbwt_sysfs_teardown(void *data)
{
	struct tegra_bpmp_mbwt_sysfs *mbwt = data;
	unsigned int i;

	for (i = 0; i < mbwt->num_attrs; i++) {
		sysfs_remove_file(mbwt->attrs[i].kobj,
				  &mbwt->attrs[i].attr.attr);
		kobject_put(mbwt->attrs[i].kobj);
	}

	for (i = 0; i < mbwt->num_groups; i++)
		kobject_put(mbwt->groups[i]);

	kobject_put(mbwt->root);
}

static int tegra_bpmp_mbwt_sysfs_add_group(struct tegra_bpmp_mbwt_sysfs *mbwt,
					   const struct tegra_bpmp_mbwt_group *group)
{
	struct tegra_bpmp_mbwt_attr *attr;
	struct kobject *group_kobj;
	unsigned int i;
	int err;

	group_kobj = kobject_create_and_add(group->name, mbwt->root);
	if (!group_kobj)
		return -ENOMEM;

	mbwt->groups[mbwt->num_groups++] = group_kobj;

	for (i = 0; i < group->num_vcs; i++) {
		attr = &mbwt->attrs[mbwt->num_attrs];
		attr->kobj = kobject_create_and_add(group->vcs[i].name,
						    group_kobj);
		if (!attr->kobj)
			return -ENOMEM;

		sysfs_attr_init(&attr->attr.attr);
		attr->attr.attr.name = "bandwidth";
		attr->attr.attr.mode = 0644;
		attr->attr.show = tegra_bpmp_mbwt_show;
		attr->attr.store = tegra_bpmp_mbwt_store;
		attr->mbwt = mbwt;
		attr->instance = group->id;
		attr->vc_type = group->vcs[i].type;

		err = sysfs_create_file(attr->kobj, &attr->attr.attr);
		if (err) {
			kobject_put(attr->kobj);
			return err;
		}

		mbwt->num_attrs++;
	}

	return 0;
}

int tegra_bpmp_init_sysfs(struct tegra_bpmp *bpmp)
{
	const struct tegra_bpmp_mbwt_soc *soc = bpmp->soc->mbwt;
	struct tegra_bpmp_mbwt_sysfs *mbwt;
	unsigned int i, num_attrs = 0;
	int err;

	if (!soc)
		return 0;

	if (!tegra_bpmp_mrq_is_supported(bpmp, MRQ_SOCHUB_MBWT))
		return 0;

	if (!tegra_bpmp_mbwt_cmd_is_supported(bpmp, CMD_SOCHUB_MBWT_GET_BW) ||
	    !tegra_bpmp_mbwt_cmd_is_supported(bpmp, CMD_SOCHUB_MBWT_SET_BW))
		return 0;

	mbwt = devm_kzalloc(bpmp->dev, sizeof(*mbwt), GFP_KERNEL);
	if (!mbwt)
		return -ENOMEM;

	mbwt->bpmp = bpmp;
	mutex_init(&mbwt->lock);

	mbwt->groups = devm_kcalloc(bpmp->dev, soc->num_groups,
				    sizeof(*mbwt->groups), GFP_KERNEL);
	if (!mbwt->groups)
		return -ENOMEM;

	for (i = 0; i < soc->num_groups; i++)
		num_attrs += soc->groups[i].num_vcs;

	mbwt->attrs = devm_kcalloc(bpmp->dev, num_attrs,
				   sizeof(*mbwt->attrs), GFP_KERNEL);
	if (!mbwt->attrs)
		return -ENOMEM;

	mbwt->root = kobject_create_and_add("mbwt", &bpmp->dev->kobj);
	if (!mbwt->root)
		return -ENOMEM;

	for (i = 0; i < soc->num_groups; i++) {
		err = tegra_bpmp_mbwt_sysfs_add_group(mbwt, &soc->groups[i]);
		if (err)
			goto remove_sysfs;
	}

	err = devm_add_action_or_reset(bpmp->dev,
				       tegra_bpmp_mbwt_sysfs_teardown, mbwt);
	if (err)
		return err;

	return 0;

remove_sysfs:
	tegra_bpmp_mbwt_sysfs_teardown(mbwt);

	return err;
}
