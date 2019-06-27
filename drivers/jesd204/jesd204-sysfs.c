/**
 * The JESD204 framework
 *
 * Copyright (c) 2019 Analog Devices Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/device.h>
#include <linux/of.h>
#include <linux/slab.h>

#include "jesd204-priv.h"

enum {
	JESD204_CON_ATTR_DEV,
	JESD204_CON_ATTR_REF,
	JESD204_CON_ATTR_STATE,
	JESD204_CON_ATTR_ERROR,
};

static const char * const jesd204_con_attrs[] = {
	[JESD204_CON_ATTR_DEV] = "dev",
	[JESD204_CON_ATTR_REF] = "ref",
	[JESD204_CON_ATTR_STATE] = "state",
	[JESD204_CON_ATTR_ERROR] = "error",
};

static const char * const jesd204_lnk_attrs[] = {
	"sample_rate", "num_lanes", "num_converters",
	"octets_per_frame", "frames_per_multiframe",
	"bits_per_sample", "converter_resolution",
	"subclass", "scrambling"
};

enum jesd204_link_attr_type {
	JESD204_LINK_ATTR_U8,
	JESD204_LINK_ATTR_U64,
	JESD204_LINK_ATTR_BOOL,
};

static ssize_t jesd204_con_printf(struct jesd204_dev *jdev,
				  const char *str,
				  struct jesd204_dev_con_out *con,
				  char *buf)
{
	int attr_idx;

	if (!jdev)
		return -ENODEV;

	attr_idx = match_string(jesd204_con_attrs,
				ARRAY_SIZE(jesd204_con_attrs),
				str);
	if (attr_idx < 0)
		return attr_idx;

	switch (attr_idx) {
	case JESD204_CON_ATTR_DEV:
		if (!jdev->dev)
			return -EINVAL;
		return sprintf(buf, "%s\n", dev_name(jdev->dev));
	case JESD204_CON_ATTR_REF:
		if (con->of.args_count == 1)
			return sprintf(buf, "%s:%u\n", jdev->np->name,
				       con->of.args[0]);
		return sprintf(buf, "%s\n", jdev->np->name);
	case JESD204_CON_ATTR_STATE:
		return sprintf(buf, "%s\n", jesd204_state_str(con->state));
	case JESD204_CON_ATTR_ERROR:
		return sprintf(buf, "%d\n", con->error);
	default:
		return -EINVAL;
	}
}

static char *str_cut_from_chr(char *s, char c)
{
	char *ptr = strchr(s, '_');

	if (!ptr)
		return NULL;
	*ptr = '\0';
	ptr++;

	return ptr;
}

static ssize_t jesd204_con_show(struct device *dev,
				struct device_attribute *conattr,
				char *buf)
{
	struct jesd204_dev *jdev = jesd204_dev_from_device(dev);
	struct jesd204_dev_con_out *con;
	unsigned int len, idx, iter_idx;
	struct jesd204_dev_list *e;
	char *name, *ptr, *ptr1;
	int rc;

	if (!jdev)
		return -ENOENT;

	name = kstrdup(conattr->attr.name, GFP_KERNEL);
	if (!name)
		return -ENOMEM;

	/* cut string at first '_' */
	ptr = str_cut_from_chr(name, '_');
	if (!ptr) {
		rc = -EINVAL;
		goto out;
	}

	len = sizeof("in") - 1;
	if (strncmp("in", name, len) == 0) {
		rc = kstrtouint(&name[len], 10, &idx);
		if (rc)
			goto out;

		if (idx >= jdev->inputs_count) {
			rc = -ERANGE;
			goto out;
		}

		con = jdev->inputs[idx];
		rc = jesd204_con_printf(con->owner, ptr, con, buf);
		goto out;
	}

	len = sizeof("out") - 1;
	if (strncmp("out", name, len) != 0) {
		rc = -EINVAL;
		goto out;
	}

	/* get X from `outX_Y` first */
	rc = kstrtouint(&name[len], 10, &idx);
	if (rc)
		goto out;

	if (idx >= jdev->outputs_count) {
		rc = -ERANGE;
		goto out;
	}

	iter_idx = 0;
	list_for_each_entry(con, &jdev->outputs, list) {
		if (idx == iter_idx)
			break;
		iter_idx++;
	}

	/* now get Y from `outX_Y` */
	ptr1 = str_cut_from_chr(ptr, '_');
	if (!ptr1) {
		rc = -EINVAL;
		goto out;
	}

	rc = kstrtouint(ptr, 10, &idx);
	if (rc)
		goto out;

	if (idx >= con->dests_count) {
		rc = -ERANGE;
		goto out;
	}

	iter_idx = 0;
	list_for_each_entry(e, &con->dests, list) {
		if (idx == iter_idx) {
			rc = jesd204_con_printf(e->jdev, ptr1, con, buf);
			break;
		}
	}

out:
	kfree(name);
	return rc;
}

/* FIXME: add some jesd204_err() printk helpers */

static ssize_t name_show(struct device *dev,
			 struct device_attribute *devattr,
			 char *buf)
{
	struct jesd204_dev *jdev = jesd204_dev_from_device(dev);
	const char *dname, *rname;

	if (!jdev)
		return -ENOENT;

	dname = dev_name(jdev->dev);
	if (!dname)
		dname = "<no-device>";
	rname = jdev->np->name;

	return sprintf(buf, "%s:%s\n", rname, dname);
}
static DEVICE_ATTR_RO(name);

static struct attribute *jesd204_static_attributes[] = {
	&dev_attr_name.attr,
};

static ssize_t jesd204_link_show_store(struct device *dev,
				       struct device_attribute *devattr,
				       char *wbuf, const char *rbuf,
				       size_t count, bool store)
{
	struct jesd204_dev *jdev = jesd204_dev_from_device(dev);
	struct jesd204_dev_top *jdev_top = jesd204_dev_top_dev(jdev);
	enum jesd204_link_attr_type link_attr_type;
	struct jesd204_link *lnk;
	struct attribute *attr;
	int len, ret, idx;
	char *name, *ptr;
	u64 val, val1;
	void *field;

	if (!jdev_top || !jdev_top->num_links || !jdev_top->cur_links)
		return -ENOENT;

	attr = &devattr->attr;
	name = kstrdup(attr->name, GFP_KERNEL);
	if (!name)
		return -ENOMEM;

	ret = -EINVAL;

	len = sizeof("link") - 1;
	if (strncmp(name, "link", len))
		goto out;

	ptr = str_cut_from_chr(name, '_');
	if (!ptr)
		goto out;

	ret = kstrtouint(&name[len], 10, &idx);
	if (ret)
		goto out;

	if (idx >= jdev_top->num_links)
		goto out;

	lnk = &jdev_top->cur_links[idx];

	link_attr_type = JESD204_LINK_ATTR_U8;
	if (strcmp(ptr, "sample_rate") == 0) {
		field = &lnk->sample_rate;
		link_attr_type = JESD204_LINK_ATTR_U64;
	} else if (strcmp(ptr, "num_lanes") == 0) {
		field = &lnk->num_lanes;
	} else if (strcmp(ptr, "num_converters") == 0) {
		field = &lnk->num_converters;
	} else if (strcmp(ptr, "octets_per_frame") == 0) {
		field = &lnk->octets_per_frame;
	} else if (strcmp(ptr, "frames_per_multiframe") == 0) {
		field = &lnk->frames_per_multiframe;
	} else if (strcmp(ptr, "bits_per_sample") == 0) {
		field = &lnk->bits_per_sample;
	} else if (strcmp(ptr, "converter_resolution") == 0) {
		field = &lnk->converter_resolution;
	} else if (strcmp(ptr, "subclass") == 0) {
		field = &lnk->subclass;
	} else if (strcmp(ptr, "scrambling") == 0) {
		field = &lnk->scrambling;
		link_attr_type = JESD204_LINK_ATTR_BOOL;
	} else
		goto out;

	switch (link_attr_type) {
	case JESD204_LINK_ATTR_U64:
		val1 = *((u64 *)field);
		break;
	case JESD204_LINK_ATTR_BOOL:
		val1 = *((bool *)field);
		break;
	case JESD204_LINK_ATTR_U8:
		val1 = *((u8 *)field);
		break;
	}

	if (!store) {
		ret = sprintf(wbuf, "%llu", val1);
		goto out;
	}

	ret = kstrtoull(rbuf, 0, &val);
	if (ret)
		goto out;

	if (link_attr_type == JESD204_LINK_ATTR_BOOL)
		val = !!val;

	/* nothing changed, exit */
	if (val == val1) {
		ret = 0;
		goto out;
	}

	switch (link_attr_type) {
	case JESD204_LINK_ATTR_U64:
		*((u64 *)field) = val;
		break;
	case JESD204_LINK_ATTR_BOOL:
		*((bool *)field) = !!val;
		break;
	case JESD204_LINK_ATTR_U8:
		*((u8 *)field) = val & 0xff;
		break;
	}

out:
	kfree(name);

	return ret;
}

static ssize_t jesd204_link_show(struct device *dev,
				 struct device_attribute *devattr,
				 char *buf)
{
	return jesd204_link_show_store(dev, devattr, buf, NULL, 0, false);
}

static ssize_t jesd204_link_store(struct device *dev,
				struct device_attribute *devattr,
				const char *buf, size_t count)
{
	return jesd204_link_show_store(dev, devattr, NULL, buf, count, true);
}

static ssize_t jesd204_top_show(struct device *dev,
			 struct device_attribute *devattr,
			 char *buf)
{
	struct jesd204_dev *jdev = jesd204_dev_from_device(dev);
	struct jesd204_dev_top *jdev_top = jesd204_dev_top_dev(jdev);
	struct attribute *attr;

	if (!jdev_top)
		return -EINVAL;

	attr = &devattr->attr;
	if (strcmp(attr->name, "error") == 0)
		return sprintf(buf, "%d\n", jdev_top->error);
	else if (strcmp(attr->name, "cur_state") == 0)
		return sprintf(buf, "%s\n",
			       jesd204_state_str(jdev_top->cur_state));
	else if (strcmp(attr->name, "nxt_state") == 0)
		return sprintf(buf, "%s\n",
			       jesd204_state_str(jdev_top->nxt_state));
	else if (strcmp(attr->name, "change_kref") == 0)
		return sprintf(buf, "%u\n", kref_read(&(jdev_top->cb_ref)));

	return -EINVAL;
}

#define JESD204_TOP_DEVICE_ATTR_RO(_name) \
	struct device_attribute dev_attr_##_name = \
		__ATTR(_name, 0444, jesd204_top_show, NULL)

static JESD204_TOP_DEVICE_ATTR_RO(error);
static JESD204_TOP_DEVICE_ATTR_RO(cur_state);
static JESD204_TOP_DEVICE_ATTR_RO(nxt_state);
static JESD204_TOP_DEVICE_ATTR_RO(change_kref);

static struct attribute *jesd204_top_static_attributes[] = {
	&dev_attr_error.attr,
	&dev_attr_cur_state.attr,
	&dev_attr_nxt_state.attr,
	&dev_attr_change_kref.attr,
};

static int jesd204_dev_create_con_io_attrs(struct device *dev,
					   struct device_attribute *conattrs,
					   int conattr_idx, int i1, int i2)
{
	struct attribute *attr;
	int i;

	for (i = 0; i < ARRAY_SIZE(jesd204_con_attrs); i++, conattr_idx++) {
		attr = &(conattrs[conattr_idx].attr);
		if (i2 > -1)
			attr->name = devm_kasprintf(dev, GFP_KERNEL,
						    "out%d_%d_%s", i1, i2,
						    jesd204_con_attrs[i]);
		else
			attr->name = devm_kasprintf(dev, GFP_KERNEL,
						    "in%d_%s", i1,
						    jesd204_con_attrs[i]);
		if (!attr->name)
			return -ENOMEM;
		attr->mode = 0444;
		conattrs[conattr_idx].show = jesd204_con_show;
	}

	return conattr_idx;
}

static struct device_attribute *jesd204_dev_create_con_attrs(
		struct jesd204_dev *jdev, int *count)
{
	struct device_attribute *conattrs;
	struct jesd204_dev_con_out *c;
	int conattrs_idx, i1, i2, ret;

	/* count all inputs & outputs */
	*count = jdev->inputs_count * ARRAY_SIZE(jesd204_con_attrs);
	list_for_each_entry(c, &jdev->outputs, list)
		*count += c->dests_count * ARRAY_SIZE(jesd204_con_attrs);

	if (*count == 0)
		return NULL;

	/* alloc device attributes */
	conattrs = devm_kcalloc(jdev->dev, *count, sizeof(*conattrs),
				GFP_KERNEL);
	if (!conattrs)
		return ERR_PTR(-ENOMEM);

	conattrs_idx = 0;
	for (i1 = 0; i1 < jdev->inputs_count; i1++) {
		ret = jesd204_dev_create_con_io_attrs(jdev->dev, conattrs,
						      conattrs_idx, i1, -1);
		if (ret < 0)
			return ERR_PTR(ret);
		conattrs_idx = ret;
	}

	i1 = 0;
	list_for_each_entry(c, &jdev->outputs, list) {
		for (i2 = 0; i2 < c->dests_count; i2++) {
			ret = jesd204_dev_create_con_io_attrs(jdev->dev,
							      conattrs,
							      conattrs_idx,
							      i1, i2);
			if (ret < 0)
				return ERR_PTR(ret);
			conattrs_idx = ret;
		}
		i1++;
	}

	return conattrs;
}

static struct device_attribute *jesd204_dev_create_lnk_attrs(
		struct jesd204_dev *jdev, int *count)
{
	struct jesd204_dev_top *jdev_top = jesd204_dev_top_dev(jdev);
	struct device_attribute *lnkattrs;
	struct attribute *attr;
	int i1, i2, lnkattr_idx;

	if (!jdev_top || !jdev_top->num_links) {
		*count = 0;
		return NULL;
	}

	*count = jdev_top->num_links * ARRAY_SIZE(jesd204_lnk_attrs);

	lnkattrs = devm_kcalloc(jdev->dev, *count, sizeof(*lnkattrs),
				GFP_KERNEL);
	if (!lnkattrs)
		return ERR_PTR(-ENOMEM);

	lnkattr_idx = 0;
	for (i1 = 0; i1 < jdev_top->num_links; i1++) {
		for (i2 = 0; i2 < ARRAY_SIZE(jesd204_lnk_attrs); i2++) {
			attr = &(lnkattrs[lnkattr_idx].attr);
			attr->name = devm_kasprintf(jdev->dev, GFP_KERNEL,
						    "link%d_%s", i1,
						    jesd204_lnk_attrs[i2]);
			if (!attr->name)
				return ERR_PTR(-ENOMEM);
			/* FIXME: make this dynamic */
			attr->mode = 0664;
			lnkattrs[lnkattr_idx].show = jesd204_link_show;
			lnkattrs[lnkattr_idx].store = jesd204_link_store;
			lnkattr_idx++;
		}
	}

	return lnkattrs;
}

int jesd204_dev_create_sysfs(struct jesd204_dev *jdev)
{
	int i1, i2, conattrs_count, attrs_count;
	int topattrs_count, lnkattrs_count;
	struct device_attribute *conattrs, *lnkattrs;
	struct attribute **attrs;

	jdev->sysfs_attr_group.name = "jesd204";

	conattrs = jesd204_dev_create_con_attrs(jdev, &conattrs_count);
	if (IS_ERR(conattrs))
		return PTR_ERR(conattrs);

	lnkattrs = jesd204_dev_create_lnk_attrs(jdev, &lnkattrs_count);
	if (IS_ERR(lnkattrs))
		return PTR_ERR(lnkattrs);

	if (jdev->is_top)
		topattrs_count = ARRAY_SIZE(jesd204_top_static_attributes);
	else
		topattrs_count = 0;

	/* +1 for the NULL pointer at the end */
	attrs_count = ARRAY_SIZE(jesd204_static_attributes);
	attrs_count += topattrs_count + conattrs_count + lnkattrs_count + 1;
	attrs = devm_kcalloc(jdev->dev, attrs_count, sizeof(*attrs),
			     GFP_KERNEL);
	if (!attrs)
		return -ENOMEM;

	/* copy the static attributes */
	for (i1 = 0; i1 < ARRAY_SIZE(jesd204_static_attributes); i1++)
		attrs[i1] = jesd204_static_attributes[i1];

	for (i2 = 0; i2 < topattrs_count; i1++, i2++)
		attrs[i1] = jesd204_top_static_attributes[i2];

	for (i2 = 0; i2 < lnkattrs_count; i1++, i2++)
		attrs[i1] = &(lnkattrs[i2].attr);

	for (i2 = 0; i2 < conattrs_count; i1++, i2++)
		attrs[i1] = &(conattrs[i2].attr);

	jdev->sysfs_attr_group.attrs = attrs;

	return sysfs_create_group(&jdev->dev->kobj, &jdev->sysfs_attr_group);
}

void jesd204_dev_destroy_sysfs(struct jesd204_dev *jdev)
{
	struct device *dev = jdev->dev;

	sysfs_remove_group(&dev->kobj, &jdev->sysfs_attr_group);
}
