// SPDX-License-Identifier: GPL-2.0+
/**
 * The JESD204 framework - sysfs hooks
 *
 * Copyright (c) 2019 Analog Devices Inc.
 */

#include <linux/device.h>
#include <linux/of.h>
#include <linux/slab.h>

#include "jesd204-priv.h"
#include "attr.h"

/* FIXME: should go away in newer kernels */
#ifndef sizeof_field
#define sizeof_field(TYPE, MEMBER) sizeof((((TYPE *)0)->MEMBER))
#endif

#define JESD204_CON_ATTR(_name)	\
	[JESD204_CON_ATTR_ ## _name] = { .name = __stringify(_name) }

#define JESD204_LNK_ATTR_TYPE(_name, _type)				\
	[JESD204_LNK_ATTR_ ## _name] = {				\
		.name = __stringify(_name),				\
		.offset = offsetof(struct jesd204_link, _name),		\
		.size = sizeof_field(struct jesd204_link, _name),	\
		.type = _type,						\
	}

#define JESD204_LNK_ATTR_TYPE_PRIV(_name, _type)			\
	[JESD204_LNK_ATTR_ ## _name] = {				\
		.name = __stringify(_name),				\
		.offset = offsetof(struct jesd204_link_opaque, _name),	\
		.size = sizeof_field(struct jesd204_link_opaque, _name),\
		.type = _type,						\
		.priv = true,						\
	}

#define JESD204_LNK_ATTR_INT(_name)	\
	JESD204_LNK_ATTR_TYPE(_name, JESD204_ATTR_TYPE_INT)

#define JESD204_LNK_ATTR_UINT(_name)	\
	JESD204_LNK_ATTR_TYPE(_name, JESD204_ATTR_TYPE_UINT)

#define JESD204_LNK_ATTR_BOOL(_name)	\
	JESD204_LNK_ATTR_TYPE(_name, JESD204_ATTR_TYPE_BOOL)

#define JESD204_LNK_ATTR_BOOL_PRIV(_name)	\
	JESD204_LNK_ATTR_TYPE_PRIV(_name, JESD204_ATTR_TYPE_BOOL)

#define JESD204_LNK_ATTR_STR_PRIV(_name)	\
	JESD204_LNK_ATTR_TYPE_PRIV(_name, JESD204_ATTR_TYPE_STR)

enum {
	JESD204_ATTR_TYPE_INT,
	JESD204_ATTR_TYPE_UINT,
	JESD204_ATTR_TYPE_BOOL,
	JESD204_ATTR_TYPE_STR,
};

struct jesd204_attr {
	const char *name;
	size_t offset;
	size_t size;
	int type;
	bool priv;
};

enum {
	JESD204_CON_ATTR_to,
	JESD204_CON_ATTR_id,
	JESD204_CON_ATTR_topo_id,
	JESD204_CON_ATTR_link_id,
	JESD204_CON_ATTR_state,
	JESD204_CON_ATTR_error,
};

static const struct attribute jesd204_con_attrs[] = {
	JESD204_CON_ATTR(to),
	JESD204_CON_ATTR(id),
	JESD204_CON_ATTR(topo_id),
	JESD204_CON_ATTR(link_id),
	JESD204_CON_ATTR(state),
	JESD204_CON_ATTR(error),
};

enum {
	JESD204_LNK_ATTR_link_id,
	JESD204_LNK_ATTR_error,
	JESD204_LNK_ATTR_state,
	JESD204_LNK_ATTR_fsm_paused,
	JESD204_LNK_ATTR_sample_rate,
	JESD204_LNK_ATTR_is_transmit,
	JESD204_LNK_ATTR_num_lanes,
	JESD204_LNK_ATTR_num_converters,
	JESD204_LNK_ATTR_octets_per_frame,
	JESD204_LNK_ATTR_frames_per_multiframe,
	JESD204_LNK_ATTR_num_of_multiblocks_in_emb,
	JESD204_LNK_ATTR_bits_per_sample,
	JESD204_LNK_ATTR_converter_resolution,
	JESD204_LNK_ATTR_jesd_version,
	JESD204_LNK_ATTR_jesd_encoder,
	JESD204_LNK_ATTR_subclass,
	JESD204_LNK_ATTR_device_id,
	JESD204_LNK_ATTR_bank_id,
	JESD204_LNK_ATTR_scrambling,
	JESD204_LNK_ATTR_high_density,
	JESD204_LNK_ATTR_ctrl_words_per_frame_clk,
	JESD204_LNK_ATTR_ctrl_bits_per_sample,
	JESD204_LNK_ATTR_samples_per_conv_frame,
	JESD204_LNK_ATTR_dac_adj_resolution_steps,
	JESD204_LNK_ATTR_dac_adj_direction,
	JESD204_LNK_ATTR_dac_phase_adj,
};

static const struct jesd204_attr jesd204_lnk_attrs[] = {
	JESD204_LNK_ATTR_UINT(link_id),
	JESD204_LNK_ATTR_INT(error),
	JESD204_LNK_ATTR_STR_PRIV(state),
	JESD204_LNK_ATTR_BOOL_PRIV(fsm_paused),
	JESD204_LNK_ATTR_UINT(sample_rate),
	JESD204_LNK_ATTR_BOOL(is_transmit),
	JESD204_LNK_ATTR_UINT(num_lanes),
	JESD204_LNK_ATTR_UINT(num_converters),
	JESD204_LNK_ATTR_UINT(octets_per_frame),
	JESD204_LNK_ATTR_UINT(frames_per_multiframe),
	JESD204_LNK_ATTR_UINT(num_of_multiblocks_in_emb),
	JESD204_LNK_ATTR_UINT(bits_per_sample),
	JESD204_LNK_ATTR_UINT(converter_resolution),
	JESD204_LNK_ATTR_UINT(jesd_version),
	JESD204_LNK_ATTR_UINT(jesd_encoder),
	JESD204_LNK_ATTR_UINT(subclass),
	JESD204_LNK_ATTR_UINT(device_id),
	JESD204_LNK_ATTR_UINT(bank_id),
	JESD204_LNK_ATTR_BOOL(scrambling),
	JESD204_LNK_ATTR_BOOL(high_density),
	JESD204_LNK_ATTR_UINT(ctrl_words_per_frame_clk),
	JESD204_LNK_ATTR_UINT(ctrl_bits_per_sample),
	JESD204_LNK_ATTR_UINT(samples_per_conv_frame),
	JESD204_LNK_ATTR_UINT(dac_adj_resolution_steps),
	JESD204_LNK_ATTR_UINT(dac_adj_direction),
	JESD204_LNK_ATTR_BOOL(dac_phase_adj),
};

static int __jesd204_match_attribute_name(const struct jesd204_attr *array,
					  size_t n, const char *string)
{
	int index;

	for (index = 0; index < n; index++) {
		if (!strcmp(array[index].name, string))
			return index;
	}

	return -EINVAL;
}

#define jesd204_match_attribute_name(_a, _s)	\
	__jesd204_match_attribute_name(_a, ARRAY_SIZE(_a), _s)

static ssize_t name_show(struct device *dev,
			 struct device_attribute *devattr,
			 char *buf)
{
	struct jesd204_dev *jdev = dev_to_jesd204_dev(dev);

	if (!jdev->dev.parent)
		return sprintf(buf, "%pOF,%s\n", jdev->np, dev_name(&jdev->dev));

	return sprintf(buf, "%pOF,%s,parent=%s\n", jdev->np, dev_name(&jdev->dev), dev_name(jdev->dev.parent));
}
static DEVICE_ATTR_RO(name);

static struct attribute *jesd204_static_attributes[] = {
	&dev_attr_name.attr,
};

static ssize_t jesd204_con_printf(struct jesd204_dev *jdev,
				  const char *str,
				  struct jesd204_dev_con_out *con,
				  char *buf)
{
	int attr_idx;

	if (!jdev)
		return -ENODEV;

	attr_idx = match_attribute_name(jesd204_con_attrs, str);
	if (attr_idx < 0)
		return attr_idx;

	switch (attr_idx) {
	case JESD204_CON_ATTR_to:
		return name_show(&jdev->dev, NULL, buf);
	case JESD204_CON_ATTR_id:
		return sprintf(buf, "%u\n", con->id);
	case JESD204_CON_ATTR_topo_id:
		return sprintf(buf, "%u\n", con->topo_id);
	case JESD204_CON_ATTR_link_id:
		return sprintf(buf, "%u\n", con->link_id);
	case JESD204_CON_ATTR_state:
		return sprintf(buf, "%s\n", jesd204_state_str(con->state));
	case JESD204_CON_ATTR_error:
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
	struct jesd204_dev *jdev = dev_to_jesd204_dev(dev);
	struct jesd204_dev_con_out *con;
	unsigned int len, idx, iter_idx;
	struct jesd204_dev_list_entry *e;
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
	list_for_each_entry(con, &jdev->outputs, entry) {
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
	list_for_each_entry(e, &con->dests, entry) {
		if (idx == iter_idx) {
			rc = jesd204_con_printf(e->jdev, ptr1, con, buf);
			break;
		}
	}

out:
	kfree(name);
	return rc;
}

static ssize_t jesd204_show_store_int(u64 *val, size_t usize,
				      char *wbuf, const char *rbuf,
				      size_t count, bool store, bool is_signed)
{
	u64 val1 = 0;
	int ret, max;

	if (!store) {
		memcpy(&val1, val, usize);
		if (is_signed)
			return sprintf(wbuf, "%lld\n", val1);
		return sprintf(wbuf, "%llu\n", val1);
	}

	if (is_signed)
		ret = kstrtoll(rbuf, 0, &val1);
	else
		ret = kstrtoull(rbuf, 0, &val1);
	if (ret)
		return ret;

	switch (usize) {
	case 1:
		max = 0xff;
		break;
	case 2:
		max = 0xffff;
		break;
	case 4:
		max = 0xffffffff;
		break;
	case 8:
		max = 0;
		break;
	default:
		return -EINVAL;
	}

	if (max && val1 > max)
		return -EINVAL;

	memcpy(val, &val1, usize);

	return count;
}

static ssize_t jesd204_show_store_bool(bool *val, char *wbuf, const char *rbuf,
				       size_t count, bool store)
{
	u64 val1;
	int ret;

	if (!store)
		return sprintf(wbuf, "%u\n", !!(*val));

	ret = kstrtoull(rbuf, 0, &val1);
	if (ret)
		return ret;
	*val = !!val1;

	return count;
}

static ssize_t jesd204_lnk_show_store_str(u32 *val,
					  const char *attr_name,
					  char *wbuf, const char *rbuf,
					  size_t count, bool store)
{
	if (!wbuf)
		return -EINVAL;

	if (sysfs_streq("state", attr_name))
		return sprintf(wbuf, "%s\n", jesd204_state_str(*val));

	return -EINVAL;
}

static ssize_t jesd204_link_show_store(struct device *dev,
				       struct device_attribute *devattr,
				       char *wbuf, const char *rbuf,
				       size_t count, bool store)
{
	struct jesd204_dev *jdev = dev_to_jesd204_dev(dev);
	struct jesd204_dev_top *jdev_top = jesd204_dev_top_dev(jdev);
	const struct jesd204_attr *jattr;
	struct attribute *attr;
	void *field;
	char *name, *ptr;
	unsigned int idx;
	int len, ret;
	bool is_signed = false;

	if (!jdev_top || !jdev_top->num_links || !jdev_top->active_links)
		return -ENOENT;

	attr = &devattr->attr;
	name = kstrdup(attr->name, GFP_KERNEL);
	if (!name)
		return -ENOMEM;

	len = sizeof("link") - 1;
	if (strncmp(name, "link", len)) {
		ret = -EINVAL;
		goto out;
	}

	ptr = str_cut_from_chr(name, '_');
	if (!ptr) {
		ret = -EINVAL;
		goto out;
	}

	ret = kstrtouint(&name[len], 0, &idx);
	if (ret)
		goto out;

	if (idx >= jdev_top->num_links) {
		ret = -EINVAL;
		goto out;
	}

	ret = jesd204_match_attribute_name(jesd204_lnk_attrs, ptr);
	if (ret < 0)
		goto out;

	jattr = &jesd204_lnk_attrs[ret];
	if (jattr->priv)
		field = &jdev_top->active_links[idx];
	else
		field = &jdev_top->active_links[idx].link;

	field += jattr->offset;

	switch (jattr->type) {
	case JESD204_ATTR_TYPE_STR:
		ret = jesd204_lnk_show_store_str(field, jattr->name,
						 wbuf, rbuf, count, store);
		break;
	case JESD204_ATTR_TYPE_BOOL:
		ret = jesd204_show_store_bool(field, wbuf, rbuf,
					      count, store);
		break;
	case JESD204_ATTR_TYPE_INT:
		is_signed = true;
	case JESD204_ATTR_TYPE_UINT:
		ret = jesd204_show_store_int(field, jattr->size, wbuf, rbuf,
					     count, store, is_signed);
		break;
	default:
		ret = -EINVAL;
		goto out;
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
	struct jesd204_dev *jdev = dev_to_jesd204_dev(dev);
	struct jesd204_dev_top *jdev_top = jesd204_dev_top_dev(jdev);
	struct attribute *attr;

	if (!jdev_top)
		return -EINVAL;

	attr = &devattr->attr;
	if (strcmp(attr->name, "initialized") == 0)
		return sprintf(buf, "%d\n", jdev_top->initialized);
	else if (strcmp(attr->name, "num_retries") == 0)
		return sprintf(buf, "%u\n", jdev_top->num_retries);
	else if (strcmp(attr->name, "topology_id") == 0)
		return sprintf(buf, "%u\n", jdev_top->topo_id);
	else if (strcmp(attr->name, "num_links") == 0)
		return sprintf(buf, "%u\n", jdev_top->num_links);

	if (strcmp(attr->name, "link_ids") == 0) {
		int i, ret = 0;

		for (i = 0; i < jdev_top->num_links; i++)
			ret += sprintf(&buf[ret], "%u ", jdev_top->link_ids[i]);
		ret += sprintf(&buf[ret], "\n");

		return ret;
	}

	return -EINVAL;
}

#define JESD204_TOP_DEVICE_ATTR_RO(_name) \
	struct device_attribute dev_attr_##_name = \
		__ATTR(_name, 0444, jesd204_top_show, NULL)

static JESD204_TOP_DEVICE_ATTR_RO(initialized);
static JESD204_TOP_DEVICE_ATTR_RO(num_retries);
static JESD204_TOP_DEVICE_ATTR_RO(topology_id);
static JESD204_TOP_DEVICE_ATTR_RO(num_links);
static JESD204_TOP_DEVICE_ATTR_RO(link_ids);

static struct attribute *jesd204_top_static_attributes[] = {
	&dev_attr_initialized.attr,
	&dev_attr_num_retries.attr,
	&dev_attr_topology_id.attr,
	&dev_attr_num_links.attr,
	&dev_attr_link_ids.attr,
};

static int jesd204_dev_create_con_io_attrs(struct device *parent,
					   struct device_attribute *conattrs,
					   int conattr_idx, int i1, int i2)
{
	struct attribute *attr;
	int i;

	for (i = 0; i < ARRAY_SIZE(jesd204_con_attrs); i++, conattr_idx++) {
		attr = &(conattrs[conattr_idx].attr);
		if (i2 > -1)
			attr->name = devm_kasprintf(parent, GFP_KERNEL,
						    "out%d_%d_%s", i1, i2,
						    jesd204_con_attrs[i].name);
		else
			attr->name = devm_kasprintf(parent, GFP_KERNEL,
						    "in%d_%s", i1,
						    jesd204_con_attrs[i].name);
		if (!attr->name)
			return -ENOMEM;
		if (jesd204_con_attrs[i].mode)
			attr->mode = jesd204_con_attrs[i].mode;
		else
			attr->mode = 0444;
		conattrs[conattr_idx].show = jesd204_con_show;
	}

	return conattr_idx;
}

static struct device_attribute *jesd204_dev_create_con_attrs(
		struct jesd204_dev *jdev, int *count)
{
	struct device *parent = jdev->dev.parent;
	struct device_attribute *conattrs;
	struct jesd204_dev_con_out *c;
	int conattrs_idx, i1, i2, ret;

	/* count all inputs & outputs */
	*count = jdev->inputs_count * ARRAY_SIZE(jesd204_con_attrs);
	list_for_each_entry(c, &jdev->outputs, entry)
		*count += c->dests_count * ARRAY_SIZE(jesd204_con_attrs);

	if (*count == 0)
		return NULL;

	/* alloc device attributes */
	conattrs = devm_kcalloc(parent, *count, sizeof(*conattrs),
				GFP_KERNEL);
	if (!conattrs)
		return ERR_PTR(-ENOMEM);

	conattrs_idx = 0;
	for (i1 = 0; i1 < jdev->inputs_count; i1++) {
		ret = jesd204_dev_create_con_io_attrs(parent, conattrs,
						      conattrs_idx, i1, -1);
		if (ret < 0)
			return ERR_PTR(ret);
		conattrs_idx = ret;
	}

	i1 = 0;
	list_for_each_entry(c, &jdev->outputs, entry) {
		for (i2 = 0; i2 < c->dests_count; i2++) {
			ret = jesd204_dev_create_con_io_attrs(parent,
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
	struct device *parent = jdev->dev.parent;
	struct device_attribute *lnkattrs;
	const struct jesd204_attr *jattr;
	struct attribute *attr;
	int num_lnk_attrs = ARRAY_SIZE(jesd204_lnk_attrs);
	int i1, i2, lnkattr_idx;

	if (!jdev_top || !jdev_top->num_links) {
		*count = 0;
		return NULL;
	}

	*count = jdev_top->num_links * num_lnk_attrs;

	lnkattrs = devm_kcalloc(parent, *count, sizeof(*lnkattrs),
				GFP_KERNEL);
	if (!lnkattrs)
		return ERR_PTR(-ENOMEM);

	lnkattr_idx = 0;
	for (i1 = 0; i1 < jdev_top->num_links; i1++) {
		for (i2 = 0; i2 < num_lnk_attrs; i2++) {
			jattr = &jesd204_lnk_attrs[i2];
			attr = &(lnkattrs[lnkattr_idx].attr);
			attr->mode = 0444;
			attr->name = devm_kasprintf(parent, GFP_KERNEL,
						    "link%d_%s", i1,
						    jattr->name);
			if (!attr->name)
				return ERR_PTR(-ENOMEM);
			lnkattrs[lnkattr_idx].show = jesd204_link_show;
			lnkattrs[lnkattr_idx].store = jesd204_link_store;
			lnkattr_idx++;
		}
	}

	return lnkattrs;
}

int jesd204_dev_create_sysfs(struct jesd204_dev *jdev)
{
	struct device_attribute *conattrs, *lnkattrs;
	struct device *parent = jdev->dev.parent;
	int i1, i2, conattrs_count, attrs_count;
	int topattrs_count, lnkattrs_count;
	struct attribute **attrs;

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
	attrs = devm_kcalloc(parent, attrs_count, sizeof(*attrs),
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

	return sysfs_create_group(&jdev->dev.kobj, &jdev->sysfs_attr_group);
}

void jesd204_dev_destroy_sysfs(struct jesd204_dev *jdev)
{
	sysfs_remove_group(&jdev->dev.kobj, &jdev->sysfs_attr_group);
}
