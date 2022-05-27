// SPDX-License-Identifier: GPL-2.0-only

#include <linux/device.h>
#include <linux/regmap.h>

#include <linux/a2b/a2b.h>
#include <linux/a2b/a2b-regs.h>

// Latest upstream has sysfs_emit
#define sysfs_emit sprintf

#define A2B_ID_ATTR(_name)                                                     \
	static ssize_t _name##_show(struct device *dev,                        \
				    struct device_attribute *attr, char *buf)  \
	{                                                                      \
		struct a2b_node *node = dev_to_a2b_node(dev);                  \
                                                                               \
		return sysfs_emit(buf, "%x\n", node->_name);                   \
	}                                                                      \
	static DEVICE_ATTR_RO(_name);

static ssize_t chip_id_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct a2b_node *node = dev_to_a2b_node(dev);
	uint8_t chipid[6];
	unsigned int val;
	int ret;
	int i;

	for (i = 0; i < ARRAY_SIZE(chipid); i++) {
		ret = regmap_read(node->regmap, A2B_CHIPID(i), &val);
		if (ret < 0)
			return ret;
		chipid[i] = val;
	}

	return sysfs_emit(buf, "%6phN\n", chipid);
}

static ssize_t label_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct a2b_node *node = dev_to_a2b_node(dev);

	return sysfs_emit(buf, "%s\n", node->label);
}

static ssize_t bit_error_count_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct a2b_node *node = dev_to_a2b_node(dev);
	unsigned int val;
	int ret;

	ret = regmap_read(node->regmap, A2B_BECNT, &val);
	if (ret < 0)
		return ret;

	return sysfs_emit(buf, "%d\n", val);
}

static ssize_t bit_error_count_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct a2b_node *node = dev_to_a2b_node(dev);
	int ret;

	if (!sysfs_streq(buf, "clear"))
		return -EINVAL;

	ret = regmap_write(node->regmap, A2B_BECNT, 0x00);
	if (ret < 0)
		return ret;

	return count;
}

A2B_ID_ATTR(vendor_id);
A2B_ID_ATTR(product_id);
A2B_ID_ATTR(version_id);
A2B_ID_ATTR(capability_id);

static DEVICE_ATTR_RO(chip_id);
static DEVICE_ATTR_RO(label);
static DEVICE_ATTR_RW(bit_error_count);

static ssize_t prbs_error_count_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct a2b_node *node = dev_to_a2b_node(dev);
	uint32_t count = 0;
	unsigned int val;
	int ret;
	int i;

	for (i = 0; i < 4; i++) {
		ret = regmap_read(node->regmap, A2B_ERRCNT(i), &val);
		if (ret < 0)
			return ret;

		count |= val << (i * 8);
	}

	return sysfs_emit(buf, "%u\n", count);
}

static ssize_t i2s_test_mode_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct a2b_node *node = dev_to_a2b_node(dev);
	unsigned int val;
	int ret;

	if (sysfs_streq(buf, "pattern-to-tx-rx"))
		val = A2B_I2STEST_BUSLOOPBK | A2B_I2STEST_PATTRN2TX;
	else if (sysfs_streq(buf, "tx-to-rx"))
		val = A2B_I2STEST_BUSLOOPBK;
	else if (sysfs_streq(buf, "rx0-to-tx"))
		val = A2B_I2STEST_LOOPBK2TX | A2B_I2STEST_RX2LOOPBK;
	else if (sysfs_streq(buf, "rx1-to-tx"))
		val = A2B_I2STEST_LOOPBK2TX | A2B_I2STEST_RX2LOOPBK |
		      A2B_I2STEST_SELRX1;
	else if (sysfs_streq(buf, "pattern-to-tx"))
		val = A2B_I2STEST_PATTRN2TX;
	else if (sysfs_streq(buf, "none"))
		val = 0;
	else
		return -EINVAL;

	ret = regmap_write(node->regmap, A2B_I2STEST, val);
	if (ret < 0)
		return ret;

	return count;
}

static ssize_t i2s_test_mode_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct a2b_node *node = dev_to_a2b_node(dev);
	unsigned int val;
	const char *mode;
	int ret;

	ret = regmap_read(node->regmap, A2B_I2STEST, &val);
	if (ret < 0)
		return ret;

	if (val & A2B_I2STEST_BUSLOOPBK) {
		if (val & A2B_I2STEST_PATTRN2TX)
			mode = "pattern-to-tx-rx";
		else
			mode = "tx-to-rx";
	} else if (val & A2B_I2STEST_LOOPBK2TX) {
		if (val & A2B_I2STEST_SELRX1)
			mode = "rx1-to-tx";
		else
			mode = "rx0-to-tx";
	} else if (val & A2B_I2STEST_PATTRN2TX) {
		mode = "pattern-to-tx";
	} else {
		mode = "none";
	}

	return sysfs_emit(buf, "%s\n", mode);
}

struct a2b_err_attr {
	struct device_attribute attr;
	unsigned int type;
};

static ssize_t a2b_monitor_err_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct a2b_err_attr *err_attr =
		container_of(attr, struct a2b_err_attr, attr);
	struct a2b_node *node = dev_to_a2b_node(dev);
	unsigned int val;
	int ret;

	ret = regmap_read(node->regmap, A2B_BECCTL, &val);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%d\n", (bool)(val & err_attr->type));
}

static ssize_t a2b_monitor_err_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct a2b_err_attr *err_attr =
		container_of(attr, struct a2b_err_attr, attr);
	struct a2b_node *node = dev_to_a2b_node(dev);
	bool monitor;
	int ret;

	ret = strtobool(buf, &monitor);
	if (ret < 0)
		return ret;

	ret = regmap_update_bits(node->regmap, A2B_BECCTL, err_attr->type,
				 monitor ? err_attr->type : 0);
	if (ret)
		return ret;

	return count;
}

static ssize_t a2b_gen_err_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
	struct a2b_err_attr *err_attr =
		container_of(attr, struct a2b_err_attr, attr);
	struct a2b_node *node = dev_to_a2b_node(dev);
	bool generate;
	int ret;

	ret = strtobool(buf, &generate);
	if (ret < 0)
		return ret;

	if (generate) {
		ret = regmap_write(node->regmap, A2B_GENERR, err_attr->type);
		if (ret)
			return ret;
	}

	return count;
}

#define A2B_ERROR_ATTRS(_name, _type)                                          \
	struct a2b_err_attr a2b_monitor_err_attr_##_name = {                   \
		.attr = __ATTR(monitor_##_name##_error, 0644,                  \
			       a2b_monitor_err_show, a2b_monitor_err_store),   \
		.type = _type,                                                 \
	};                                                                     \
	struct a2b_err_attr a2b_gen_err_attr_##_name = {                       \
		.attr = __ATTR(generate_##_name##_error, 0200, NULL,           \
			       a2b_gen_err_store),                             \
		.type = _type,                                                 \
	}

static A2B_ERROR_ATTRS(header_count, A2B_GENERR_GENHCERR);
static A2B_ERROR_ATTRS(data_decoding, A2B_GENERR_GENDDERR);
static A2B_ERROR_ATTRS(crc, A2B_GENERR_GENCRCERR);
static A2B_ERROR_ATTRS(data_parity, A2B_GENERR_GENDPERR);
static A2B_ERROR_ATTRS(irq_frame_crc, A2B_GENERR_GENICRCERR);

static DEVICE_ATTR_RO(prbs_error_count);
static DEVICE_ATTR_RW(i2s_test_mode);

static struct attribute *a2b_dev_attrs[] = { &dev_attr_vendor_id.attr,
					     &dev_attr_product_id.attr,
					     &dev_attr_version_id.attr,
					     &dev_attr_capability_id.attr,
					     &dev_attr_chip_id.attr,
					     &dev_attr_label.attr,
					     NULL };

static umode_t a2b_dev_attrs_is_visible(struct kobject *kobj,
					struct attribute *attr, int n)
{
	struct device *dev = kobj_to_dev(kobj);
	struct a2b_node *node = dev_to_a2b_node(dev);

	if (attr == &dev_attr_label.attr && !node->label)
		return 0;

	return attr->mode;
}

static const struct attribute_group a2b_dev_attr_group = {
	.attrs = a2b_dev_attrs,
	.is_visible = a2b_dev_attrs_is_visible
};

static struct attribute *a2b_bist_attrs[] = {
	&dev_attr_prbs_error_count.attr,
	&dev_attr_bit_error_count.attr,
	&dev_attr_i2s_test_mode.attr,
	&a2b_monitor_err_attr_header_count.attr.attr,
	&a2b_monitor_err_attr_data_decoding.attr.attr,
	&a2b_monitor_err_attr_crc.attr.attr,
	&a2b_monitor_err_attr_data_parity.attr.attr,
	&a2b_monitor_err_attr_irq_frame_crc.attr.attr,
	&a2b_gen_err_attr_header_count.attr.attr,
	&a2b_gen_err_attr_data_decoding.attr.attr,
	&a2b_gen_err_attr_crc.attr.attr,
	&a2b_gen_err_attr_data_parity.attr.attr,
	&a2b_gen_err_attr_irq_frame_crc.attr.attr,
	NULL
};

static const struct attribute_group a2b_bist_attr_group = {
	.attrs = a2b_bist_attrs,
	.name = "bist",
};

const struct attribute_group *a2b_attr_groups[] = {
	&a2b_dev_attr_group,
	&a2b_bist_attr_group,
	NULL,
};

static ssize_t bus_status_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct a2b_mainnode *mainnode;
	struct a2b_subnode *subnode;
	int product_id, vendor_id;
	bool status = true;
	int i;

	mainnode = a2b_node_to_mainnode(dev_to_a2b_node(dev));

	for (i = 0; i < mainnode->num_subnodes; i++) {
		subnode = mainnode->subnodes[i];

		product_id = a2b_node_read(&subnode->node, A2B_PRODUCT);
		vendor_id = a2b_node_read(&subnode->node, A2B_VENDOR);

		if (subnode->node.vendor_id != vendor_id ||
		    subnode->node.product_id != product_id) {
			status = false;
			break;
		}
	}

	return sysfs_emit(buf, "%d\n", status);
}
static DEVICE_ATTR_RO(bus_status);

static struct attribute *a2b_mainnode_attrs[] = { &dev_attr_bus_status.attr,
						  NULL };

static const struct attribute_group a2b_mainnode_attr_group = {
	.attrs = a2b_mainnode_attrs,
};

const struct attribute_group *a2b_mainnode_attr_groups[] = {
	&a2b_mainnode_attr_group,
	NULL,
};
