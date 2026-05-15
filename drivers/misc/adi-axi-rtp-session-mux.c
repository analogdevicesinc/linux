// SPDX-License-Identifier: GPL-2.0
/*
* RTP_SESSION_MUX FPGA IP Core driver
* Copyright 2025 - 2026 Analog Devices Inc.
*/

#include <linux/adi-axi-common.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sysfs.h>

/* Register Map */
#define ADI_RTP_SESSION_MUX_REG_VERSION				0x0000
#define ADI_RTP_SESSION_MUX_REG_START_TRANSFER			0x0004

enum adi_axi_rtp_session_mux_attribute_id {
	ADI_RTP_SESSION_MUX_ATTR_VERSION,
	ADI_RTP_SESSION_MUX_ATTR_START_TRANSFER
};

struct adi_axi_rtp_session_mux_attribute {
	enum adi_axi_rtp_session_mux_attribute_id id;
	struct device_attribute attr;
	u8 name[32];
};

#define to_rtp_session_mux_attribute(x) container_of(x, struct adi_axi_rtp_session_mux_attribute, attr);

/**
 * struct adi_axi_rtp_session_mux_state - Driver state information for the RTP SESSION MUX CORE
 * @base: Device register base address in memory
 * @regs: Device memory-mapped region regmap
 */
struct adi_axi_rtp_session_mux_state {
	void __iomem *base;
	struct regmap *regs;
};

static const struct regmap_config adi_axi_rtp_session_mux_regmap_cfg = {
	.name = "adi-axi-session-mux",
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
};

static ssize_t adi_axi_rtp_session_mux_show(struct device *dev, struct device_attribute *dev_attr, char *buf)
{
	const struct adi_axi_rtp_session_mux_attribute *attr;
	struct adi_axi_rtp_session_mux_state *st;
	u32 data;
	int ret;
	
	attr = to_rtp_session_mux_attribute(dev_attr);
	st = dev_get_drvdata(dev);

	switch (attr->id) {
		case ADI_RTP_SESSION_MUX_ATTR_VERSION:
			ret = regmap_read(st->regs, ADI_RTP_SESSION_MUX_REG_VERSION, &data);
			if (ret)
				return ret;
			return sysfs_emit(buf, "%d.%.2d.%c\n",
							ADI_AXI_PCORE_VER_MAJOR(data),
							ADI_AXI_PCORE_VER_MINOR(data),
							ADI_AXI_PCORE_VER_PATCH(data));
		case ADI_RTP_SESSION_MUX_ATTR_START_TRANSFER:
			ret = regmap_read(st->regs, ADI_RTP_SESSION_MUX_REG_START_TRANSFER, &data);
			if (ret)
				return ret;
			
			return sysfs_emit(buf, "%u\n", data);
		default:
				return -EINVAL;
	}
}

static int adi_axi_rtp_session_mux_write_regs(const struct adi_axi_rtp_session_mux_attribute *attr,
									struct adi_axi_rtp_session_mux_state *st,
									const char *buf)
{
	u32 data;
	int ret;

	switch (attr->id) {
		case ADI_RTP_SESSION_MUX_ATTR_START_TRANSFER:
			ret = kstrtou32(buf, 0, &data);
			if (ret)
				return ret;
			return regmap_write(st->regs, ADI_RTP_SESSION_MUX_REG_START_TRANSFER, data);
		default:
			return -EINVAL;
	}
}

static ssize_t adi_axi_rtp_session_mux_store(struct device *dev,
				 struct device_attribute *dev_attr,
				 const char *buf, size_t count)
{
	const struct adi_axi_rtp_session_mux_attribute *attr;
	struct adi_axi_rtp_session_mux_state *st;
	
	attr = to_rtp_session_mux_attribute(dev_attr);
	st = dev_get_drvdata(dev);

	ssize_t ret;
	ret = adi_axi_rtp_session_mux_write_regs(attr, st, buf) ?: count;
	if (ret)
		printk("Error in write_regs\n");
	return ret;
}

#define RTP_SESSION_MUX_ATTR(_name, _id, _mode)							\
	struct adi_axi_rtp_session_mux_attribute dev_attr_##_name = 				\
		{										\
			.attr = __ATTR(_name, _mode, adi_axi_rtp_session_mux_show,		\
							adi_axi_rtp_session_mux_store),		\
			.id = _id,								\
		}

static const RTP_SESSION_MUX_ATTR(version, ADI_RTP_SESSION_MUX_ATTR_VERSION, 0444);
static const RTP_SESSION_MUX_ATTR(start_transfer, ADI_RTP_SESSION_MUX_ATTR_START_TRANSFER, 0644);

static const struct attribute *adi_axi_rtp_session_mux_base_attributes[] = {
	&dev_attr_version.attr.attr,
	&dev_attr_start_transfer.attr.attr,
};

static int adi_axi_rtp_session_mux_init_sysfs(struct platform_device *pdev, struct adi_axi_rtp_session_mux_state *st)
{
	size_t base_attr_count = ARRAY_SIZE(adi_axi_rtp_session_mux_base_attributes);
	size_t attribute_count = base_attr_count + 1;
	struct attribute_group *attr_group;
	struct attribute **_attrs;
	
	_attrs = devm_kcalloc(&pdev->dev, attribute_count, sizeof(*_attrs), GFP_KERNEL);
	if (!_attrs)
		return -ENOMEM;

	memcpy(_attrs, adi_axi_rtp_session_mux_base_attributes,
			sizeof(adi_axi_rtp_session_mux_base_attributes));

	attr_group = devm_kzalloc(&pdev->dev, sizeof(*attr_group), GFP_KERNEL);
	if (!attr_group)
		return -ENOMEM;
	
	attr_group->attrs = _attrs;

	return devm_device_add_group(&pdev->dev, attr_group);
}

static int adi_axi_rtp_session_mux_probe(struct platform_device *pdev)
{
	unsigned int expected_version, version;
	struct adi_axi_rtp_session_mux_state *st;
	int ret;

	st = devm_kzalloc(&pdev->dev, (sizeof(*st)), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(st->base))
		return PTR_ERR(st->base);

	platform_set_drvdata(pdev, st);

	st->regs = devm_regmap_init_mmio(&pdev->dev, st->base, &adi_axi_rtp_session_mux_regmap_cfg);
	if (IS_ERR(st->regs))
		return PTR_ERR(st->regs);
	
	ret = regmap_read(st->regs, ADI_RTP_SESSION_MUX_REG_VERSION, &version);
	if (ret)
		return ret;

	expected_version = ADI_AXI_PCORE_VER(1, 0, 0);

	if (ADI_AXI_PCORE_VER_MAJOR(version) != ADI_AXI_PCORE_VER_MAJOR(expected_version))
		return dev_err_probe(&pdev->dev,
				     -ENODEV,
				     "Major version mismatch between PCORE and driver. Driver expected %d.%.2d.%c, PCORE reported %d.%.2d.%c\n",
				     ADI_AXI_PCORE_VER_MAJOR(expected_version),
				     ADI_AXI_PCORE_VER_MINOR(expected_version),
				     ADI_AXI_PCORE_VER_PATCH(expected_version),
				     ADI_AXI_PCORE_VER_MAJOR(version),
				     ADI_AXI_PCORE_VER_MINOR(version),
				     ADI_AXI_PCORE_VER_PATCH(version));

	ret = adi_axi_rtp_session_mux_init_sysfs(pdev, st);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Failed to init sysfs\n");

	dev_info(&pdev->dev, "Probed Analog Devices AXI RTP Session MUX (%d.%.2d.%c)\n",
			ADI_AXI_PCORE_VER_MAJOR(version),
			ADI_AXI_PCORE_VER_MINOR(version),
			ADI_AXI_PCORE_VER_PATCH(version));

	return 0;
}

static const struct of_device_id adi_axi_rtp_session_mux_of_match[] = {
	{ .compatible = "adi,axi-rtp-session-mux" },
	{ }
};
MODULE_DEVICE_TABLE(of, adi_axi_rtp_session_mux_of_match);

static struct platform_driver adi_axi_rtp_session_mux_driver = {
	.driver = {
		.name = "adi,axi-rtp-session-mux",
		.of_match_table = adi_axi_rtp_session_mux_of_match,
	},
	.probe = adi_axi_rtp_session_mux_probe,
};

module_platform_driver(adi_axi_rtp_session_mux_driver);

MODULE_AUTHOR("Alin-Tudor Sferle <alin-tudor.sferle@analog.com>");
MODULE_DESCRIPTION("Driver to control ADI RTP Session MUX FPGA IP");
MODULE_LICENSE("GPL v2");
