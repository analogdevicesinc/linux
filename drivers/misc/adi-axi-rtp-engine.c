// SPDX-License-Identifier: GPL-2.0
/*
* RTP_ENGINE FPGA IP Core driver
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
#define ADI_RTP_ENGINE_REG_VERSION				0x0000
#define ADI_RTP_ENGINE_REG_DEST_MAC_ADDR_LB			0x0004
#define ADI_RTP_ENGINE_REG_DEST_MAC_ADDR_HB			0x0008
#define ADI_RTP_ENGINE_REG_SRC_MAC_ADDR_LB			0x000c
#define ADI_RTP_ENGINE_REG_SRC_MAC_ADDR_HB			0x0010
#define ADI_RTP_ENGINE_REG_SRC_IP_ADDR				0x0014
#define ADI_RTP_ENGINE_REG_DEST_IP_ADDR				0x0018
#define ADI_RTP_ENGINE_REG_IP_QOS				0x001C
#define ADI_RTP_ENGINE_REG_UDP_PORTS				0x0020
#define ADI_RTP_ENGINE_REG_NUM_LINES				0x0024
#define ADI_RTP_ENGINE_REG_NUM_PIXELS_PER_LINE			0x0028
#define ADI_RTP_ENGINE_REG_CONVERT_UYVY_TO_YUYV			0x002C

enum adi_axi_rtp_engine_attribute_id {
	ADI_RTP_ENGINE_ATTR_VERSION,
	ADI_RTP_ENGINE_ATTR_DEST_MAC_ADDR,
	ADI_RTP_ENGINE_ATTR_SRC_MAC_ADDR,
	ADI_RTP_ENGINE_ATTR_SRC_IP_ADDR,
	ADI_RTP_ENGINE_ATTR_DEST_IP_ADDR,
	ADI_RTP_ENGINE_ATTR_IP_QOS,
	ADI_RTP_ENGINE_ATTR_SRC_UDP_PORT,
	ADI_RTP_ENGINE_ATTR_DEST_UDP_PORT,
	ADI_RTP_ENGINE_ATTR_NUM_LINES,
	ADI_RTP_ENGINE_ATTR_NUM_PIXELS_PER_LINE,
	ADI_RTP_ENGINE_ATTR_CONVERT_UYVY_TO_YUYV
};

struct adi_axi_rtp_engine_attribute {
	enum adi_axi_rtp_engine_attribute_id id;
	struct device_attribute attr;
	u8 name[32];
};

#define to_rtp_engine_attribute(x) container_of(x, struct adi_axi_rtp_engine_attribute, attr);

/**
 * struct adi_axi_rtp_engine_state - Driver state information for the RTP ENGINE CORE
 * @base: Device register base address in memory
 * @regs: Device memory-mapped region regmap
 */
struct adi_axi_rtp_engine_state {
	void __iomem *base;
	struct regmap *regs;
};

static const struct regmap_config adi_axi_rtp_engine_regmap_cfg = {
	.name = "adi-axi-rtp-engine",
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
};

static ssize_t adi_axi_rtp_engine_show(struct device *dev, struct device_attribute *dev_attr, char *buf)
{
	const struct adi_axi_rtp_engine_attribute *attr;
	struct adi_axi_rtp_engine_state *st;
	u64 data64;
	u32 data;
	u32 mask;
	int ret;
	
	attr = to_rtp_engine_attribute(dev_attr);
	st = dev_get_drvdata(dev);

	switch (attr->id) {
		case ADI_RTP_ENGINE_ATTR_VERSION:
			ret = regmap_read(st->regs, ADI_RTP_ENGINE_REG_VERSION, &data);
			if (ret)
				return ret;
			return sysfs_emit(buf, "%d.%.2d.%c\n",
							ADI_AXI_PCORE_VER_MAJOR(data),
							ADI_AXI_PCORE_VER_MINOR(data),
							ADI_AXI_PCORE_VER_PATCH(data));
		case ADI_RTP_ENGINE_ATTR_DEST_MAC_ADDR:
			ret = regmap_bulk_read(st->regs, ADI_RTP_ENGINE_REG_DEST_MAC_ADDR_LB, &data64, 2);
			if (ret)
				return ret;
			
			return sysfs_emit(buf, "%x:%x:%x:%x:%x:%x\n", (unsigned int) ((data64 & 0x0000ff0000000000)>>40),
								(unsigned int) ((data64 & 0x000000ff00000000)>>32),
								(unsigned int) ((data64 & 0x00000000ff000000)>>24),
								(unsigned int) ((data64 & 0x0000000000ff0000)>>16),
								(unsigned int) ((data64 & 0x000000000000ff00)>>8),
								(unsigned int) (data64 & 0x00000000000000ff));
		case ADI_RTP_ENGINE_ATTR_SRC_MAC_ADDR:
			ret = regmap_bulk_read(st->regs, ADI_RTP_ENGINE_REG_SRC_MAC_ADDR_LB, &data64, 2);
			if (ret)
				return ret;
			return sysfs_emit(buf, "%x:%x:%x:%x:%x:%x\n", (unsigned int) ((data64 & 0x0000ff0000000000)>>40),
								(unsigned int) ((data64 & 0x000000ff00000000)>>32),
								(unsigned int) ((data64 & 0x00000000ff000000)>>24),
								(unsigned int) ((data64 & 0x0000000000ff0000)>>16),
								(unsigned int) ((data64 & 0x000000000000ff00)>>8),
								(unsigned int) (data64 & 0x00000000000000ff));
		case ADI_RTP_ENGINE_ATTR_SRC_IP_ADDR:
			ret = regmap_read(st->regs, ADI_RTP_ENGINE_REG_SRC_IP_ADDR, &data);
			if (ret)
				return ret;
			
			return sysfs_emit(buf, "%u.%u.%u.%u\n", (data & 0xff000000)>>24,
								(data & 0x00ff0000)>>16,
								(data & 0x0000ff00)>>8,
								data & 0x000000ff);
		case ADI_RTP_ENGINE_ATTR_DEST_IP_ADDR:
			ret = regmap_read(st->regs, ADI_RTP_ENGINE_REG_DEST_IP_ADDR, &data);
			if (ret)
				return ret;
			
			return sysfs_emit(buf, "%u.%u.%u.%u\n", (data & 0xff000000)>>24,
								(data & 0x00ff0000)>>16,
								(data & 0x0000ff00)>>8,
								data & 0x000000ff);
		case ADI_RTP_ENGINE_ATTR_IP_QOS:
			ret = regmap_read(st->regs, ADI_RTP_ENGINE_REG_IP_QOS, &data);
			if (ret)
				return ret;
			
			return sysfs_emit(buf, "%u\n", data);
		case ADI_RTP_ENGINE_ATTR_SRC_UDP_PORT:
			mask = 0xffff0000;
			ret = regmap_read(st->regs, ADI_RTP_ENGINE_REG_UDP_PORTS, &data);
			if (ret)
				return ret;
		
			return sysfs_emit(buf, "%u\n", (data & mask)>>16);
		case ADI_RTP_ENGINE_ATTR_DEST_UDP_PORT:
			mask = 0x0000ffff;
			ret = regmap_read(st->regs, ADI_RTP_ENGINE_REG_UDP_PORTS, &data);
			if (ret)
				return ret;
			
			return sysfs_emit(buf, "%u\n", data & mask);
		case ADI_RTP_ENGINE_ATTR_NUM_LINES:
			ret = regmap_read(st->regs, ADI_RTP_ENGINE_REG_NUM_LINES, &data);
			if (ret)
				return ret;
			
			return sysfs_emit(buf, "%u\n", data);
		case ADI_RTP_ENGINE_ATTR_NUM_PIXELS_PER_LINE:
			ret = regmap_read(st->regs, ADI_RTP_ENGINE_REG_NUM_PIXELS_PER_LINE, &data);
			if (ret)
				return ret;
			
			return sysfs_emit(buf, "%u\n", data);
		case ADI_RTP_ENGINE_ATTR_CONVERT_UYVY_TO_YUYV:
			ret = regmap_read(st->regs, ADI_RTP_ENGINE_REG_CONVERT_UYVY_TO_YUYV, &data);
			if (ret)
				return ret;

			return sysfs_emit(buf, "%u\n", data);

		default:
				return -EINVAL;
	}
}

static int adi_axi_rtp_engine_write_regs(const struct adi_axi_rtp_engine_attribute *attr,
									struct adi_axi_rtp_engine_state *st,
									const char *buf)
{
	u64 data64;
	u32 data;
	u32 current_val;
	u16 data16;
	int ret;

	switch (attr->id) {
		case ADI_RTP_ENGINE_ATTR_DEST_MAC_ADDR:
			ret = kstrtou64(buf, 0, &data64);
			if (ret)
				return ret;
			return regmap_bulk_write(st->regs, ADI_RTP_ENGINE_REG_DEST_MAC_ADDR_LB, &data64, 2);
		case ADI_RTP_ENGINE_ATTR_SRC_MAC_ADDR:
			ret = kstrtou64(buf, 0, &data64);
			printk(KERN_INFO "data64 is %u\n", data64);
			if (ret)
				return ret;
			return regmap_bulk_write(st->regs, ADI_RTP_ENGINE_REG_SRC_MAC_ADDR_LB, &data64, 2);
		case ADI_RTP_ENGINE_ATTR_SRC_IP_ADDR:
			ret = kstrtou32(buf, 0, &data);
			if (ret)
				return ret;
			return regmap_write(st->regs, ADI_RTP_ENGINE_REG_SRC_IP_ADDR, data);
		case ADI_RTP_ENGINE_ATTR_DEST_IP_ADDR:
			ret = kstrtou32(buf, 0, &data);
			if (ret)
				return ret;
			return regmap_write(st->regs, ADI_RTP_ENGINE_REG_DEST_IP_ADDR, data);
		case ADI_RTP_ENGINE_ATTR_IP_QOS:
			ret = kstrtou32(buf, 0, &data);
			if (ret)
				return ret;
			return regmap_write(st->regs, ADI_RTP_ENGINE_REG_IP_QOS, data);
		case ADI_RTP_ENGINE_ATTR_SRC_UDP_PORT:
			ret = kstrtou16(buf, 0, &data16);
			if (ret)
				return ret;
			
			ret = regmap_read(st->regs, ADI_RTP_ENGINE_REG_UDP_PORTS, &current_val);
			if (ret)
				return ret;
			return regmap_write(st->regs, ADI_RTP_ENGINE_REG_UDP_PORTS, (current_val & 0x0000FFFFU) | (uint32_t)data16 << 16);
		case ADI_RTP_ENGINE_ATTR_DEST_UDP_PORT:
			ret = kstrtou16(buf, 0, &data16);
			if (ret)
				return ret;

			ret = regmap_read(st->regs, ADI_RTP_ENGINE_REG_UDP_PORTS, &current_val);
			if (ret)
				return ret;
			return regmap_write(st->regs, ADI_RTP_ENGINE_REG_UDP_PORTS, (current_val & 0xFFFF0000U) | (uint32_t)data16);
		case ADI_RTP_ENGINE_ATTR_NUM_LINES:
			ret = kstrtou32(buf, 0, &data);
			if (ret)
				return ret;
			return regmap_write(st->regs, ADI_RTP_ENGINE_REG_NUM_LINES, data);
		case ADI_RTP_ENGINE_ATTR_NUM_PIXELS_PER_LINE:
			ret = kstrtou32(buf, 0, &data);
			if (ret)
				return ret;
			return regmap_write(st->regs, ADI_RTP_ENGINE_REG_NUM_PIXELS_PER_LINE, data);
		case ADI_RTP_ENGINE_ATTR_CONVERT_UYVY_TO_YUYV:
			ret = kstrtou32(buf, 0, &data);
			if (ret)
				return ret;
			return regmap_write(st->regs, ADI_RTP_ENGINE_REG_CONVERT_UYVY_TO_YUYV, data);
		default:
			return -EINVAL;
	}
}

static ssize_t adi_axi_rtp_engine_store(struct device *dev,
				 struct device_attribute *dev_attr,
				 const char *buf, size_t count)
{
	const struct adi_axi_rtp_engine_attribute *attr;
	struct adi_axi_rtp_engine_state *st;
	
	attr = to_rtp_engine_attribute(dev_attr);
	st = dev_get_drvdata(dev);

	ssize_t ret;
	ret = adi_axi_rtp_engine_write_regs(attr, st, buf) ?: count;
	if (ret)
		printk("Error in write_regs\n");
	return ret;
}

#define RTP_ENGINE_ATTR(_name, _id, _mode)							\
	struct adi_axi_rtp_engine_attribute dev_attr_##_name = 			\
		{															\
			.attr = __ATTR(_name, _mode, adi_axi_rtp_engine_show,	\
							adi_axi_rtp_engine_store),				\
			.id = _id,												\
		}

static const RTP_ENGINE_ATTR(version, ADI_RTP_ENGINE_ATTR_VERSION, 0444);
static const RTP_ENGINE_ATTR(dest_mac_addr, ADI_RTP_ENGINE_ATTR_DEST_MAC_ADDR, 0644);
static const RTP_ENGINE_ATTR(src_mac_addr, ADI_RTP_ENGINE_ATTR_SRC_MAC_ADDR, 0644);
static const RTP_ENGINE_ATTR(src_ip_addr, ADI_RTP_ENGINE_ATTR_SRC_IP_ADDR, 0644);
static const RTP_ENGINE_ATTR(dest_ip_addr, ADI_RTP_ENGINE_ATTR_DEST_IP_ADDR, 0644);
static const RTP_ENGINE_ATTR(ip_qos, ADI_RTP_ENGINE_ATTR_IP_QOS, 0644);
static const RTP_ENGINE_ATTR(src_udp_port, ADI_RTP_ENGINE_ATTR_SRC_UDP_PORT, 0644);
static const RTP_ENGINE_ATTR(dest_udp_port, ADI_RTP_ENGINE_ATTR_DEST_UDP_PORT, 0644);
static const RTP_ENGINE_ATTR(num_lines, ADI_RTP_ENGINE_ATTR_NUM_LINES, 0644);
static const RTP_ENGINE_ATTR(num_px_p_line, ADI_RTP_ENGINE_ATTR_NUM_PIXELS_PER_LINE, 0644);
static const RTP_ENGINE_ATTR(convert_yuyv_to_uyvy, ADI_RTP_ENGINE_ATTR_CONVERT_UYVY_TO_YUYV, 0644);

static const struct attribute *adi_axi_rtp_engine_base_attributes[] = {
	&dev_attr_version.attr.attr,
	&dev_attr_dest_mac_addr.attr.attr,
	&dev_attr_src_mac_addr.attr.attr,
	&dev_attr_src_ip_addr.attr.attr,
	&dev_attr_dest_ip_addr.attr.attr,
	&dev_attr_ip_qos.attr.attr,
	&dev_attr_src_udp_port.attr.attr,
	&dev_attr_dest_udp_port.attr.attr,
	&dev_attr_num_lines.attr.attr,
	&dev_attr_num_px_p_line.attr.attr,
	&dev_attr_convert_yuyv_to_uyvy.attr.attr,
};

static int adi_axi_rtp_engine_init_sysfs(struct platform_device *pdev,
										struct adi_axi_rtp_engine_state *st)
{
	size_t base_attr_count = ARRAY_SIZE(adi_axi_rtp_engine_base_attributes);
	size_t attribute_count = base_attr_count + 1;
	struct attribute_group *attr_group;
	struct attribute **rtp_engine_attrs;
	
	rtp_engine_attrs = devm_kcalloc(&pdev->dev, attribute_count,
									sizeof(*rtp_engine_attrs), GFP_KERNEL);
	if (!rtp_engine_attrs)
		return -ENOMEM;

	memcpy(rtp_engine_attrs, adi_axi_rtp_engine_base_attributes,
			sizeof(adi_axi_rtp_engine_base_attributes));

	attr_group = devm_kzalloc(&pdev->dev, sizeof(*attr_group), GFP_KERNEL);
	if (!attr_group)
		return -ENOMEM;
	
	attr_group->attrs = rtp_engine_attrs;

	return devm_device_add_group(&pdev->dev, attr_group);
}

static int adi_axi_rtp_engine_probe(struct platform_device *pdev)
{
	unsigned int expected_version, version;
	struct adi_axi_rtp_engine_state *st;
	int ret;

	st = devm_kzalloc(&pdev->dev, (sizeof(*st)), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(st->base))
		return PTR_ERR(st->base);

	platform_set_drvdata(pdev, st);

	st->regs = devm_regmap_init_mmio(&pdev->dev, st->base, &adi_axi_rtp_engine_regmap_cfg);
	if (IS_ERR(st->regs))
		return PTR_ERR(st->regs);
	
	ret = regmap_read(st->regs, ADI_AXI_REG_VERSION, &version);
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

	ret = adi_axi_rtp_engine_init_sysfs(pdev, st);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Failed to init sysfs\n");

	dev_info(&pdev->dev, "Probed Analog Devices AXI RTP Engine (%d.%.2d.%c)\n",
			ADI_AXI_PCORE_VER_MAJOR(version),
			ADI_AXI_PCORE_VER_MINOR(version),
			ADI_AXI_PCORE_VER_PATCH(version));

	return 0;
}

static const struct of_device_id adi_axi_rtp_engine_of_match[] = {
	{ .compatible = "adi,axi-rtp-engine" },
	{ }
};
MODULE_DEVICE_TABLE(of, adi_axi_rtp_engine_of_match);

static struct platform_driver adi_axi_rtp_engine_driver = {
	.driver = {
		.name = "adi,axi-rtp-engine",
		.of_match_table = adi_axi_rtp_engine_of_match,
	},
	.probe = adi_axi_rtp_engine_probe,
};

module_platform_driver(adi_axi_rtp_engine_driver);

MODULE_AUTHOR("Alin-Tudor Sferle <alin-tudor.sferle@analog.com>");
MODULE_DESCRIPTION("Driver to control ADI RTP Engine FPGA IP");
MODULE_LICENSE("GPL v2");
