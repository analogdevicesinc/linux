// SPDX-License-Identifier: GPL-2.0
/*
 * Corundum MQNIC Application driver
 *
 * Copyright 2025 Analog Devices Inc.
 *
 */

#include <linux/auxiliary_bus.h>
#include <linux/bitfield.h>
#include <linux/module.h>
#include "mqnic.h"
#include <linux/regmap.h>


/* Register Map */
// Generic
#define MQNIC_APP_VERSION_REG                 0x0000
#define MQNIC_APP_SCRATCH_REG                 0x0004
#define MQNIC_APP_CONTROL_CNT_REG             0x0008
#define MQNIC_APP_CONTROL_CNT_START           BIT(0)
#define MQNIC_APP_CONTROL_CNT_STOP            BIT(1)
#define MQNIC_APP_CLEAR_CNT_REG               0x000c
#define MQNIC_APP_CLEAR_CNT_BIT               BIT(0)
#define MQNIC_APP_CNT_VALUE_REG               0x0010

// Data generator
#define MQNIC_APP_START_GEN_REG               0x0014
#define MQNIC_APP_START_GEN_BIT               BIT(0)

// Packetizer 6
#define MQNIC_APP_PKT_SIZE_REG                0x0018 /* 2048 * packet_size */

// Ethernet header 7
#define MQNIC_APP_ETH_DEST_MAC_HI_REG         0x001C /* MAC higher 2 bytes */
#define MQNIC_APP_ETH_DEST_MAC_LO_REG         0x0020 /* MAC lower 4 bytes */
#define MQNIC_APP_ETH_SRC_MAC_HI_REG          0x0024 /* MAC higher 2 bytes */
#define MQNIC_APP_ETH_SRC_MAC_LO_REG          0x0028 /* MAC lower 4 bytes */
#define MQNIC_APP_ETH_TYPE_REG                0x002C

// IPv4 header 12
#define MQNIC_APP_IP_VERSION_REG              0x0030
#define MQNIC_APP_IP_VERSION_MASK             GENMASK(3, 0)
#define MQNIC_APP_IP_HDR_LEN_REG              0x0034
#define MQNIC_APP_IP_HDR_LEN_MASK             GENMASK(3, 0)
#define MQNIC_APP_IP_TOS_REG                  0x0038
#define MQNIC_APP_IP_TOS_MASK                 GENMASK(7, 0)
#define MQNIC_APP_IP_TOTAL_LEN_REG            0x003C
#define MQNIC_APP_IP_ID_REG                   0x0040
#define MQNIC_APP_IP_FLAGS_REG                0x0044
#define MQNIC_APP_IP_FLAGS_MASK               GENMASK(2, 0)
#define MQNIC_APP_IP_FRAG_OFF_REG             0x0048
#define MQNIC_APP_IP_FRAG_OFF_MASK            GENMASK(12, 0)
#define MQNIC_APP_IP_TTL_REG                  0x004C
#define MQNIC_APP_IP_TTL_MASK                 GENMASK(7, 0)
#define MQNIC_APP_IP_PROTOCOL_REG             0x0050
#define MQNIC_APP_IP_PROTOCOL_MASK            GENMASK(7, 0)
#define MQNIC_APP_IP_HDR_CKSUM_REG            0x0054
#define MQNIC_APP_IP_SRC_ADDR_REG             0x0058
#define MQNIC_APP_IP_DEST_ADDR_REG            0x005C

// UDP header 24
#define MQNIC_APP_UDP_SRC_PORT_REG            0x0060
#define MQNIC_APP_UDP_DEST_PORT_REG           0x0064
#define MQNIC_APP_UDP_LENGTH_REG              0x0068
#define MQNIC_APP_UDP_CKSUM_REG               0x006C

// SWITCH
#define MQNIC_APP_SWITCH_REG                  0x0070
#define MQNIC_APP_SWITCH_BIT                  BIT(0)

#define MQNIC_APP_16BIT_MASK                  GENMASK(15, 0)


struct mqnic_app_aux_dev {
	struct device *dev;
	u8 __iomem *app_hw_addr;
	struct regmap *regs;
};

static const struct regmap_config mqnic_app_aux_regmap_cfg = {
	.name = "mqnic_app_aux",
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
};

enum mqnic_app_attribute_id {
	MQNIC_APP_ATTR_VERSION,
	MQNIC_APP_ATTR_SCRATCH,
	MQNIC_APP_ATTR_COUNTER_START,
	MQNIC_APP_ATTR_COUNTER_STOP,
	MQNIC_APP_ATTR_COUNTER_CLEAR,
	MQNIC_APP_ATTR_COUNTER_VALUE,
	MQNIC_APP_ATTR_START_GEN,
	MQNIC_APP_ATTR_PKT_SIZE,
	MQNIC_APP_ATTR_ETH_DEST_MAC_HI,
	MQNIC_APP_ATTR_ETH_DEST_MAC_LO,
	MQNIC_APP_ATTR_ETH_SRC_MAC_HI,
	MQNIC_APP_ATTR_ETH_SRC_MAC_LO,
	MQNIC_APP_ATTR_ETH_TYPE,
	MQNIC_APP_ATTR_IP_VERSION,
	MQNIC_APP_ATTR_IP_HDR_LEN,
	MQNIC_APP_ATTR_IP_TOS,
	MQNIC_APP_ATTR_IP_TOTAL_LEN,
	MQNIC_APP_ATTR_IP_ID,
	MQNIC_APP_ATTR_IP_FLAGS,
	MQNIC_APP_ATTR_IP_FRAG_OFF,
	MQNIC_APP_ATTR_IP_TTL,
	MQNIC_APP_ATTR_IP_PROTOCOL,
	MQNIC_APP_ATTR_IP_HDR_CKSUM,
	MQNIC_APP_ATTR_IP_SRC_ADDR,
	MQNIC_APP_ATTR_IP_DEST_ADDR,
	MQNIC_APP_ATTR_UDP_SRC_PORT,
	MQNIC_APP_ATTR_UDP_DEST_PORT,
	MQNIC_APP_ATTR_UDP_LENGTH,
	MQNIC_APP_ATTR_UDP_CKSUM,
	MQNIC_APP_ATTR_SWITCH,
};

struct mqnic_app_attribute {
	enum mqnic_app_attribute_id id;
	struct device_attribute attr;
	u8 name[32];
};
#define to_mqnic_app_attribute(x) container_of(x, struct mqnic_app_attribute, attr)

static ssize_t mqnic_app_auxiliary_show(struct device *dev,
				struct device_attribute *dev_attr, char *buf)
{
	const struct mqnic_app_attribute *attr = to_mqnic_app_attribute(dev_attr);
	struct mqnic_app_aux_dev *mqnic_app_adev = auxiliary_get_drvdata(to_auxiliary_dev(dev));
	u32 data;
	int ret;

	switch (attr->id) {
	case MQNIC_APP_ATTR_VERSION:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_VERSION_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "0x%08x\n", data);
	case MQNIC_APP_ATTR_SCRATCH:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_SCRATCH_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "0x%08x\n", data);
	case MQNIC_APP_ATTR_COUNTER_START:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_CONTROL_CNT_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%u\n", MQNIC_APP_CONTROL_CNT_START & data ? 1 : 0);
	case MQNIC_APP_ATTR_COUNTER_STOP:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_CONTROL_CNT_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%u\n", MQNIC_APP_CONTROL_CNT_STOP & data ? 1 : 0);
	case MQNIC_APP_ATTR_COUNTER_CLEAR:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_CLEAR_CNT_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%u\n", MQNIC_APP_CLEAR_CNT_BIT & data ? 1 : 0);
	case MQNIC_APP_ATTR_COUNTER_VALUE:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_CNT_VALUE_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%u\n", data);
	case MQNIC_APP_ATTR_START_GEN:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_START_GEN_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%u\n", data);
	case MQNIC_APP_ATTR_PKT_SIZE:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_PKT_SIZE_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%u\n", data);
	case MQNIC_APP_ATTR_ETH_DEST_MAC_HI:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_ETH_DEST_MAC_HI_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "0x%04lx\n", FIELD_GET(MQNIC_APP_16BIT_MASK, data));
	case MQNIC_APP_ATTR_ETH_DEST_MAC_LO:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_ETH_DEST_MAC_LO_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "0x%08x\n", data);
	case MQNIC_APP_ATTR_ETH_SRC_MAC_HI:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_ETH_SRC_MAC_HI_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "0x%04lx\n", FIELD_GET(MQNIC_APP_16BIT_MASK, data));
	case MQNIC_APP_ATTR_ETH_SRC_MAC_LO:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_ETH_SRC_MAC_LO_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "0x%08x\n", data);
	case MQNIC_APP_ATTR_ETH_TYPE:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_ETH_TYPE_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "0x%04lx\n", FIELD_GET(MQNIC_APP_16BIT_MASK, data));
	case MQNIC_APP_ATTR_IP_VERSION:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_IP_VERSION_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "0x%01lx\n", FIELD_GET(MQNIC_APP_IP_VERSION_MASK, data));
	case MQNIC_APP_ATTR_IP_HDR_LEN:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_IP_HDR_LEN_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "0x%01lx\n", FIELD_GET(MQNIC_APP_IP_HDR_LEN_MASK, data));
	case MQNIC_APP_ATTR_IP_TOS:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_IP_TOS_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "0x%02lx\n", FIELD_GET(MQNIC_APP_IP_TOS_MASK, data));
	case MQNIC_APP_ATTR_IP_TOTAL_LEN:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_IP_TOTAL_LEN_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "0x%04lx\n", FIELD_GET(MQNIC_APP_16BIT_MASK, data));
	case MQNIC_APP_ATTR_IP_ID:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_IP_ID_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "0x%04lx\n", FIELD_GET(MQNIC_APP_16BIT_MASK, data));
	case MQNIC_APP_ATTR_IP_FLAGS:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_IP_FLAGS_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "0x%01lx\n", FIELD_GET(MQNIC_APP_IP_FLAGS_MASK, data));
	case MQNIC_APP_ATTR_IP_FRAG_OFF:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_IP_FRAG_OFF_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "0x%04lx\n", FIELD_GET(MQNIC_APP_IP_FRAG_OFF_MASK, data));
	case MQNIC_APP_ATTR_IP_TTL:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_IP_TTL_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "0x%02lx\n", FIELD_GET(MQNIC_APP_IP_TTL_MASK, data));
	case MQNIC_APP_ATTR_IP_PROTOCOL:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_IP_PROTOCOL_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "0x%02lx\n", FIELD_GET(MQNIC_APP_IP_PROTOCOL_MASK, data));
	case MQNIC_APP_ATTR_IP_HDR_CKSUM:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_IP_HDR_CKSUM_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "0x%04lx\n", FIELD_GET(MQNIC_APP_16BIT_MASK, data));
	case MQNIC_APP_ATTR_IP_SRC_ADDR:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_IP_SRC_ADDR_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "0x%08x\n", data);
	case MQNIC_APP_ATTR_IP_DEST_ADDR:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_IP_DEST_ADDR_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "0x%08x\n", data);
	case MQNIC_APP_ATTR_UDP_SRC_PORT:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_UDP_SRC_PORT_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "0x%04lx\n", FIELD_GET(MQNIC_APP_16BIT_MASK, data));
	case MQNIC_APP_ATTR_UDP_DEST_PORT:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_UDP_DEST_PORT_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "0x%04lx\n", FIELD_GET(MQNIC_APP_16BIT_MASK, data));
	case MQNIC_APP_ATTR_UDP_LENGTH:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_UDP_LENGTH_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "0x%04lx\n", FIELD_GET(MQNIC_APP_16BIT_MASK, data));
	case MQNIC_APP_ATTR_UDP_CKSUM:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_UDP_CKSUM_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "0x%04lx\n", FIELD_GET(MQNIC_APP_16BIT_MASK, data));
	case MQNIC_APP_ATTR_SWITCH:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_SWITCH_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%u\n", MQNIC_APP_SWITCH_BIT & data ? 1 : 0);
	default:
		return -EINVAL;
	}
}

static ssize_t mqnic_app_auxiliary_write(struct mqnic_app_aux_dev *mqnic_app_adev,
				 const struct mqnic_app_attribute *attr,
				 const char *buf)
{
	u32 data;
	int ret;

	switch (attr->id) {
	case MQNIC_APP_ATTR_SCRATCH:
		ret = kstrtou32(buf, 0, &data);
		if (ret)
			return ret;
		return regmap_write(mqnic_app_adev->regs, MQNIC_APP_SCRATCH_REG, data);
	case MQNIC_APP_ATTR_COUNTER_START:
		ret = kstrtou32(buf, 0, &data);
		if (ret)
			return ret;
		//todo: maybe find better implementation to write value for 1 bit
		return regmap_write(mqnic_app_adev->regs, MQNIC_APP_CONTROL_CNT_REG,
				    MQNIC_APP_CONTROL_CNT_START * !!data);
	case MQNIC_APP_ATTR_COUNTER_STOP:
		ret = kstrtou32(buf, 0, &data);
		if (ret)
			return ret;
		return regmap_write(mqnic_app_adev->regs, MQNIC_APP_CONTROL_CNT_REG,
				    !!data << 1);
	case MQNIC_APP_ATTR_COUNTER_CLEAR:
		ret = kstrtou32(buf, 0, &data);
		if (ret)
			return ret;
		return regmap_write(mqnic_app_adev->regs, MQNIC_APP_CLEAR_CNT_REG,
				    MQNIC_APP_CLEAR_CNT_BIT * !!data);
	case MQNIC_APP_ATTR_START_GEN:
		ret = kstrtou32(buf, 0, &data);
		if (ret)
			return ret;
		return regmap_write(mqnic_app_adev->regs, MQNIC_APP_START_GEN_REG,
				    MQNIC_APP_START_GEN_BIT * !!data);
	case MQNIC_APP_ATTR_PKT_SIZE:
		ret = kstrtou32(buf, 0, &data);
		if (ret)
			return ret;
		return regmap_write(mqnic_app_adev->regs, MQNIC_APP_PKT_SIZE_REG, data);
	case MQNIC_APP_ATTR_ETH_DEST_MAC_HI:
		ret = kstrtou32(buf, 0, &data);
		if (ret)
			return ret;
		return regmap_write(mqnic_app_adev->regs, MQNIC_APP_ETH_DEST_MAC_HI_REG, FIELD_GET(MQNIC_APP_16BIT_MASK, data));
	case MQNIC_APP_ATTR_ETH_DEST_MAC_LO:
		ret = kstrtou32(buf, 0, &data);
		if (ret)
			return ret;
		return regmap_write(mqnic_app_adev->regs, MQNIC_APP_ETH_DEST_MAC_LO_REG, data);
	case MQNIC_APP_ATTR_ETH_SRC_MAC_HI:
		ret = kstrtou32(buf, 0, &data);
		if (ret)
			return ret;
		return regmap_write(mqnic_app_adev->regs, MQNIC_APP_ETH_SRC_MAC_HI_REG, FIELD_GET(MQNIC_APP_16BIT_MASK, data));
	case MQNIC_APP_ATTR_ETH_SRC_MAC_LO:
		ret = kstrtou32(buf, 0, &data);
		if (ret)
			return ret;
		return regmap_write(mqnic_app_adev->regs, MQNIC_APP_ETH_SRC_MAC_LO_REG, data);
	case MQNIC_APP_ATTR_ETH_TYPE:
		ret = kstrtou32(buf, 0, &data);
		if (ret)
			return ret;
		return regmap_write(mqnic_app_adev->regs, MQNIC_APP_ETH_TYPE_REG, FIELD_GET(MQNIC_APP_16BIT_MASK, data));
	case MQNIC_APP_ATTR_IP_VERSION:
		ret = kstrtou32(buf, 0, &data);
		if (ret)
			return ret;
		return regmap_write(mqnic_app_adev->regs, MQNIC_APP_IP_VERSION_REG, FIELD_GET(MQNIC_APP_IP_VERSION_MASK, data));
	case MQNIC_APP_ATTR_IP_HDR_LEN:
		ret = kstrtou32(buf, 0, &data);
		if (ret)
			return ret;
		return regmap_write(mqnic_app_adev->regs, MQNIC_APP_IP_HDR_LEN_REG, FIELD_GET(MQNIC_APP_IP_HDR_LEN_MASK, data));
	case MQNIC_APP_ATTR_IP_TOS:
		ret = kstrtou32(buf, 0, &data);
		if (ret)
			return ret;
		return regmap_write(mqnic_app_adev->regs, MQNIC_APP_IP_TOS_REG, FIELD_GET(MQNIC_APP_IP_TOS_MASK, data));
	case MQNIC_APP_ATTR_IP_TOTAL_LEN:
		ret = kstrtou32(buf, 0, &data);
		if (ret)
			return ret;
		return regmap_write(mqnic_app_adev->regs, MQNIC_APP_IP_TOTAL_LEN_REG, FIELD_GET(MQNIC_APP_16BIT_MASK, data));
	case MQNIC_APP_ATTR_IP_ID:
		ret = kstrtou32(buf, 0, &data);
		if (ret)
			return ret;
		return regmap_write(mqnic_app_adev->regs, MQNIC_APP_IP_ID_REG, FIELD_GET(MQNIC_APP_16BIT_MASK, data));
	case MQNIC_APP_ATTR_IP_FLAGS:
		ret = kstrtou32(buf, 0, &data);
		if (ret)
			return ret;
		return regmap_write(mqnic_app_adev->regs, MQNIC_APP_IP_FLAGS_REG, FIELD_GET(MQNIC_APP_IP_FLAGS_MASK, data));
	case MQNIC_APP_ATTR_IP_FRAG_OFF:
		ret = kstrtou32(buf, 0, &data);
		if (ret)
			return ret;
		return regmap_write(mqnic_app_adev->regs, MQNIC_APP_IP_FRAG_OFF_REG, FIELD_GET(MQNIC_APP_IP_FRAG_OFF_MASK, data));
	case MQNIC_APP_ATTR_IP_TTL:
		ret = kstrtou32(buf, 0, &data);
		if (ret)
			return ret;
		return regmap_write(mqnic_app_adev->regs, MQNIC_APP_IP_TTL_REG, FIELD_GET(MQNIC_APP_IP_TTL_MASK, data));
	case MQNIC_APP_ATTR_IP_PROTOCOL:
		ret = kstrtou32(buf, 0, &data);
		if (ret)
			return ret;
		return regmap_write(mqnic_app_adev->regs, MQNIC_APP_IP_PROTOCOL_REG, FIELD_GET(MQNIC_APP_IP_PROTOCOL_MASK, data));
	case MQNIC_APP_ATTR_IP_HDR_CKSUM:
		ret = kstrtou32(buf, 0, &data);
		if (ret)
			return ret;
		return regmap_write(mqnic_app_adev->regs, MQNIC_APP_IP_HDR_CKSUM_REG, FIELD_GET(MQNIC_APP_16BIT_MASK, data));
	case MQNIC_APP_ATTR_IP_SRC_ADDR:
		ret = kstrtou32(buf, 0, &data);
		if (ret)
			return ret;
		return regmap_write(mqnic_app_adev->regs, MQNIC_APP_IP_SRC_ADDR_REG, data);
	case MQNIC_APP_ATTR_IP_DEST_ADDR:
		ret = kstrtou32(buf, 0, &data);
		if (ret)
			return ret;
		return regmap_write(mqnic_app_adev->regs, MQNIC_APP_IP_DEST_ADDR_REG, data);
	case MQNIC_APP_ATTR_UDP_SRC_PORT:
		ret = kstrtou32(buf, 0, &data);
		if (ret)
			return ret;
		return regmap_write(mqnic_app_adev->regs, MQNIC_APP_UDP_SRC_PORT_REG, FIELD_GET(MQNIC_APP_16BIT_MASK, data));
	case MQNIC_APP_ATTR_UDP_DEST_PORT:
		ret = kstrtou32(buf, 0, &data);
		if (ret)
			return ret;
		return regmap_write(mqnic_app_adev->regs, MQNIC_APP_UDP_DEST_PORT_REG, FIELD_GET(MQNIC_APP_16BIT_MASK, data));
	case MQNIC_APP_ATTR_UDP_LENGTH:
		ret = kstrtou32(buf, 0, &data);
		if (ret)
			return ret;
		return regmap_write(mqnic_app_adev->regs, MQNIC_APP_UDP_LENGTH_REG, FIELD_GET(MQNIC_APP_16BIT_MASK, data));
	case MQNIC_APP_ATTR_UDP_CKSUM:
		ret = kstrtou32(buf, 0, &data);
		if (ret)
			return ret;
		return regmap_write(mqnic_app_adev->regs, MQNIC_APP_UDP_CKSUM_REG, FIELD_GET(MQNIC_APP_16BIT_MASK, data));
	case MQNIC_APP_ATTR_SWITCH:
		ret = kstrtou32(buf, 0, &data);
		if (ret)
			return ret;
		return regmap_write(mqnic_app_adev->regs, MQNIC_APP_SWITCH_REG,
				    MQNIC_APP_SWITCH_BIT * !!data);
	default:
		return -EINVAL;
	}
}

static ssize_t mqnic_app_auxiliary_store(struct device *dev,
				 struct device_attribute *dev_attr,
				 const char *buf, size_t count)
{
	struct mqnic_app_aux_dev *mqnic_app_adev = auxiliary_get_drvdata(to_auxiliary_dev(dev));
	const struct mqnic_app_attribute *attr = to_mqnic_app_attribute(dev_attr);

	return mqnic_app_auxiliary_write(mqnic_app_adev, attr, buf) ?: count;
}

//todo: channel is removed, in the future is should be added.
#define __MQNIC_APP_ATTR(_name, _id, _mode)                         \
	{								\
		.attr = __ATTR(_name, _mode, mqnic_app_auxiliary_show,          \
				mqnic_app_auxiliary_store),                     \
		.id = _id,                                              \
	}

#define MQNIC_APP_ATTR(_name, _id, _mode)                                     \
	struct mqnic_app_attribute dev_attr_##_name =                 \
		__MQNIC_APP_ATTR(_name, _id, _mode)                        \

static const MQNIC_APP_ATTR(version, MQNIC_APP_ATTR_VERSION, 0444);
static const MQNIC_APP_ATTR(scratch, MQNIC_APP_ATTR_SCRATCH, 0644);
static const MQNIC_APP_ATTR(counter_start, MQNIC_APP_ATTR_COUNTER_START, 0644);
static const MQNIC_APP_ATTR(counter_stop, MQNIC_APP_ATTR_COUNTER_STOP, 0644);
static const MQNIC_APP_ATTR(counter_clear, MQNIC_APP_ATTR_COUNTER_CLEAR, 0644);
static const MQNIC_APP_ATTR(counter_value, MQNIC_APP_ATTR_COUNTER_VALUE, 0444);
static const MQNIC_APP_ATTR(start_generator, MQNIC_APP_ATTR_START_GEN, 0644);
static const MQNIC_APP_ATTR(packet_size, MQNIC_APP_ATTR_PKT_SIZE, 0644);
static const MQNIC_APP_ATTR(eth_dest_mac_2B_hi, MQNIC_APP_ATTR_ETH_DEST_MAC_HI, 0644);
static const MQNIC_APP_ATTR(eth_dest_mac_4B_lo, MQNIC_APP_ATTR_ETH_DEST_MAC_LO, 0644);
static const MQNIC_APP_ATTR(eth_src_mac_2B_hi, MQNIC_APP_ATTR_ETH_SRC_MAC_HI, 0644);
static const MQNIC_APP_ATTR(eth_src_mac_4B_lo, MQNIC_APP_ATTR_ETH_SRC_MAC_LO, 0644);
static const MQNIC_APP_ATTR(eth_type, MQNIC_APP_ATTR_ETH_TYPE, 0644);
static const MQNIC_APP_ATTR(ip_version, MQNIC_APP_ATTR_IP_VERSION, 0644);
static const MQNIC_APP_ATTR(ip_header_length, MQNIC_APP_ATTR_IP_HDR_LEN, 0644);
static const MQNIC_APP_ATTR(ip_type_of_service, MQNIC_APP_ATTR_IP_TOS, 0644);
static const MQNIC_APP_ATTR(ip_total_length, MQNIC_APP_ATTR_IP_TOTAL_LEN, 0644);
static const MQNIC_APP_ATTR(ip_identification, MQNIC_APP_ATTR_IP_ID, 0644);
static const MQNIC_APP_ATTR(ip_flags, MQNIC_APP_ATTR_IP_FLAGS, 0644);
static const MQNIC_APP_ATTR(ip_fragment_offset, MQNIC_APP_ATTR_IP_FRAG_OFF, 0644);
static const MQNIC_APP_ATTR(ip_time_to_live, MQNIC_APP_ATTR_IP_TTL, 0644);
static const MQNIC_APP_ATTR(ip_protocol, MQNIC_APP_ATTR_IP_PROTOCOL, 0644);
static const MQNIC_APP_ATTR(ip_header_checksum, MQNIC_APP_ATTR_IP_HDR_CKSUM, 0644);
static const MQNIC_APP_ATTR(ip_src_addr, MQNIC_APP_ATTR_IP_SRC_ADDR, 0644);
static const MQNIC_APP_ATTR(ip_dest_addr, MQNIC_APP_ATTR_IP_DEST_ADDR, 0644);
static const MQNIC_APP_ATTR(udp_src_port, MQNIC_APP_ATTR_UDP_SRC_PORT, 0644);
static const MQNIC_APP_ATTR(udp_dest_port, MQNIC_APP_ATTR_UDP_DEST_PORT, 0644);
static const MQNIC_APP_ATTR(udp_length, MQNIC_APP_ATTR_UDP_LENGTH, 0644);
static const MQNIC_APP_ATTR(udp_checksum, MQNIC_APP_ATTR_UDP_CKSUM, 0644);
static const MQNIC_APP_ATTR(datapath_switch, MQNIC_APP_ATTR_SWITCH, 0644);

static const struct attribute *mqnic_app_attributes[] = {
	&dev_attr_version.attr.attr,
	&dev_attr_scratch.attr.attr,
	&dev_attr_counter_start.attr.attr,
	&dev_attr_counter_stop.attr.attr,
	&dev_attr_counter_clear.attr.attr,
	&dev_attr_counter_value.attr.attr,
	&dev_attr_start_generator.attr.attr,
	&dev_attr_packet_size.attr.attr,
	&dev_attr_eth_dest_mac_2B_hi.attr.attr,
	&dev_attr_eth_dest_mac_4B_lo.attr.attr,
	&dev_attr_eth_src_mac_2B_hi.attr.attr,
	&dev_attr_eth_src_mac_4B_lo.attr.attr,
	&dev_attr_eth_type.attr.attr,
	&dev_attr_ip_version.attr.attr,
	&dev_attr_ip_header_length.attr.attr,
	&dev_attr_ip_type_of_service.attr.attr,
	&dev_attr_ip_total_length.attr.attr,
	&dev_attr_ip_identification.attr.attr,
	&dev_attr_ip_flags.attr.attr,
	&dev_attr_ip_fragment_offset.attr.attr,
	&dev_attr_ip_time_to_live.attr.attr,
	&dev_attr_ip_protocol.attr.attr,
	&dev_attr_ip_header_checksum.attr.attr,
	&dev_attr_ip_src_addr.attr.attr,
	&dev_attr_ip_dest_addr.attr.attr,
	&dev_attr_udp_src_port.attr.attr,
	&dev_attr_udp_dest_port.attr.attr,
	&dev_attr_udp_length.attr.attr,
	&dev_attr_udp_checksum.attr.attr,
	&dev_attr_datapath_switch.attr.attr,
	/* NOT TERMINATED */
};

static int mqnic_app_auxiliary_init_sysfs(struct device *dev)
{
	size_t attribute_count = ARRAY_SIZE(mqnic_app_attributes);
	struct attribute **mqnic_app_attrs;
	struct attribute_group *attr_group;

	mqnic_app_attrs = devm_kcalloc(dev, attribute_count,
				 sizeof(*mqnic_app_attrs), GFP_KERNEL);
	if (!mqnic_app_attrs)
		return -ENOMEM;

	memcpy(mqnic_app_attrs, mqnic_app_attributes,
	       sizeof(mqnic_app_attributes));

	attr_group = devm_kzalloc(dev, sizeof(attr_group), GFP_KERNEL);
	if (!attr_group)
		return -ENOMEM;

	attr_group->attrs = mqnic_app_attrs;

	return devm_device_add_group(dev, attr_group);
}

static int mqnic_app_auxiliary_probe(struct auxiliary_device *aux_dev, const struct auxiliary_device_id *id)
{
	struct mqnic_adev *mqnic_adev = container_of(&aux_dev->dev, struct mqnic_adev, adev.dev);
	struct mqnic_app_aux_dev *mqnic_app_adev;
	unsigned int version;
	int ret;

	mqnic_app_adev = devm_kzalloc(&aux_dev->dev, sizeof(*mqnic_app_adev), GFP_KERNEL);
	if (!mqnic_app_adev)
		return -ENOMEM;

	mqnic_app_adev->app_hw_addr = mqnic_adev->mdev->app_hw_addr;
	mqnic_app_adev->dev = &aux_dev->dev;

	mqnic_app_adev->regs = devm_regmap_init_mmio(&aux_dev->dev, mqnic_app_adev->app_hw_addr,
					 &mqnic_app_aux_regmap_cfg);
	if (IS_ERR(mqnic_app_adev->regs))
		return PTR_ERR(mqnic_app_adev->regs);

	auxiliary_set_drvdata(aux_dev, mqnic_app_adev);

	ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_VERSION_REG, &version);
	if (ret)
		return ret;

	dev_info(&aux_dev->dev, "MQNIC app version: %x\n", version);
	udelay(2);

	ret = mqnic_app_auxiliary_init_sysfs(&aux_dev->dev);
	if (ret)
		return dev_err_probe(&aux_dev->dev, ret, "Failed to init sysfs, aborting ...\n");

	return 0;
}

static void mqnic_app_auxiliary_remove(struct auxiliary_device *aux_dev)
{
	//todo: make sure to use kfree for allocated memory
	dev_info(&aux_dev->dev, "MQNIC Auxiliary Bus: Device Removed\n");
}

static const struct auxiliary_device_id mqnic_app_auxiliary_id_table[] = {
	{ .name = "mqnic.app_12340001" },
	{},
};

MODULE_DEVICE_TABLE(auxiliary, mqnic_app_auxiliary_id_table);

static struct auxiliary_driver mqnic_app_auxiliary_driver = {
	.name = "mqnic_app_driver",
	.probe = mqnic_app_auxiliary_probe,
	.remove = mqnic_app_auxiliary_remove,
	.id_table = mqnic_app_auxiliary_id_table,
};

module_auxiliary_driver(mqnic_app_auxiliary_driver);

MODULE_DESCRIPTION("Corundum MQNIC Application Auxiliary driver");
MODULE_AUTHOR("Eliza Balas <eliza.balas@analog.com>");
MODULE_LICENSE("GPL");
