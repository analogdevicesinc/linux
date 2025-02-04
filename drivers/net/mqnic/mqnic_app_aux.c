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
#define MQNIC_APP_VERSION_REG                 0x00
#define MQNIC_APP_SCRATCH_REG                 0x04
#define MQNIC_APP_START_CNT_REG               0x08
#define MQNIC_APP_START_CNT_BIT               BIT(0)
#define MQNIC_APP_CNT_VALUE_REG               0x10

// Data generator
#define MQNIC_APP_START_APP_REG               0x14
#define MQNIC_APP_START_APP_BIT               BIT(0)

// Packetizer
#define MQNIC_APP_PKT_SIZE_REG                0x18 /* packet_size NOTE: THIS WILL BE OBSOLETE WITH SAMPLE COUNT PER CHANNEL*/

// Ethernet header 7
#define MQNIC_APP_ETH_DEST_MAC_MSB_REG        0x1C /* MAC higher 2 bytes */
#define MQNIC_APP_ETH_DEST_MAC_LSB_REG        0x20 /* MAC lower 4 bytes */
#define MQNIC_APP_ETH_SRC_MAC_MSB_REG         0x24 /* MAC higher 2 bytes */
#define MQNIC_APP_ETH_SRC_MAC_LSB_REG         0x28 /* MAC lower 4 bytes */
#define MQNIC_APP_ETH_TYPE_REG                0x2C
#define MQNIC_APP_ETH_TYPE_MASK               GENMASK(15, 0)

// IPv4 header
#define MQNIC_APP_IP_VERSION_REG              0x30
#define MQNIC_APP_IP_VERSION_MASK             GENMASK(3, 0)
#define MQNIC_APP_IP_HDR_LEN_REG              0x34
#define MQNIC_APP_IP_HDR_LEN_MASK             GENMASK(3, 0)
#define MQNIC_APP_IP_TOS_REG                  0x38
#define MQNIC_APP_IP_TOS_MASK                 GENMASK(7, 0)
#define MQNIC_APP_IP_TOTAL_LEN_REG            0x3C
#define MQNIC_APP_IP_TOTAL_LEN_MASK           GENMASK(15, 0)
#define MQNIC_APP_IP_ID_REG                   0x40
#define MQNIC_APP_IP_ID_MASK                  GENMASK(15, 0)
#define MQNIC_APP_IP_FLAGS_REG                0x44
#define MQNIC_APP_IP_FLAGS_MASK               GENMASK(2, 0)
#define MQNIC_APP_IP_FRAG_OFF_REG             0x48
#define MQNIC_APP_IP_FRAG_OFF_MASK            GENMASK(12, 0)
#define MQNIC_APP_IP_TTL_REG                  0x4C
#define MQNIC_APP_IP_TTL_MASK                 GENMASK(7, 0)
#define MQNIC_APP_IP_PROTOCOL_REG             0x50
#define MQNIC_APP_IP_PROTOCOL_MASK            GENMASK(7, 0)
#define MQNIC_APP_IP_HDR_CKSUM_REG            0x54
#define MQNIC_APP_IP_HDR_CKSUM_MASK           GENMASK(15, 0)
#define MQNIC_APP_IP_SRC_ADDR_REG             0x58
#define MQNIC_APP_IP_SRC_ADDR_MASK            GENMASK(31, 0)
#define MQNIC_APP_IP_DEST_ADDR_REG            0x5C
#define MQNIC_APP_IP_DEST_ADDR_MASK           GENMASK(31, 0)

// UDP header
#define MQNIC_APP_UDP_SRC_PORT_REG            0x60
#define MQNIC_APP_UDP_SRC_PORT_MASK           GENMASK(15, 0)
#define MQNIC_APP_UDP_DEST_PORT_REG           0x64
#define MQNIC_APP_UDP_DEST_PORT_MASK          GENMASK(15, 0)
#define MQNIC_APP_UDP_LENGTH_REG              0x68
#define MQNIC_APP_UDP_LENGTH_MASK             GENMASK(15, 0)
#define MQNIC_APP_UDP_CKSUM_REG               0x6C
#define MQNIC_APP_UDP_CKSUM_MASK              GENMASK(15, 0)

// BER Test
#define MQNIC_APP_BER_TEST_ENABLE_REG         0x70
#define MQNIC_APP_BER_TEST_ENABLE_BIT         BIT(0)
#define MQNIC_APP_BER_RESET_REG               0x74
#define MQNIC_APP_BER_RESET_BIT               BIT(0)
#define MQNIC_APP_BER_TOTAL_BITS_MSB_REG      0x78
#define MQNIC_APP_BER_TOTAL_BITS_LSB_REG      0x7C
#define MQNIC_APP_BER_ERROR_BITS_MSB_REG      0x80
#define MQNIC_APP_BER_ERROR_BITS_LSB_REG      0x84
#define MQNIC_APP_BER_OUT_OF_SYNC_REG         0x88
#define MQNIC_APP_BER_INSERT_BIT_ERROR_REG    0x8C
#define MQNIC_APP_BER_INSERT_BIT_ERROR_BIT    BIT(0)

//TEPORARY HERE. WILL REPLACE PACKET SIZE
#define MQNIC_APP_SAMPLE_COUNT_PER_CH_REG     0x90
#define MQNIC_APP_SAMPLE_COUNT_PER_CH_MASK    GENMASK(15, 0)


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
	MQNIC_APP_ATTR_COUNTER_VALUE,
	MQNIC_APP_ATTR_START_APP,
	MQNIC_APP_ATTR_PKT_SIZE,
	MQNIC_APP_ATTR_ETH_DEST_MAC,
	MQNIC_APP_ATTR_ETH_SRC_MAC,
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
	MQNIC_APP_ATTR_BER_TEST_EN,
	MQNIC_APP_ATTR_BER_RESET,
	MQNIC_APP_ATTR_BER_TOT_BITS,
	MQNIC_APP_ATTR_BER_ERR_BITS,
	MQNIC_APP_ATTR_BER_OUT_OF_SYNC,
	MQNIC_APP_ATTR_BER_INS_BIT_ERR,
	MQNIC_APP_ATTR_SAMPLE_COUNT_PER_CH,
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
	u64 data64;
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
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_START_CNT_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%u\n", MQNIC_APP_START_CNT_BIT & data ? 1 : 0);
	case MQNIC_APP_ATTR_COUNTER_VALUE:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_CNT_VALUE_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%u\n", data);
	case MQNIC_APP_ATTR_START_APP:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_START_APP_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%u\n", MQNIC_APP_START_APP_BIT & data ? 1 : 0);
	case MQNIC_APP_ATTR_PKT_SIZE:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_PKT_SIZE_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%u\n", data);
	case MQNIC_APP_ATTR_ETH_DEST_MAC:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_ETH_DEST_MAC_LSB_REG, &data);
		if (ret)
			return ret;
		data64 = 0xFFFFFFFF & data;
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_ETH_DEST_MAC_MSB_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "0x%012llx\n", ((u64)(0xFFFF & data) << 32) | data64);
	case MQNIC_APP_ATTR_ETH_SRC_MAC:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_ETH_SRC_MAC_LSB_REG, &data);
		if (ret)
			return ret;
		data64 = 0xFFFFFFFF & data;
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_ETH_SRC_MAC_MSB_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "0x%012llx\n", ((u64)(0xFFFF & data) << 32) | data64);
	case MQNIC_APP_ATTR_ETH_TYPE:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_ETH_TYPE_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "0x%04lx\n", FIELD_GET(MQNIC_APP_ETH_TYPE_MASK, data));
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
		return sysfs_emit(buf, "0x%04lx\n", FIELD_GET(MQNIC_APP_IP_TOTAL_LEN_MASK, data));
	case MQNIC_APP_ATTR_IP_ID:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_IP_ID_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "0x%04lx\n", FIELD_GET(MQNIC_APP_IP_ID_MASK, data));
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
		return sysfs_emit(buf, "0x%04lx\n", FIELD_GET(MQNIC_APP_IP_HDR_CKSUM_MASK, data));
	case MQNIC_APP_ATTR_IP_SRC_ADDR:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_IP_SRC_ADDR_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "0x%08lx\n", FIELD_GET(MQNIC_APP_IP_SRC_ADDR_MASK, data));
	case MQNIC_APP_ATTR_IP_DEST_ADDR:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_IP_DEST_ADDR_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "0x%08lx\n", FIELD_GET(MQNIC_APP_IP_DEST_ADDR_MASK, data));
	case MQNIC_APP_ATTR_UDP_SRC_PORT:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_UDP_SRC_PORT_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "0x%04lx\n", FIELD_GET(MQNIC_APP_UDP_SRC_PORT_MASK, data));
	case MQNIC_APP_ATTR_UDP_DEST_PORT:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_UDP_DEST_PORT_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "0x%04lx\n", FIELD_GET(MQNIC_APP_UDP_DEST_PORT_MASK, data));
	case MQNIC_APP_ATTR_UDP_LENGTH:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_UDP_LENGTH_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "0x%04lx\n", FIELD_GET(MQNIC_APP_UDP_LENGTH_MASK, data));
	case MQNIC_APP_ATTR_UDP_CKSUM:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_UDP_CKSUM_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "0x%04lx\n", FIELD_GET(MQNIC_APP_UDP_CKSUM_MASK, data));
	case MQNIC_APP_ATTR_BER_TEST_EN:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_BER_TEST_ENABLE_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%u\n", MQNIC_APP_BER_TEST_ENABLE_BIT & data ? 1 : 0);
	case MQNIC_APP_ATTR_BER_RESET:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_BER_RESET_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%u\n", MQNIC_APP_BER_RESET_BIT & data ? 1 : 0);
	case MQNIC_APP_ATTR_BER_TOT_BITS:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_BER_TOTAL_BITS_LSB_REG, &data);
		if (ret)
			return ret;
		data64 = 0xFFFFFFFF & data;
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_BER_TOTAL_BITS_MSB_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%llu\n", ((u64)data << 32) | data64);
	case MQNIC_APP_ATTR_BER_ERR_BITS:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_BER_ERROR_BITS_LSB_REG, &data);
		if (ret)
			return ret;
		data64 = 0xFFFFFFFF & data;
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_BER_ERROR_BITS_MSB_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%llu\n", ((u64)data << 32) | data64);
	case MQNIC_APP_ATTR_BER_OUT_OF_SYNC:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_BER_OUT_OF_SYNC_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%u\n", data);
	case MQNIC_APP_ATTR_BER_INS_BIT_ERR:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_BER_INSERT_BIT_ERROR_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%u\n", MQNIC_APP_BER_INSERT_BIT_ERROR_BIT & data ? 1 : 0);
	case MQNIC_APP_ATTR_SAMPLE_COUNT_PER_CH:
		ret = regmap_read(mqnic_app_adev->regs, MQNIC_APP_SAMPLE_COUNT_PER_CH_REG, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%u\n", data);
	default:
		return -EINVAL;
	}
}

static ssize_t mqnic_app_auxiliary_write(struct mqnic_app_aux_dev *mqnic_app_adev,
				 const struct mqnic_app_attribute *attr,
				 const char *buf)
{
	u64 data64;
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
		return regmap_write(mqnic_app_adev->regs, MQNIC_APP_START_CNT_REG,
				    MQNIC_APP_START_CNT_BIT * !!data);
	case MQNIC_APP_ATTR_START_APP:
		ret = kstrtou32(buf, 0, &data);
		if (ret)
			return ret;
		return regmap_write(mqnic_app_adev->regs, MQNIC_APP_START_APP_REG,
				    MQNIC_APP_START_APP_BIT * !!data);
	case MQNIC_APP_ATTR_PKT_SIZE:
		ret = kstrtou32(buf, 0, &data);
		if (ret)
			return ret;
		return regmap_write(mqnic_app_adev->regs, MQNIC_APP_PKT_SIZE_REG, data);
	case MQNIC_APP_ATTR_ETH_DEST_MAC:
		ret = kstrtou64(buf, 0, &data64);
		if (ret)
			return ret;
		ret = regmap_write(mqnic_app_adev->regs, MQNIC_APP_ETH_DEST_MAC_LSB_REG, 0xFFFFFFFF & data64);
		if (ret)
			return ret;
		return regmap_write(mqnic_app_adev->regs, MQNIC_APP_ETH_DEST_MAC_MSB_REG, 0xFFFF & (data64 >> 32));
	case MQNIC_APP_ATTR_ETH_SRC_MAC:
		ret = kstrtou64(buf, 0, &data64);
		if (ret)
			return ret;
		ret = regmap_write(mqnic_app_adev->regs, MQNIC_APP_ETH_SRC_MAC_LSB_REG, 0xFFFFFFFF & data64);
		if (ret)
			return ret;
		return regmap_write(mqnic_app_adev->regs, MQNIC_APP_ETH_SRC_MAC_MSB_REG, 0xFFFF & (data64 >> 32));
	case MQNIC_APP_ATTR_ETH_TYPE:
		ret = kstrtou32(buf, 0, &data);
		if (ret)
			return ret;
		return regmap_write(mqnic_app_adev->regs, MQNIC_APP_ETH_TYPE_REG, FIELD_GET(MQNIC_APP_ETH_TYPE_MASK, data));
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
	case MQNIC_APP_ATTR_IP_ID:
		ret = kstrtou32(buf, 0, &data);
		if (ret)
			return ret;
		return regmap_write(mqnic_app_adev->regs, MQNIC_APP_IP_ID_REG, FIELD_GET(MQNIC_APP_IP_ID_MASK, data));
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
	case MQNIC_APP_ATTR_IP_SRC_ADDR:
		ret = kstrtou32(buf, 0, &data);
		if (ret)
			return ret;
		return regmap_write(mqnic_app_adev->regs, MQNIC_APP_IP_SRC_ADDR_REG, FIELD_GET(MQNIC_APP_IP_SRC_ADDR_MASK, data));
	case MQNIC_APP_ATTR_IP_DEST_ADDR:
		ret = kstrtou32(buf, 0, &data);
		if (ret)
			return ret;
		return regmap_write(mqnic_app_adev->regs, MQNIC_APP_IP_DEST_ADDR_REG, FIELD_GET(MQNIC_APP_IP_DEST_ADDR_MASK, data));
	case MQNIC_APP_ATTR_UDP_SRC_PORT:
		ret = kstrtou32(buf, 0, &data);
		if (ret)
			return ret;
		return regmap_write(mqnic_app_adev->regs,MQNIC_APP_UDP_SRC_PORT_REG, FIELD_GET(MQNIC_APP_UDP_SRC_PORT_MASK, data));
	case MQNIC_APP_ATTR_UDP_DEST_PORT:
		ret = kstrtou32(buf, 0, &data);
		if (ret)
			return ret;
		return regmap_write(mqnic_app_adev->regs, MQNIC_APP_UDP_DEST_PORT_REG, FIELD_GET(MQNIC_APP_UDP_DEST_PORT_MASK, data));
	case MQNIC_APP_ATTR_UDP_CKSUM:
		ret = kstrtou32(buf, 0, &data);
		if (ret)
			return ret;
		return regmap_write(mqnic_app_adev->regs, MQNIC_APP_UDP_CKSUM_REG, FIELD_GET(MQNIC_APP_UDP_CKSUM_MASK, data));
	case MQNIC_APP_ATTR_BER_TEST_EN:
		ret = kstrtou32(buf, 0, &data);
		if (ret)
			return ret;
		return regmap_write(mqnic_app_adev->regs, MQNIC_APP_BER_TEST_ENABLE_REG, FIELD_GET(MQNIC_APP_BER_TEST_ENABLE_BIT, data));
	case MQNIC_APP_ATTR_BER_RESET:
		ret = kstrtou32(buf, 0, &data);
		if (ret)
			return ret;
		return regmap_write(mqnic_app_adev->regs, MQNIC_APP_BER_RESET_REG, FIELD_GET(MQNIC_APP_BER_RESET_BIT, data));
	case MQNIC_APP_ATTR_BER_INS_BIT_ERR:
		ret = kstrtou32(buf, 0, &data);
		if (ret)
			return ret;
		return regmap_write(mqnic_app_adev->regs, MQNIC_APP_BER_INSERT_BIT_ERROR_REG, FIELD_GET(MQNIC_APP_BER_INSERT_BIT_ERROR_BIT, data));
	case MQNIC_APP_ATTR_SAMPLE_COUNT_PER_CH:
		ret = kstrtou32(buf, 0, &data);
		if (ret)
			return ret;
		return regmap_write(mqnic_app_adev->regs, MQNIC_APP_SAMPLE_COUNT_PER_CH_REG, FIELD_GET(MQNIC_APP_SAMPLE_COUNT_PER_CH_MASK, data));
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
static const MQNIC_APP_ATTR(counter_value, MQNIC_APP_ATTR_COUNTER_VALUE, 0444);
static const MQNIC_APP_ATTR(start_app, MQNIC_APP_ATTR_START_APP, 0644);
static const MQNIC_APP_ATTR(packet_size, MQNIC_APP_ATTR_PKT_SIZE, 0644);
static const MQNIC_APP_ATTR(eth_dest_mac, MQNIC_APP_ATTR_ETH_DEST_MAC, 0644);
static const MQNIC_APP_ATTR(eth_src_mac, MQNIC_APP_ATTR_ETH_SRC_MAC, 0644);
static const MQNIC_APP_ATTR(eth_type, MQNIC_APP_ATTR_ETH_TYPE, 0644);
static const MQNIC_APP_ATTR(ip_version, MQNIC_APP_ATTR_IP_VERSION, 0644);
static const MQNIC_APP_ATTR(ip_header_length, MQNIC_APP_ATTR_IP_HDR_LEN, 0644);
static const MQNIC_APP_ATTR(ip_type_of_service, MQNIC_APP_ATTR_IP_TOS, 0644);
static const MQNIC_APP_ATTR(ip_total_length, MQNIC_APP_ATTR_IP_TOTAL_LEN, 0444);
static const MQNIC_APP_ATTR(ip_identification, MQNIC_APP_ATTR_IP_ID, 0644);
static const MQNIC_APP_ATTR(ip_flags, MQNIC_APP_ATTR_IP_FLAGS, 0644);
static const MQNIC_APP_ATTR(ip_fragment_offset, MQNIC_APP_ATTR_IP_FRAG_OFF, 0644);
static const MQNIC_APP_ATTR(ip_time_to_live, MQNIC_APP_ATTR_IP_TTL, 0644);
static const MQNIC_APP_ATTR(ip_protocol, MQNIC_APP_ATTR_IP_PROTOCOL, 0644);
static const MQNIC_APP_ATTR(ip_header_checksum, MQNIC_APP_ATTR_IP_HDR_CKSUM, 0444);
static const MQNIC_APP_ATTR(ip_src_addr, MQNIC_APP_ATTR_IP_SRC_ADDR, 0644);
static const MQNIC_APP_ATTR(ip_dest_addr, MQNIC_APP_ATTR_IP_DEST_ADDR, 0644);
static const MQNIC_APP_ATTR(udp_src_port, MQNIC_APP_ATTR_UDP_SRC_PORT, 0644);
static const MQNIC_APP_ATTR(udp_dest_port, MQNIC_APP_ATTR_UDP_DEST_PORT, 0644);
static const MQNIC_APP_ATTR(udp_length, MQNIC_APP_ATTR_UDP_LENGTH, 0444);
static const MQNIC_APP_ATTR(udp_checksum, MQNIC_APP_ATTR_UDP_CKSUM, 0644);
static const MQNIC_APP_ATTR(ber_test_en, MQNIC_APP_ATTR_BER_TEST_EN, 0644);
static const MQNIC_APP_ATTR(ber_reset, MQNIC_APP_ATTR_BER_RESET, 0644);
static const MQNIC_APP_ATTR(ber_total_bits, MQNIC_APP_ATTR_BER_TOT_BITS, 0444);
static const MQNIC_APP_ATTR(ber_error_bits, MQNIC_APP_ATTR_BER_ERR_BITS, 0444);
static const MQNIC_APP_ATTR(ber_out_of_sync, MQNIC_APP_ATTR_BER_OUT_OF_SYNC, 0444);
static const MQNIC_APP_ATTR(ber_insert_bit_err, MQNIC_APP_ATTR_BER_INS_BIT_ERR, 0644);
static const MQNIC_APP_ATTR(sample_count_per_ch, MQNIC_APP_ATTR_SAMPLE_COUNT_PER_CH, 0644);

static const struct attribute *mqnic_app_attributes[] = {
	&dev_attr_version.attr.attr,
	&dev_attr_scratch.attr.attr,
	&dev_attr_counter_start.attr.attr,
	&dev_attr_counter_value.attr.attr,
	&dev_attr_start_app.attr.attr,
	&dev_attr_packet_size.attr.attr,
	&dev_attr_eth_dest_mac.attr.attr,
	&dev_attr_eth_src_mac.attr.attr,
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
	&dev_attr_ber_test_en.attr.attr,
	&dev_attr_ber_reset.attr.attr,
	&dev_attr_ber_total_bits.attr.attr,
	&dev_attr_ber_error_bits.attr.attr,
	&dev_attr_ber_out_of_sync.attr.attr,
	&dev_attr_ber_insert_bit_err.attr.attr,
	&dev_attr_sample_count_per_ch.attr.attr,
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

	attr_group = devm_kzalloc(dev, sizeof(*attr_group), GFP_KERNEL);
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
