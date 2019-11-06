// SPDX-License-Identifier: GPL-2.0
/*
 * Generic IIO access driver
 *
 * Copyright 2019 Analog Devices Inc.
 */

/* Possible register operations table:
 * +----------+--------+-----------+--------+-------+
 * | OP      | ADDR   |    MASK   |   VAL  |  TIME |
 * +------------------------------------------------+
 * |WAIT_US   |        |           |        |   X   |
 * +------------------------------------------------+
 * |READ_MASK |   X    |     X     |        |   X   |
 * +------------------------------------------------+
 * |WRITE_MASK|   X    |     X     |    X   |   X   |
 * +------------------------------------------------+
 * |WAIT_MASK |   X    |     X     |    X   |   X   |
 * +----------+--------+-----------+--------+-------+
 * Wait times are defined in microseconds.
 * WAIT_US -> wait X microseconds
 * READ_MASK -> wait X microseconds then read the value
 * WRITE_MASK -> wait X microseconds then write the value
 * WAIT_MASK -> wait X microseconds unitl register contains value
 */

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regmap.h>

#include "iio-regmap.h"

#define REG_OP_SIZE		20
#define NR_CONF_LINES		2
#define WAIT_POLL_TIME_US	1000

enum iio_regmap_op {
	READ_MASK,
	WAIT_MASK,
	WAIT_US,
	WRITE_MASK,
};

static char *iio_reg_op_map[] = {
	[WAIT_US] = "WAIT_US",
	[READ_MASK] = "READ_MASK",
	[WRITE_MASK] = "WRITE_MASK",
	[WAIT_MASK] = "WAIT_MASK"
};

struct iio_regmap {
	struct device *dev;
	struct regmap *regmap;
};

struct register_op {
	char op_str[REG_OP_SIZE];
	unsigned int reg_addr;
	unsigned int reg_mask;
	unsigned int reg_value;
	unsigned int wait_us;
};

int config_regmap(struct device *dev, struct regmap_config *regmap_cfg)
{
	u32 reg_bits;
	u32 val_bits;
	int ret;

	ret = device_property_read_u32(dev, "reg_bits", &reg_bits);
	if (ret < 0) {
		dev_err(dev, "Reading reg_bits property failed!\n");
		return -EINVAL;
	}

	ret = device_property_read_u32(dev, "val_bits", &val_bits);
	if (ret < 0) {
		dev_err(dev, "Reading val_bits property failed!\n");
		return -EINVAL;
	}

	regmap_cfg->reg_bits = reg_bits;
	regmap_cfg->val_bits = val_bits;

	return 0;
}
EXPORT_SYMBOL_GPL(config_regmap);

static int parse_op(struct device *dev, const char *reg_ops,
		    struct register_op *op, int line)
{
	int ret;

	ret = sscanf(reg_ops,
		     "%x,%x,%x,%u",
		     &op->reg_addr, &op->reg_mask,
		     &op->reg_value, &op->wait_us);

	if (ret == '\0' || ret != 4)
		dev_err(dev, "Invalid op format, line: %d.", line);
	return ret;
}

static int parse_read_op(struct device *dev, const char *reg_ops,
			 struct register_op *op, int line)
{
	int ret;

	ret = sscanf(reg_ops,
		     "%x,%x,,%u",
		     &op->reg_addr, &op->reg_mask, &op->wait_us);

	if (ret == '\0' || ret != 3)
		dev_err(dev, "Invalid read op format, line: %d.", line);
	return ret;
}

static int parse_wait_op(struct device *dev, const char *reg_ops,
			 struct register_op *op, int line)
{
	int ret;

	ret = sscanf(reg_ops, ",,,%u", &op->wait_us);
	if (ret == '\0' || ret != 1)
		dev_err(dev, "Invalid read op format, line: %d.", line);
	return ret;
}

static int run_write_op(struct device *dev, struct regmap *regmap,
			struct register_op *op)
{
	int ret;
	unsigned int register_value;

	udelay(op->wait_us);
	ret = regmap_read(regmap, op->reg_addr, &register_value);
	if (ret < 0) {
		dev_err(dev, "regmap_read failed, addr: %x", op->reg_addr);
		return ret;
	}

	register_value &= ~op->reg_mask;
	register_value |= op->reg_value;

	ret = regmap_write(regmap, op->reg_addr, op->reg_value);
	if (ret < 0) {
		dev_err(dev, "regmap_write failed, addr: %x", op->reg_addr);
		return ret;
	}

	return 0;
}

static int run_read_op(struct device *dev, struct regmap *regmap,
		       struct register_op *op, unsigned int *value)
{
	int ret;

	udelay(op->wait_us);
	ret = regmap_read(regmap, op->reg_addr, value);
	if (ret < 0) {
		dev_err(dev, "regmap_read failed, addr: %x", op->reg_addr);
		return ret;
	}

	*value &= op->reg_mask;

	return 0;
}

static int run_wait_mask_op(struct device *dev, struct regmap *regmap,
			    struct register_op *op)
{
	int ret;
	unsigned int value;
	unsigned int poll_times;

	poll_times = op->wait_us / WAIT_POLL_TIME_US + 1;
	udelay(op->wait_us % WAIT_POLL_TIME_US);
	while (poll_times != 0) {
		udelay(WAIT_POLL_TIME_US);
		ret = regmap_read(regmap, op->reg_addr, &value);
		if (ret < 0) {
			dev_err(dev, "run_wait_op failed");
			return ret;
		}

		poll_times--;
		value &= op->reg_mask;
		if (value == op->reg_value)
			break;

		if (poll_times == 0) {
			dev_err(dev,
				"Invalid reg [%x] value:[%x], expected[%x]",
				op->reg_addr, value, op->reg_value);
			return -1;
		}
	}
	return 0;
}

static int run_register_ops(struct device *dev, struct regmap *regmap,
			    struct register_op *reg_op, const char *reg_ops,
			    int line, int op_index)
{
	unsigned int reg_value;
	int ret = 0;

	switch (op_index) {
	case READ_MASK:
		ret = parse_read_op(dev, reg_ops, reg_op, line);
		if (ret < 0)
			return ret;
		ret = run_read_op(dev, regmap, reg_op, &reg_value);
		break;
	case WAIT_MASK:
		ret = parse_op(dev, reg_ops, reg_op, line);
		if (ret < 0)
			return ret;
		ret = run_wait_mask_op(dev, regmap, reg_op);
		break;
	case WAIT_US:
		ret = parse_wait_op(dev, reg_ops, reg_op, line);
		udelay(reg_op->wait_us);
		break;
	case WRITE_MASK:
		ret = parse_op(dev, reg_ops, reg_op, line);
		if (ret < 0)
			return ret;
		ret = run_write_op(dev, regmap, reg_op);
		break;
	default:
		dev_err(dev, "Invalid op: [%s],a t line: %d", reg_op->op_str,
			line);
		return -EINVAL;
	}

	if (ret < 0)
		return ret;

	return 0;
}

/* Each line represents a register operation.
 * OP,ADDRESS,MASK,VALUE,WAIT_US
 */
static int interpret_register_ops(struct device *dev, struct regmap *regmap,
				  const char *reg_ops)
{
	struct register_op reg_op;
	int ret;
	char *reg_op_end;
	int op_size = 0;
	int line = 0;
	int i = 0;
	int op_index;

	if (!reg_ops)
		return -EINVAL;

	while (*reg_ops != '\0') {
		reg_op_end = strchr(reg_ops, ',');
		op_size = reg_op_end - reg_ops;
		if (op_size + 1 > REG_OP_SIZE) {
			dev_err(dev, "Invalid op size.");
			return -EINVAL;
		}

		memset(reg_op.op_str, 0, REG_OP_SIZE * sizeof(char));
		memcpy(reg_op.op_str, reg_ops, op_size * sizeof(char));
		reg_ops = reg_op_end + 1;

		op_index = -1;
		for (i = READ_MASK; i <= WRITE_MASK; i++) {
			if (!strcmp(reg_op.op_str, iio_reg_op_map[i])) {
				op_index = i;
				break;
			}
		}

		ret = run_register_ops(dev, regmap, &reg_op, reg_ops,
				       line, op_index);
		if (ret < 0)
			return ret;

		reg_ops = strchr(reg_ops, '\n') + 1;
		line++;
	}

	return 0;
}

/* Retrieve from device node the firmware name then
 * read from firmware register operations,
 * allocate a copy of the firmware data
 * and return the start address of register operations within
 * the firmware.
 */
static const char *read_firmware(struct device *dev)
{
	char *reg_ops;
	int ret;
	const char *firmware_name;
	const char *data;
	const struct firmware *firmware;

	ret = device_property_read_string(dev, "firmware", &firmware_name);
	if (ret < 0) {
		dev_err(dev, "Firmware name property read failed!\n");
		return ERR_PTR(-EINVAL);
	}

	ret = request_firmware(&firmware, firmware_name, dev);
	if (ret < 0) {
		dev_err(dev, "request_firmware failed!\n");
		return ERR_PTR(-EINVAL);
	}

	reg_ops = devm_kzalloc(dev, (firmware->size + 2) * sizeof(char),
			       GFP_KERNEL);
	if (!reg_ops)
		return ERR_PTR(-ENOMEM);

	data = firmware->data;
	if (!data) {
		dev_err(dev, "Firmware data not loaded.");
		return ERR_PTR(-EINVAL);
	}
	memcpy(reg_ops, data, (firmware->size + 2) * sizeof(char));
	release_firmware(firmware);

	return reg_ops;
}

int iio_regmap_probe(struct device *dev, struct regmap *regmap,
		     const char *name)
{
	struct iio_dev *indio_dev;
	struct iio_regmap *st;
	const char *register_ops;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	dev_set_drvdata(dev, indio_dev);

	st->dev = dev;
	st->regmap = regmap;

	indio_dev->dev.parent = dev;
	indio_dev->name = name;
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret < 0) {
		dev_err(&indio_dev->dev, "iio-regmap device register failed\n");
		return ret;
	}

	register_ops = read_firmware(dev);
	if (IS_ERR(register_ops)) {
		dev_err(dev, "read_firmware failed!\n");
		return PTR_ERR(register_ops);
	}

	ret = interpret_register_ops(dev, regmap, register_ops);

	return ret;
}
EXPORT_SYMBOL_GPL(iio_regmap_probe);

MODULE_AUTHOR("Alexandru Tachici <alexandru.tachici@analog.com>");
MODULE_DESCRIPTION("Generic IIO access driver");
MODULE_LICENSE("GPL v2");
