// SPDX-License-Identifier: GPL-2.0
/*
 * Generic IIO access driver
 *
 * Copyright 2019 Analog Devices Inc.
 */

 /* Possible register operations table:
  * +----------+--------+-----------+--------+-------+
  * |    OP      |  ADDR  |    MASK   |  VAL  | TIME |
  * +------------------------------------------------+
  * |    READ    |    X   |          |        |      |
  * +------------------------------------------------+
  * | READ_MASK  |   X    |     X    |        |      |
  * +------------------------------------------------+
  * | WAIT_MASK  |   X    |     X    |    X   |   X  |
  * +------------------------------------------------+
  * |  WAIT_MS   |        |          |        |   X  |
  * +----------+--------+-----------+--------+-------+
  * |   WRITE    |   X    |          |    X   |      |
  * +----------+--------+-----------+--------+-------+
  * | WRITE_MASK |   X    |     X    |    X   |      |
  * +----------+--------+-----------+--------+-------+
  * Wait times are defined in milliseconds:
  * READ        -> read value at ADDR
  * READ_MASK   -> read value at ADDR with MASK
  * WAIT_MASK   -> wait TIME for ADDR value with MASK to become VAL
  * WAIT_MS     -> wait TIME milliseconds
  * WRTIE       -> write VAL at ADDR
  * WRITE_MASK  -> write VAL at ADDR with MASK
  */

#include <linux/iio/iio.h>
#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/regmap.h>

#include "iio-regmap.h"

#define REG_OP_STRLEN		20
#define WAIT_POLL_NR_TIMES	10

enum iio_regmap_opcode {
	IIO_REGMAP_READ,
	IIO_REGMAP_READ_MASK,
	IIO_REGMAP_WAIT_MASK,
	IIO_REGMAP_WAIT_MS,
	IIO_REGMAP_WRITE,
	IIO_REGMAP_WRITE_MASK,
};

static char *iio_reg_op_map[] = {
	[IIO_REGMAP_WAIT_MS] = "WAIT_MS",
	[IIO_REGMAP_READ_MASK] = "READ_MASK",
	[IIO_REGMAP_WRITE_MASK] = "WRITE_MASK",
	[IIO_REGMAP_WAIT_MASK] = "WAIT_MASK",
	[IIO_REGMAP_WRITE] = "WRITE",
	[IIO_REGMAP_READ] = "READ",
};

struct iio_regmap_op {
	enum iio_regmap_opcode	op;
	unsigned int		addr;
	unsigned int		mask;
	unsigned int		val;
	unsigned int		time;
	unsigned int		dbg;
};

struct iio_regmap {
	struct device *dev;
	struct regmap *regmap;
};

static const struct iio_info iio_regmap_info = {
};

int iio_regmap_read_config(struct device *dev, struct regmap_config *regmap_cfg)
{
	u32 reg_bits;
	u32 val_bits;
	int ret;

	ret = device_property_read_u32(dev, "reg-bits", &reg_bits);
	if (ret < 0) {
		dev_err(dev, "Reading reg-bits property failed!\n");
		return -EINVAL;
	}

	ret = device_property_read_u32(dev, "val-bits", &val_bits);
	if (ret < 0) {
		dev_err(dev, "Reading val-bits property failed!\n");
		return -EINVAL;
	}

	regmap_cfg->reg_bits = reg_bits;
	regmap_cfg->val_bits = val_bits;

	return 0;
}
EXPORT_SYMBOL_GPL(iio_regmap_read_config);

static struct iio_regmap_op *alloc_register_ops(struct device *dev,
						const char *fw_reg_ops)
{
	const char *c;
	unsigned int new_lines;
	struct iio_regmap_op *reg_ops;

	if (!fw_reg_ops)
		return NULL;

	new_lines = 0;
	for (c = fw_reg_ops; *c != '\0'; c++)
		if (*c == '\n')
			new_lines++;
	reg_ops = devm_kzalloc(dev, new_lines * sizeof(*reg_ops), GFP_KERNEL);
	if (!reg_ops)
		return NULL;

	return reg_ops;
}

static int parse_read_op(struct device *dev, const char *reg_ops,
			 struct iio_regmap_op *op, int line)
{
	int ret;

	ret = sscanf(reg_ops, " %i , , , , %i ", &op->addr, &op->dbg);
	if (ret != 2) {
		dev_err(dev, "Invalid READ op format, line: %d.", line);
		return -EINVAL;
	}
	return 0;
}

static int parse_read_mask_op(struct device *dev, const char *reg_ops,
			      struct iio_regmap_op *op, int line)
{
	int ret;

	ret = sscanf(reg_ops, " %i , %i , , , %i ", &op->addr, &op->mask,
		     &op->dbg);
	if (ret != 3) {
		dev_err(dev, "Invalid READ_MASK op format, line: %d.", line);
		return -EINVAL;
	}
	return 0;
}

static int parse_wait_mask_op(struct device *dev, const char *reg_ops,
			      struct iio_regmap_op *op, int line)
{
	int ret;

	ret = sscanf(reg_ops, " %i , %i , %i , %i , %i ", &op->addr, &op->mask,
		     &op->val, &op->time, &op->dbg);
	if (ret != 5) {
		dev_err(dev, "Invalid WAIT_MASK op format, line: %d.", line);
		return -EINVAL;
	}
	return 0;
}

static int parse_wait_op(struct device *dev, const char *reg_ops,
			 struct iio_regmap_op *op, int line)
{
	int ret;

	ret = sscanf(reg_ops, " , , , %i , %i ", &op->time, &op->dbg);
	if (ret != 2) {
		dev_err(dev, "Invalid WAIT_MS op format, line: %d.", line);
		return -EINVAL;
	}
	return 0;
}

static int parse_write_op(struct device *dev, const char *reg_ops,
			  struct iio_regmap_op *op, int line)
{
	int ret;

	ret = sscanf(reg_ops, " %i , , %i , , %i ", &op->addr, &op->val,
		     &op->dbg);
	if (ret != 3) {
		dev_err(dev, "Invalid WRITE op format, line: %d.", line);
		return -EINVAL;
	}
	return 0;
}

static int parse_write_mask_op(struct device *dev, const char *reg_ops,
			       struct iio_regmap_op *op, int line)
{
	int ret;

	ret = sscanf(reg_ops, " %i , %i , %i , , %i ", &op->addr, &op->mask,
		     &op->val, &op->dbg);
	if (ret != 4) {
		dev_err(dev, "Invalid WRITE_MASK format, line: %d.", line);
		return -EINVAL;
	}
	return 0;
}

static int read_register_op(struct device *dev, const char *fw_reg_ops,
			    struct iio_regmap_op *reg_op, unsigned int line)
{
	int ret = 0;

	if (!fw_reg_ops || !reg_op)
		return -EINVAL;

	switch (reg_op->op) {
	case IIO_REGMAP_READ:
		ret = parse_read_op(dev, fw_reg_ops, reg_op, line);
		break;
	case IIO_REGMAP_READ_MASK:
		ret = parse_read_mask_op(dev, fw_reg_ops, reg_op, line);
		break;
	case IIO_REGMAP_WAIT_MASK:
		ret = parse_wait_mask_op(dev, fw_reg_ops, reg_op, line);
		break;
	case IIO_REGMAP_WAIT_MS:
		ret = parse_wait_op(dev, fw_reg_ops, reg_op, line);
		break;
	case IIO_REGMAP_WRITE:
		ret = parse_write_op(dev, fw_reg_ops, reg_op, line);
		break;
	case IIO_REGMAP_WRITE_MASK:
		ret = parse_write_mask_op(dev, fw_reg_ops, reg_op, line);
		break;
	default:
		dev_err(dev, "Invalid op at line: %d", line);
		return -EINVAL;
	}

	if (ret < 0)
		return ret;

	return 0;
}

static int regmap_cmd_to_opcode(const char *cmd)
{
	int i;
	int op_code = -1;

	for (i = IIO_REGMAP_READ; i <= IIO_REGMAP_WRITE_MASK; i++) {
		if (!strcasecmp(cmd, iio_reg_op_map[i])) {
			op_code = i;
			break;
		}
	}
	return op_code;
}

static int parse_register_ops(struct device *dev, const char *input_fw,
			      struct iio_regmap_op *reg_ops)
{
	char parsed_cmd[REG_OP_STRLEN];
	char *reg_op_end;
	unsigned int op_size = 0;
	unsigned int line = 0;
	int op_nr = 0;
	int ret;
	int op_code;

	if (!input_fw || !reg_ops)
		return -EINVAL;

	while (*input_fw != '\0') {
		reg_op_end = strchr(input_fw, ',');
		op_size = reg_op_end - input_fw;
		if (op_size > REG_OP_STRLEN) {
			dev_err(dev, "Invalid op size.");
			return -EINVAL;
		}

		memset(parsed_cmd, 0, REG_OP_STRLEN);
		memcpy(parsed_cmd, input_fw, op_size);
		input_fw = reg_op_end + 1;

		op_code = regmap_cmd_to_opcode(parsed_cmd);
		if (op_code > 0) {
			reg_ops[op_nr].op = op_code;
			ret = read_register_op(dev, input_fw,
					       &reg_ops[op_nr], line);
			if (ret < 0)
				return ret;
			op_nr++;
		} else {
			dev_err(dev, "Invalid cmd at line: %d", line);
			return -EINVAL;
		}

		input_fw = strchr(input_fw, '\n') + 1;
		line++;
	}

	return op_nr;
}

static int run_read_op(struct device *dev, struct regmap *regmap, bool use_mask,
		       const struct iio_regmap_op *op)
{
	int ret;
	unsigned int value;

	ret = regmap_read(regmap, op->addr, &value);
	if (ret < 0) {
		dev_err(dev, "regmap_read failed, addr: %x, code: %d",
		reop->addr, ret);
		return ret;
	}
	if (use_mask)
		value &= use_mask;
	if (op->dbg)
		dev_info(dev, "Register: [%x], value: [%x].", op->addr, value);
	return 0;
}

static int run_wait_mask_op(struct device *dev, struct regmap *regmap,
			    const struct iio_regmap_op *op)
{
	int ret;
	unsigned int value;
	unsigned int wait_time;
	unsigned int wait_nr_times = WAIT_POLL_NR_TIMES;

	wait_time = op->time % WAIT_POLL_NR_TIMES;
	if (wait_time)
		mdelay(wait_time);
	wait_time = op->time / WAIT_POLL_NR_TIMES;

	while (wait_nr_times != 0) {
		if (wait_time)
			mdelay(wait_time);

		ret = regmap_read(regmap, op->addr, &value);
		if (ret < 0) {
			dev_err(dev, "regmap_read failed, addr:%x, code: %d",
				op->addr, ret);
			return ret;
		}

		wait_nr_times--;
		value &= op->mask;
		if (value == op->val)
			break;

		if (wait_nr_times == 0) {
			dev_err(dev,
				"Invalid reg [%x] value:[%x], expected[%x]",
				op->addr, value, op->val);
			return -ETIMEDOUT;
		}
	}

	return 0;
}

static int run_write_mask_op(struct device *dev, struct regmap *regmap,
			     const struct iio_regmap_op *op)
{
	int ret;
	unsigned int register_value;

	ret = regmap_read(regmap, op->addr, &register_value);
	if (ret < 0) {
		dev_err(dev, "regmap_read failed, addr: %x, code: %d",
			op->addr, ret);
		return ret;
	}

	register_value &= ~op->mask;
	register_value |= op->val;

	ret = regmap_write(regmap, op->addr, op->val);
	if (ret < 0)
		dev_err(dev, "regmap_write failed, addr: %x, code: %d",
		op->addr, ret);
	return ret;
}

static int run_write_op(struct device *dev, struct regmap *regmap,
			const struct iio_regmap_op *op)
{
	int ret;

	ret = regmap_write(regmap, op->addr, op->val);
	if (ret < 0)
		dev_err(dev, "regmap_write failed, addr: %x, code: %d",
			op->addr, ret);
	return ret;
}

static int iio_run_regmap_ops(struct device *dev, struct regmap *regmap,
			      const struct iio_regmap_op *reg_ops,
			      unsigned int nr_ops)
{
	const struct iio_regmap_op *reg_op = 0;
	int ret = 0;

	if (!reg_ops || !regmap)
		return -EINVAL;

	for (reg_op = reg_ops; reg_op < reg_ops + nr_ops; reg_op++) {
		switch (reg_op->op) {
		case IIO_REGMAP_READ:
			ret = run_read_op(dev, regmap, false, reg_op);
			break;
		case IIO_REGMAP_READ_MASK:
			ret = run_read_op(dev, regmap, true, reg_op);
			break;
		case IIO_REGMAP_WAIT_MASK:
			ret = run_wait_mask_op(dev, regmap, reg_op);
			break;
		case IIO_REGMAP_WAIT_MS:
			if (reg_op->time)
				mdelay(reg_op->time);
			break;
		case IIO_REGMAP_WRITE:
			ret = run_write_op(dev, regmap, reg_op);
			break;
		case IIO_REGMAP_WRITE_MASK:
			ret = run_write_mask_op(dev, regmap, reg_op);
			break;
		default:
			dev_err(dev, "Invalid op code: %d.", reg_op->op);
			return -EINVAL;
		}
		if (ret < 0)
			return ret;
	}
	return 0;
}

/* Retrieve from device node the firmware name then
 * read from firmware register operations,
 * allocate a copy of the firmware data
 * parse found register operations and return
 * an iio_regmap_op array containing all operations
 * to be ran.
 */
static const struct iio_regmap_op *parse_firmware(struct device *dev,
						  int *nr_ops)
{
	const char *firmware_name;
	const char *data;
	const struct firmware *firmware;
	struct iio_regmap_op *reg_ops;
	char *input_fw;
	int ret;

	ret = device_property_read_string(dev, "firmware", &firmware_name);
	if (ret < 0) {
		dev_err(dev, "Firmware name property read failed!");
		return ERR_PTR(-EINVAL);
	}

	ret = request_firmware(&firmware, firmware_name, dev);
	if (ret < 0) {
		dev_err(dev, "request_firmware failed!");
		return ERR_PTR(-EINVAL);
	}

	input_fw = devm_kzalloc(dev,
				(firmware->size + 1) * sizeof(char),
				GFP_KERNEL);
	if (!input_fw)
		return ERR_PTR(-ENOMEM);

	data = firmware->data;
	if (!data) {
		dev_err(dev, "Firmware data not loaded.");
		return ERR_PTR(-EINVAL);
	}
	memcpy(input_fw, data, (firmware->size) * sizeof(char));
	release_firmware(firmware);

	reg_ops = alloc_register_ops(dev, input_fw);
	if (!reg_ops) {
		dev_err(dev, "Could not allocate registers array.");
		return ERR_PTR(-ENOMEM);
	}

	ret = parse_register_ops(dev, input_fw, reg_ops);
	if (ret < 0) {
		dev_err(dev, "Register ops parsing failed!");
		return ERR_PTR(ret);
	}
	*nr_ops = ret;

	return reg_ops;
}

int iio_regmap_probe(struct device *dev, struct regmap *regmap,
		     const char *name)
{
	struct iio_dev *indio_dev;
	struct iio_regmap *st;
	const struct iio_regmap_op *register_ops;
	int nr_ops;
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
	indio_dev->info = &iio_regmap_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret < 0)
		dev_err(&indio_dev->dev, "iio-regmap device register failed\n");

	register_ops = parse_firmware(dev, &nr_ops);
	if (IS_ERR(register_ops)) {
		dev_err(dev, "parse_firmware failed!\n");
		return PTR_ERR(register_ops);
	}

	return iio_run_regmap_ops(dev, regmap, register_ops, nr_ops);
}

EXPORT_SYMBOL_GPL(iio_regmap_probe);

MODULE_AUTHOR("Alexandru Tachici <alexandru.tachici@analog.com>");
MODULE_DESCRIPTION("Generic IIO access driver");
MODULE_LICENSE("GPL v2");
