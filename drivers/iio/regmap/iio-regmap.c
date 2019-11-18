// SPDX-License-Identifier: GPL-2.0
/*
 * Generic IIO access driver
 *
 * Copyright 2019 Analog Devices Inc.
 */

 /* Possible register operations table:
  * +----------+--------+-----------+--------+-------+-------+
  * |    OP      |  ADDR  |    MASK   |  VAL  | TIME |  DBG  |
  * +------------------------------------------------+-------+
  * |    READ    |   X    |          |        |      |   X   |
  * +------------------------------------------------+-------+
  * | READ_MASK  |   X    |     X    |        |      |   X   |
  * +------------------------------------------------+-------+
  * | WAIT_MASK  |   X    |     X    |    X   |   X  |   X   |
  * +------------------------------------------------+-------+
  * |  WAIT_MS   |        |          |        |   X  |   X   |
  * +----------+--------+-----------+--------+-------+-------+
  * |   WRITE    |   X    |          |    X   |      |   X   |
  * +----------+--------+-----------+--------+-------+-------+
  * | WRITE_MASK |   X    |     X    |    X   |      |   X   |
  * +----------+--------+-----------+--------+-------+-------+
  * Wait times are defined in milliseconds:
  * READ        -> read value at ADDR
  * READ_MASK   -> read value at ADDR with MASK
  * WAIT_MASK   -> wait TIME for ADDR value with MASK to become VAL
  * WAIT_MS     -> wait TIME milliseconds
  * WRTIE       -> write VAL at ADDR
  * WRITE_MASK  -> write VAL at ADDR with MASK
  */

#include <linux/ctype.h>
#include <linux/iio/iio.h>
#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/regmap.h>
#include <linux/string.h>
#include <linux/errno.h>

#include "iio-regmap.h"

#define REG_OP_STRLEN	20

enum iio_regmap_opcode {
	IIO_REGMAP_READ,
	IIO_REGMAP_READ_MASK,
	IIO_REGMAP_WAIT_MASK,
	IIO_REGMAP_WAIT_MS,
	IIO_REGMAP_WRITE,
	IIO_REGMAP_WRITE_MASK,
};

static const char * const iio_reg_op_map[] = {
	[IIO_REGMAP_READ] = "READ",
	[IIO_REGMAP_READ_MASK] = "READ_MASK",
	[IIO_REGMAP_WAIT_MASK] = "WAIT_MASK",
	[IIO_REGMAP_WAIT_MS] = "WAIT_MS",
	[IIO_REGMAP_WRITE] = "WRITE",
	[IIO_REGMAP_WRITE_MASK] = "WRITE_MASK",
};

struct iio_regmap_op {
	enum iio_regmap_opcode	op;
	unsigned int		addr;
	unsigned int		mask;
	unsigned int		val;
	unsigned int		time;
	unsigned int		dbg;
};

static const struct iio_regmap_op  iio_regmap_op_required_fields[] = {
	[IIO_REGMAP_READ] = { .addr = 1, .dbg = 1 },
	[IIO_REGMAP_READ_MASK] = { .addr = 1, .mask = 1, .dbg = 1 },
	[IIO_REGMAP_WAIT_MASK] = { .addr = 1, .mask = 1, .val = 1,
				   .time = 1, .dbg = 1 },
	[IIO_REGMAP_WAIT_MS] = { .time = 1, .dbg = 1 },
	[IIO_REGMAP_WRITE] = { .addr = 1, .val = 1, .dbg = 1 },
	[IIO_REGMAP_WRITE_MASK] = { .addr = 1, .mask = 1, .val = 1, .dbg = 1 },
};

struct iio_regmap {
	struct device *dev;
	struct regmap *regmap;
};

static const struct iio_info iio_regmap_info = {
};

struct regmap_config *iio_regmap_read_config(struct device *dev)
{
	struct regmap_config *regmap_cfg;
	u32 reg_bits;
	u32 val_bits;
	int ret;

	regmap_cfg = devm_kzalloc(dev, sizeof(*regmap_cfg),
				  GFP_KERNEL);
	if (!regmap_cfg)
		return ERR_PTR(-ENOMEM);

	ret = device_property_read_u32(dev, "reg-bits", &reg_bits);
	if (ret < 0) {
		dev_err(dev, "Reading reg-bits property failed!\n");
		return ERR_PTR(-EINVAL);
	}

	ret = device_property_read_u32(dev, "val-bits", &val_bits);
	if (ret < 0) {
		dev_err(dev, "Reading val-bits property failed!\n");
		return ERR_PTR(-EINVAL);
	}

	regmap_cfg->reg_bits = reg_bits;
	regmap_cfg->val_bits = val_bits;

	return regmap_cfg;
}
EXPORT_SYMBOL_GPL(iio_regmap_read_config);

static struct iio_regmap_op *alloc_register_ops(struct device *dev,
						const char *fw_reg_ops,
						unsigned int fw_size)
{
	unsigned int new_lines;
	const char *c;

	if (!fw_reg_ops)
		return ERR_PTR(-EINVAL);

	new_lines = 0;
	for (c = fw_reg_ops; c < fw_reg_ops + fw_size; c++) {
		if (*c == '\n')
			new_lines++;
	}

	if (!new_lines) {
		dev_err(dev, "Firmware file has an invalid format!\n");
		return ERR_PTR(-EINVAL);
	}

	return devm_kzalloc(dev, new_lines * sizeof(struct iio_regmap_op),
			    GFP_KERNEL);
}

/* Skips register command parameter delimeters,
 * places an '\0' at the end of the first parameter
 * found and returns its starting address.
 * fw_reg_ops is updated and points past the returned string.
 */
static char *get_fw_token(char **fw_reg_ops)
{
	char *fw_token;
	char *p;

	if (!fw_reg_ops || !(*fw_reg_ops))
		return NULL;

	for (p = *fw_reg_ops; *p != '\0'; p++) {
		if (*p != ',' && *p != ' ')
			break;
	}
	if (*p != '\0')
		fw_token = p;
	else
		return NULL;

	for (; *p != '\0'; p++) {
		if (*p == ',' || *p == ' ')
			break;
	}

	*p = '\0';
	*fw_reg_ops = ++p;
	return fw_token;
}

static int parse_register_op_field(struct device *dev, unsigned int *field,
				   char **fw_reg_ops, int line)
{
	long field_value;
	char *fw_field;
	int ret;

	fw_field = get_fw_token(fw_reg_ops);
	if (!fw_field) {
		dev_err(dev, "Invalid register command format, line: %d", line);
		return -EINVAL;
	}

	ret = kstrtoul(fw_field, 0, &field_value);
	if (ret < 0) {
		dev_err(dev, "Invalid field value: %s, line: %d", fw_field,
			line);
		return ret;
	}
	*field = field_value;

	return 0;
}

static int command_to_op_code(const char *reg_op_cmd)
{
	char *cmd_buf;
	int op_code;
	int i;

	cmd_buf = kstrdup(reg_op_cmd, GFP_KERNEL);
	if (!cmd_buf)
		return -ENOMEM;

	for (i = 0; i < REG_OP_STRLEN; i++)
		cmd_buf[i] = toupper(cmd_buf[i]);

	op_code =  match_string(iio_reg_op_map, IIO_REGMAP_WRITE_MASK + 1,
				cmd_buf);

	kfree(cmd_buf);
	return op_code;
}

static int parse_register_op(struct device *dev, char *fw_reg_ops,
			     struct iio_regmap_op *reg_op, unsigned int line)
{
	const struct iio_regmap_op *req_fields;
	const char *fw_reg_cmd;
	int ret;

	if (!fw_reg_ops || !reg_op)
		return -EINVAL;

	fw_reg_cmd = get_fw_token(&fw_reg_ops);
	if (!fw_reg_cmd) {
		dev_err(dev, "Malformed register command, line: %d", line);
		return -EINVAL;
	}

	ret = command_to_op_code(fw_reg_cmd);
	if (ret < 0) {
		dev_err(dev, "Invalid register command: %s, line: %d",
			fw_reg_cmd, line);
		return ret;
	}
	reg_op->op = ret;

	req_fields = &iio_regmap_op_required_fields[reg_op->op];
	if (req_fields->addr) {
		ret = parse_register_op_field(dev, &reg_op->addr,
					      &fw_reg_ops, line);
		if (ret < 0)
			return ret;
	}
	if (req_fields->mask) {
		ret = parse_register_op_field(dev, &reg_op->mask,
					      &fw_reg_ops, line);
		if (ret < 0)
			return ret;
	}
	if (req_fields->val) {
		ret = parse_register_op_field(dev, &reg_op->val,
					      &fw_reg_ops, line);
		if (ret < 0)
			return ret;
	}
	if (req_fields->time) {
		ret = parse_register_op_field(dev, &reg_op->time,
					      &fw_reg_ops, line);
		if (ret < 0)
			return ret;
	}
	if (req_fields->dbg) {
		ret = parse_register_op_field(dev, &reg_op->dbg,
					      &fw_reg_ops, line);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int parse_register_ops(struct device *dev, char *input_fw, int size_fw,
			      struct iio_regmap_op *reg_ops)
{
	char *input_fw_next_line;
	int parsed_fw_len = 0;
	int nr_line = 1;
	int ret = 0;

	if (!reg_ops || !input_fw)
		return -EINVAL;

	/* replace each \n with \0  and parse cmd */
	while (parsed_fw_len < size_fw) {
		input_fw_next_line = strchr(input_fw, '\n');
		if (!input_fw_next_line) {
			dev_err(dev, "Missing new line at line: %d.", nr_line);
			return -EINVAL;
		}
		*input_fw_next_line = '\0';
		ret = parse_register_op(dev, input_fw, &reg_ops[nr_line],
					nr_line);
		if (ret < 0)
			return -EINVAL;

		nr_line++;
		input_fw_next_line++;
		parsed_fw_len += input_fw_next_line - input_fw;
		input_fw = input_fw_next_line;
	}

	return nr_line;
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
	const struct firmware *firmware;
	struct iio_regmap_op *reg_ops;
	const char *firmware_name;
	const char *data;
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

	reg_ops = alloc_register_ops(dev, input_fw, firmware->size);
	if (IS_ERR(reg_ops))
		return ERR_CAST(reg_ops);

	ret = parse_register_ops(dev, input_fw, firmware->size, reg_ops);
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
	const struct iio_regmap_op *register_ops;
	struct iio_dev *indio_dev;
	struct iio_regmap *st;
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
	return ret;
}
EXPORT_SYMBOL_GPL(iio_regmap_probe);

MODULE_AUTHOR("Alexandru Tachici <alexandru.tachici@analog.com>");
MODULE_DESCRIPTION("Generic IIO access driver");
MODULE_LICENSE("GPL v2");
