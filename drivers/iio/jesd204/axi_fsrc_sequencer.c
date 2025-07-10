// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices AXI FSRC Sequencer
 *
 * Copyright 2025 Analog Devices Inc.
 */

#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/math.h>

#define REG_SCRATCH				0x08

// Sequencer Registers
#define REG_SEQ_CTRL_1				0x10
#define   REG_SEQ_GPIO_CHANGE_CNT		GENMASK(15, 0)

#define REG_SEQ_CTRL_2				0x14
#define   REG_SEQ_FIRST_TRIG_CNT		GENMASK(15, 0)
#define   REG_SEQ_SECOND_TRIG_CNT		GENMASK(31, 16)

#define REG_SEQ_CTRL_3				0x18
#define   REG_SEQ_START				BIT(0)
#define   REG_SEQ_EN				BIT(1)
#define   REG_SEQ_TX_ACCUM_RST_CNT		GENMASK(31, 16)

#define REG_SEQ_CTRL_4				0x1c
#define   REG_SEQ_EXT_TRIG_EN			BIT(0)
#define   REG_SEQ_DEBUG				BIT(12)
#define   REG_SEQ_RX_DELAY			GENMASK(31, 16)

// TX Registers
#define REG_TX_ENABLE				0x10
#define   REG_TX_ENABLE_ENABLE			BIT(0)
#define   REG_TX_ENABLE_EXT_TRIG_EN		BIT(1)

#define REG_CTRL_TRANSMIT			0x14
#define   REG_CTRL_TRANSMIT_START		BIT(0)
#define   REG_CTRL_TRANSMIT_STOP		BIT(1)
#define   REG_CTRL_TRANSMIT_ACCUM_SET		BIT(2)
#define   REG_CTRL_TRANSMIT_CHANGE_RATE		BIT(3)

#define REG_CONV_MASK				0x18
#define   REG_CONV_MASK_MASK			GENMASK(15, 0)

#define REG_ACCUM_ADD_VAL_L			0x1c
#define REG_ACCUM_ADD_VAL_H			0x20
#define REG_ACCUM_SET_VAL_ADDR			0x24
#define   REG_ACCUM_SET_VAL_ADDR_(x)		FIELD_PREP(GENMASK(3, 0), (x))
#define REG_ACCUM_SET_VAL_L			0x28
#define REG_ACCUM_SET_VAL_H			0x2c
#define REG_ACCUM_WIDTH				0x30

// RX Register
#define REG_RX_ENABLE				0x10
#define   REG_RX_ENABLE_ENABLE			BIT(0)

enum {
	AXI_FSRC_RX_ENABLE,
	AXI_FSRC_TX_ENABLE,
	AXI_FSRC_TX_ACTIVE,
	AXI_FSRC_TX_RATIO_SET,
};

struct axi_fsrc {
	void __iomem *addr[5];
	struct gpio_desc *trig_req_gpio;
	const struct axi_fsrc_sequencer_info *info;
	struct device dev;
	bool tx_enable;
	bool tx_active;
	u8 accum_width;
	u8 m;
	u8 n;
	u8 en_mask;
};

enum {
	AXI_FSRC_CTRL,
	AXI_FSRC_RX,
	AXI_FSRC_TX,
	AXI_FSRC_RX_A,
	AXI_FSRC_TX_A,
};

static inline u32 axi_fsrc_read(void __iomem *base, const u32 reg)
{
	return ioread32(base + reg);
}

static inline void axi_fsrc_write(void __iomem *base, const u32 reg, const u32 value)
{
	iowrite32(value, base + reg);
}

static inline void axi_fsrc_update(void __iomem *base, const u32 reg, u32 mask, const u32 value)
{
	u32 read = ioread32(base + reg);

	read &= ~mask;
	read |= value;

	iowrite32(read, base + reg);
}

static int axi_fsrc_rx_enable(struct axi_fsrc *st, bool en)
{
	if (!st->addr[AXI_FSRC_RX])
		return -ENODEV;
	axi_fsrc_write(st->addr[AXI_FSRC_RX], REG_RX_ENABLE, FIELD_PREP(REG_RX_ENABLE_ENABLE, en));

	return 0;
}

static int axi_fsrc_tx_enable(struct axi_fsrc *st, bool en)
{
	if (!st->addr[AXI_FSRC_TX])
		return -ENODEV;

	axi_fsrc_update(st->addr[AXI_FSRC_TX], REG_TX_ENABLE,
			REG_TX_ENABLE_ENABLE, FIELD_PREP(REG_TX_ENABLE_ENABLE, en));
	st->tx_enable = en;
	if (!en)
		st->tx_active = false;

	return 0;
}

static int axi_fsrc_tx_active(struct axi_fsrc *st, bool en)
{
	if (!st->addr[AXI_FSRC_TX])
		return -ENODEV;
	if (!st->tx_enable != false)
		return -EINVAL;

	if (en)
		axi_fsrc_write(st->addr[AXI_FSRC_TX], REG_CTRL_TRANSMIT,
			       REG_CTRL_TRANSMIT_START);
	else
		/* Send only invalid samples */
		axi_fsrc_write(st->addr[AXI_FSRC_TX], REG_CTRL_TRANSMIT,
			       REG_CTRL_TRANSMIT_STOP);
	st->tx_active = en;

	return 0;
}

static int axi_fsrc_tx_set_ratio(struct axi_fsrc *st, const u64 n, const u64 m)
{
	u64 val;
	const u64 one_fixed = 1ULL << st->accum_width;
	const u64 ratio_fixed = mul_u64_u64_div_u64(one_fixed, n, m);

	axi_fsrc_write(st->addr[AXI_FSRC_TX], REG_CONV_MASK, (u32)REG_CONV_MASK_MASK);
	for (int i = 0; i <= 15; i++) {
		val = ((~ratio_fixed + 1) + (i * ratio_fixed));
		axi_fsrc_write(st->addr[AXI_FSRC_TX], REG_ACCUM_SET_VAL_L, val);
		axi_fsrc_write(st->addr[AXI_FSRC_TX], REG_ACCUM_SET_VAL_H, val >> 32);
		axi_fsrc_write(st->addr[AXI_FSRC_TX], REG_ACCUM_SET_VAL_ADDR, REG_ACCUM_SET_VAL_ADDR_(i));
	}
	val = ratio_fixed;
	axi_fsrc_write(st->addr[AXI_FSRC_TX], REG_ACCUM_ADD_VAL_L, val);
	axi_fsrc_write(st->addr[AXI_FSRC_TX], REG_ACCUM_ADD_VAL_H, val >> 32);
	axi_fsrc_write(st->addr[AXI_FSRC_TX], REG_CTRL_TRANSMIT, REG_CTRL_TRANSMIT_ACCUM_SET);

	st->n = n;
	st->m = m;

	return 0;
}

static ssize_t axi_fsrc_ext_read(struct iio_dev *indio_dev,
				 uintptr_t private,
				 const struct iio_chan_spec *chan,
				 char *buf)
{
	struct axi_fsrc *st = iio_priv(indio_dev);
	unsigned long val = 0;

	iio_device_claim_direct_scoped(return -EBUSY, indio_dev) {
		switch ((u32)private) {
		case AXI_FSRC_RX_ENABLE:
			if (!st->addr[AXI_FSRC_RX])
				return -ENODEV;
			val = FIELD_GET(REG_RX_ENABLE_ENABLE,
					axi_fsrc_read(st->addr[AXI_FSRC_RX], REG_RX_ENABLE));
			return sprintf(buf, "%lu\n", val);
		case AXI_FSRC_TX_ENABLE:
			if (!st->addr[AXI_FSRC_TX])
				return -ENODEV;
			val = FIELD_GET(REG_TX_ENABLE_ENABLE,
					axi_fsrc_read(st->addr[AXI_FSRC_TX], REG_TX_ENABLE));
			return sprintf(buf, "%lu\n", val);
		case AXI_FSRC_TX_ACTIVE:
			if (!st->addr[AXI_FSRC_TX])
				return -ENODEV;
			return sprintf(buf, "%x\n", st->tx_active);
		case AXI_FSRC_TX_RATIO_SET:
			return sprintf(buf, "%u %u\n", st->n, st->m);
		default:
			return -EINVAL;
		}
	}
	unreachable();
}

static ssize_t axi_fsrc_ext_write(struct iio_dev *indio_dev,
				  uintptr_t private,
				  const struct iio_chan_spec *chan,
				  const char *buf, size_t len)
{
	struct axi_fsrc *st = iio_priv(indio_dev);
	u32 n = 0, m = 0;
	bool enable;
	int size, ret = 0;

	iio_device_claim_direct_scoped(return -EBUSY, indio_dev) {
		switch ((u32)private) {
		case AXI_FSRC_RX_ENABLE:
		case AXI_FSRC_TX_ENABLE:
		case AXI_FSRC_TX_ACTIVE:
			ret = strtobool(buf, &enable);
			if (ret)
				return ret;
			break;
		case AXI_FSRC_TX_RATIO_SET:
			size = sscanf(buf, "%u %u", &n, &m);
			if (size != 2)
				return -EINVAL;
			break;
		}

		switch ((u32)private) {
		case AXI_FSRC_RX_ENABLE:
			ret = axi_fsrc_rx_enable(st, enable);
			break;
		case AXI_FSRC_TX_ENABLE:
			ret = axi_fsrc_tx_enable(st, enable);
			break;
		case AXI_FSRC_TX_ACTIVE:
			ret = axi_fsrc_tx_active(st, enable);
			break;
		case AXI_FSRC_TX_RATIO_SET:
			if ((m == 0) || (n / m >= 2))
				return -EINVAL;
			axi_fsrc_tx_set_ratio(st, n, m);
			break;
		}

		return ret ? ret : len;
	}
	unreachable();
}
#define AXI_FSRC_EXT_INFO(_name, _ident) { \
	.name = _name, \
	.read = axi_fsrc_ext_read, \
	.write = axi_fsrc_ext_write, \
	.private = _ident, \
	.shared = IIO_SEPARATE, \
}

static const struct iio_chan_spec_ext_info axi_fsrc_ext_info[] = {
	AXI_FSRC_EXT_INFO("rx_enable", AXI_FSRC_RX_ENABLE),
	AXI_FSRC_EXT_INFO("tx_enable", AXI_FSRC_TX_ENABLE),
	AXI_FSRC_EXT_INFO("tx_active", AXI_FSRC_TX_ACTIVE),
	AXI_FSRC_EXT_INFO("tx_ratio_set", AXI_FSRC_TX_RATIO_SET),
	{ },
};

static const struct iio_chan_spec axi_fsrc_chan = {
	.type = IIO_ALTVOLTAGE,
	.indexed = 1,
	.output = 1,
	.ext_info = axi_fsrc_ext_info,
};

/**
 * REVISIT: upstream these methods are now optional.
 */
static int axi_fsrc_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val,
			    int *val2,
			    long mask)
{
	return -EINVAL;
}

static int axi_fsrc_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	return -EINVAL;
}

static int axi_fsrc_debugfs_reg_access(struct iio_dev *indio_dev,
				       unsigned reg, unsigned writeval,
				       unsigned *readval)
{
	struct axi_fsrc *st = iio_priv(indio_dev);
	u8 addr = reg >> 16;

	reg &= GENMASK(15,0);
	if (addr > ARRAY_SIZE(st->addr) || (reg & GENMASK(1,0)))
		return -EINVAL;
	if (!st->addr[addr])
		return -ENODEV;

	if (readval != NULL)
		*readval = axi_fsrc_read(st->addr[addr], reg);
	else
		axi_fsrc_write(st->addr[addr], reg, writeval);

	return 0;
}

static const struct iio_info axi_fsrc_info = {
	.read_raw = &axi_fsrc_read_raw,
	.write_raw = &axi_fsrc_write_raw,
	.debugfs_reg_access = &axi_fsrc_debugfs_reg_access,
};

/* Match table for of_platform binding */
static const struct of_device_id axi_fsrc_sequencer_of_match[] = {
	{ .compatible = "adi,axi-fsrc-sequencer", .data = &platform_bus_type},
	{ /* end of list */ }
};
MODULE_DEVICE_TABLE(of, axi_fsrc_sequencer_of_match);

int axi_fsrc_sequencer_add_to_topology(struct device *dev, struct axi_fsrc *st, struct device_node *np)
{
	const char rx_a[] = "adi,fsrc_rx_a";
	const char tx_a[] = "adi,fsrc_tx_a";
	const char rx[] = "adi,fsrc_rx";
	const char tx[] = "adi,fsrc_tx";
	const char *props[4] = {rx, tx, rx_a, tx_a};
	bool matched = 0;
	u32 reg[2];
	int ret;

	ret = of_property_read_u32_array(np, "reg", reg, 2);
	if (ret)
		return ret;

	for (int i = 0; i < ARRAY_SIZE(props); i++) {
		if (of_property_read_bool(np, props[i])) {
			if (st->en_mask & BIT(i)) {
				dev_err(&st->dev,
					"index %d in fscr topology already allocated by %p",
					i, st->addr[i+1]);
				return -ENOENT;
			}
			if (!devm_request_mem_region(dev, reg[0], reg[1],
						  dev_name(&st->dev))) {
				dev_err(&st->dev, "request_mem_region failed\n");
				return -ENOMEM;
			}
			st->addr[i+1] = devm_ioremap(dev, reg[0], reg[1]);
			if (!st->addr[i+1]) {
				dev_err(&st->dev, "ioremap failed\n");
				return -ENOMEM;
			}
			st->en_mask |= BIT(i);

			matched = true;
		}
	}

	if (!matched) {
		dev_err(&st->dev,
			"device of address %x in fscr topology missing role",
			reg[0]);
		return -ENOENT;
	}

	return 0;
}

typedef struct {
    uint16_t gpio_change_cnt;               /*!< GPIO change count      */
    uint16_t first_trig_cnt;                /*!< First trigger count    */
    uint16_t fsrc_accum_reset_cnt;          /*!< FSRC accum reset count */
    uint16_t rx_delay_cnt;                  /*!< Rx capture count */
} adi_fpga_apollo_hw_fsrc_count_t;

static int axi_fsrc_seq_configure(struct axi_fsrc *st, adi_fpga_apollo_hw_fsrc_count_t *count)
{
	axi_fsrc_update(st->addr[AXI_FSRC_CTRL], REG_SEQ_CTRL_2, (u32)REG_SEQ_FIRST_TRIG_CNT,
			FIELD_PREP(REG_SEQ_FIRST_TRIG_CNT, count->first_trig_cnt));
	axi_fsrc_update(st->addr[AXI_FSRC_CTRL], REG_SEQ_CTRL_3, (u32)REG_SEQ_TX_ACCUM_RST_CNT,
			FIELD_PREP(REG_SEQ_TX_ACCUM_RST_CNT, count->fsrc_accum_reset_cnt));
	axi_fsrc_update(st->addr[AXI_FSRC_CTRL], REG_SEQ_CTRL_4, (u32)REG_SEQ_RX_DELAY,
			FIELD_PREP(REG_SEQ_RX_DELAY, count->rx_delay_cnt));
	axi_fsrc_update(st->addr[AXI_FSRC_CTRL], REG_SEQ_CTRL_3, REG_SEQ_EN, REG_SEQ_EN);

	return 0;
}

static int axi_fsrc_tx_configure(struct axi_fsrc *st)
{
	if (st->addr[AXI_FSRC_TX])
		st->accum_width = axi_fsrc_read(st->addr[AXI_FSRC_TX], REG_ACCUM_WIDTH);
	return axi_fsrc_tx_set_ratio(st, 1, 1);
}

static int axi_fsrc_rx_configure(struct axi_fsrc *st)
{
	return 0;
}

static int axi_fsrc_init(struct axi_fsrc *st)
{
	int ret;
	adi_fpga_apollo_hw_fsrc_count_t count = {
		.gpio_change_cnt = 1,
		.first_trig_cnt = 1002,
		.fsrc_accum_reset_cnt = 1102,
		.rx_delay_cnt = 2102
	};

	ret = axi_fsrc_seq_configure(st, &count);
	if (ret)
		return ret;
	if (st->en_mask & BIT(0)) {
		ret = axi_fsrc_rx_configure(st);
		if (ret)
			return ret;
	}
	if (st->en_mask & BIT(1)) {
		ret = axi_fsrc_tx_configure(st);
		if (ret)
			return ret;
	}
	return 0;
}

static int axi_fsrc_sequencer_probe(struct platform_device *pdev)
{
	const struct axi_fsrc_sequencer_info *info;
	struct iio_dev *indio_dev;
	struct resource *res;
	struct axi_fsrc *st;
	int ret;

	info = device_get_match_data(&pdev->dev);
	if (!info)
		return -ENODEV;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	indio_dev->info = &axi_fsrc_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = &axi_fsrc_chan;
	indio_dev->num_channels = 1;
	indio_dev->name = "axi_fsrc";

	for (int i = 0 ; i < ARRAY_SIZE(st->addr); i++)
		st->addr[i] = NULL;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	st->addr[AXI_FSRC_CTRL] = devm_ioremap_resource(&pdev->dev, res);

	if (IS_ERR(st->addr[AXI_FSRC_CTRL]))
		return PTR_ERR(st->addr[AXI_FSRC_CTRL]);
	st->dev = pdev->dev;

	struct device_node *np = pdev->dev.of_node;
	struct device_node *np_;

	st->tx_enable = false;
	st->tx_active = false;
	st->en_mask = 0;

	for (int i = 0 ; ; i++) {
		np_ = of_parse_phandle(np, "fsrc-topology", i);
		if (!np_)
			break;

		ret = axi_fsrc_sequencer_add_to_topology(&pdev->dev, st, np_);
		if (ret)
			return ret;

		of_node_put(np_);
	}

	st->trig_req_gpio = devm_gpiod_get_optional(&pdev->dev, "trig-req", GPIOD_OUT_LOW);
	if (IS_ERR(st->trig_req_gpio))
		return PTR_ERR(st->trig_req_gpio);

	axi_fsrc_write(st->addr[AXI_FSRC_CTRL], REG_SCRATCH, 0xBE);
	u8 val = axi_fsrc_read(st->addr[AXI_FSRC_CTRL], REG_SCRATCH);
	if (val != 0xBE)
		return dev_err_probe(&pdev->dev, -EINVAL, "Failed sanity test\n");

	ret = axi_fsrc_init(st);
	if (ret)
		return ret;

	return devm_iio_device_register(&pdev->dev, indio_dev);;
}

static struct platform_driver axi_fsrc_sequencer = {
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = axi_fsrc_sequencer_of_match,
	},
	.probe = axi_fsrc_sequencer_probe,
};
module_platform_driver(axi_fsrc_sequencer);

MODULE_AUTHOR("Me <me@analog.com>");
MODULE_DESCRIPTION("Analog Devices AXI FSRC Sequencer device driver");
MODULE_LICENSE("GPL v2");
