// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices AXI FSRC Sequencer
 *
 * Copyright 2026 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/io.h>
#include <linux/iio/iio.h>
#include <linux/math.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define REG_SCRATCH				0x08

// Sequencer Registers
#define REG_SEQ_CTRL_1				0x10
#define   REG_SEQ_GPIO_CHANGE_CNT		GENMASK(15, 0)

/* One 4-bit SYSREF count per trigger output, trigger n at bits [4n+3:4n] */
#define REG_SEQ_CTRL_2				0x14
#define   REG_SEQ_FIRST_TRIG_CNT		GENMASK(15, 0)

#define REG_SEQ_CTRL_3				0x18
#define   REG_SEQ_START				BIT(0)
#define   REG_SEQ_EN				BIT(1)
#define   REG_SEQ_TX_ACCUM_RST_CNT		GENMASK(19, 16)

#define REG_SEQ_CTRL_4				0x1c
#define   REG_SEQ_EXT_TRIG_EN			BIT(0)
#define   REG_SEQ_DEBUG				BIT(12)
#define   REG_SEQ_RX_DELAY			GENMASK(19, 16)

#define REG_SEQ_CTRL_5				0x44
#define   REG_SEQ_SECOND_TRIG_CNT		GENMASK(15, 0)

#define SEQ_TRIG_CNT(x)				((x) * 0x1111)

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
#define   REG_ACCUM_SET_VAL_ADDR_(x)		FIELD_PREP(GENMASK(5, 0), (x))
#define REG_ACCUM_SET_VAL_L			0x28
#define REG_ACCUM_SET_VAL_H			0x2c
#define REG_ACCUM_WIDTH				0x30
#define REG_ACCUM_STEP_VAL_L			0x38
#define REG_ACCUM_STEP_VAL_H			0x3c
#define REG_NUM_SAMPLES				0x40
#define REG_GROUP				0x44
#define   REG_GROUP_BEATS_M1			GENMASK(7, 0)
#define ACCUM_MAX_SLOTS				64
/* Keeps exact carries carrying despite rounding, far below one sample step */
#define ACCUM_BIAS				BIT_ULL(20)

// RX Register
#define REG_RX_ENABLE				0x10
#define   REG_RX_ENABLE_ENABLE			BIT(0)

enum {
	AXI_FSRC_RX_ENABLE,
	AXI_FSRC_TX_ENABLE,
	AXI_FSRC_TX_TRANSMIT,
	AXI_FSRC_TX_RATIO_SET,
	AXI_FSRC_SEQ_START,
};

struct axi_fsrc {
	struct regmap *map[5];
	struct device *dev;
	bool tx_enable;
	bool tx_active;
	u8 accum_width;
	u8 num_samples;
	u32 m;
	u32 n;
	u32 ns;
};

enum {
	AXI_FSRC_CTRL,
	AXI_FSRC_RX,
	AXI_FSRC_TX,
	AXI_FSRC_RX_A,
	AXI_FSRC_TX_A,
};

static const char * const axi_fsrc_core_names[] = {
	[AXI_FSRC_CTRL] = "ctrl",
	[AXI_FSRC_RX] = "rx",
	[AXI_FSRC_TX] = "tx",
	[AXI_FSRC_RX_A] = "rx_a",
	[AXI_FSRC_TX_A] = "tx_a",
};

static int axi_fsrc_regmap_init(struct axi_fsrc *st, int core,
				void __iomem *base, resource_size_t size)
{
	const struct regmap_config config = {
		.name = axi_fsrc_core_names[core],
		.reg_bits = 32,
		.val_bits = 32,
		.reg_stride = 4,
		.max_register = size - 4,
		.fast_io = true,
	};

	st->map[core] = devm_regmap_init_mmio(st->dev, base, &config);
	if (IS_ERR(st->map[core]))
		return dev_err_probe(st->dev, PTR_ERR(st->map[core]),
				     "failed to init %s regmap\n",
				     axi_fsrc_core_names[core]);

	return 0;
}

static int axi_fsrc_write64(struct regmap *map, u32 reg_l, u32 reg_h, u64 val)
{
	int ret;

	ret = regmap_write(map, reg_l, lower_32_bits(val));
	if (ret)
		return ret;

	return regmap_write(map, reg_h, upper_32_bits(val));
}

static int axi_fsrc_rx_enable(struct axi_fsrc *st, bool en)
{
	if (!st->map[AXI_FSRC_RX])
		return -ENODEV;

	return regmap_write(st->map[AXI_FSRC_RX], REG_RX_ENABLE,
			    FIELD_PREP(REG_RX_ENABLE_ENABLE, en));
}

static int axi_fsrc_tx_enable(struct axi_fsrc *st, bool en)
{
	int ret;

	if (!st->map[AXI_FSRC_TX])
		return -ENODEV;

	ret = regmap_assign_bits(st->map[AXI_FSRC_TX], REG_TX_ENABLE,
				 REG_TX_ENABLE_ENABLE, en);
	if (ret)
		return ret;

	st->tx_enable = en;
	if (!en)
		st->tx_active = false;

	return 0;
}

static int axi_fsrc_tx_active(struct axi_fsrc *st, bool en)
{
	struct regmap *map = st->map[AXI_FSRC_TX];
	int ret;

	if (!map)
		return -ENODEV;
	if (!st->tx_enable)
		return -EINVAL;

	if (en) {
		ret = regmap_clear_bits(map, REG_TX_ENABLE, REG_TX_ENABLE_EXT_TRIG_EN);
		if (ret)
			return ret;

		ret = regmap_write(map, REG_CTRL_TRANSMIT, REG_CTRL_TRANSMIT_START);
	} else {
		/* Send only invalid samples */
		ret = regmap_write(map, REG_CTRL_TRANSMIT, REG_CTRL_TRANSMIT_STOP);
	}
	if (ret)
		return ret;

	st->tx_active = en;

	return 0;
}

static int axi_fsrc_seq_start(struct axi_fsrc *st);

/*
 * Apollo's JRx rate match FIFO takes the NS samples of each conv_clk as all
 * valid or all invalid, so holes come in groups of NS samples. With r = m/n,
 * group g is valid when adding r to its phase 1 - r + g * r carries. A beat
 * holds num_samples / NS groups, or a group spans NS / num_samples beats and
 * the IP core increments the accumulators once per group.
 */
static u64 axi_fsrc_grid_frac(u64 x, u64 grid, u64 one)
{
	u64 rem;

	div64_u64_rem(x, grid, &rem);
	return mul_u64_u64_div_u64(rem, one, grid);
}

static int axi_fsrc_tx_set_ratio(struct axi_fsrc *st, const u32 n, const u32 m,
				 const u32 ns)
{
	struct regmap *map = st->map[AXI_FSRC_TX];
	const u64 one = BIT_ULL(st->accum_width);
	const u64 mask = one - 1;
	u32 groups_per_beat = 1, group_beats = 1;
	u64 add, step, val;
	int slots, ret;

	if (!map)
		return -ENODEV;
	if (!m || !ns || n < m || n / m != 1 || st->accum_width >= 64)
		return -EINVAL;

	if (st->num_samples) {
		if (ns <= st->num_samples) {
			if (st->num_samples % ns)
				return -EINVAL;
			groups_per_beat = st->num_samples / ns;
		} else {
			if (ns % st->num_samples || ns / st->num_samples > 256)
				return -EINVAL;
			group_beats = ns / st->num_samples;
		}
	}
	if (n == m) {
		/* r = 1 does not fit the accumulator: every add has to carry */
		add = mask;
		step = 0;
	} else {
		add = mul_u64_u64_div_u64(one, m, n);
		step = axi_fsrc_grid_frac((u64)m * groups_per_beat, n, one) + 1;
	}

	ret = regmap_write(map, REG_CONV_MASK, REG_CONV_MASK_MASK);
	if (ret)
		return ret;

	ret = regmap_write(map, REG_GROUP, FIELD_PREP(REG_GROUP_BEATS_M1, group_beats - 1));
	if (ret)
		return ret;

	slots = st->num_samples ? st->num_samples : ACCUM_MAX_SLOTS;
	for (int i = 0; i < slots; i++) {
		u64 g = groups_per_beat > 1 ? i / ns : 0;

		if (n == m)
			val = ACCUM_BIAS;
		else
			val = axi_fsrc_grid_frac((u64)(n - m) + m * g, n, one) + 1 + ACCUM_BIAS;
		val &= mask;
		ret = axi_fsrc_write64(map, REG_ACCUM_SET_VAL_L, REG_ACCUM_SET_VAL_H, val);
		if (ret)
			return ret;

		ret = regmap_write(map, REG_ACCUM_SET_VAL_ADDR, REG_ACCUM_SET_VAL_ADDR_(i));
		if (ret)
			return ret;
	}

	ret = axi_fsrc_write64(map, REG_ACCUM_ADD_VAL_L, REG_ACCUM_ADD_VAL_H, add);
	if (ret)
		return ret;

	ret = axi_fsrc_write64(map, REG_ACCUM_STEP_VAL_L, REG_ACCUM_STEP_VAL_H, step);
	if (ret)
		return ret;

	ret = regmap_write(map, REG_CTRL_TRANSMIT, REG_CTRL_TRANSMIT_ACCUM_SET);
	if (ret)
		return ret;

	st->n = n;
	st->m = m;
	st->ns = ns;

	return 0;
}

static ssize_t axi_fsrc_ext_read(struct iio_dev *indio_dev,
				 uintptr_t private,
				 const struct iio_chan_spec *chan,
				 char *buf)
{
	struct axi_fsrc *st = iio_priv(indio_dev);
	unsigned int val;
	int ret;

	switch ((u32)private) {
	case AXI_FSRC_RX_ENABLE:
		if (!st->map[AXI_FSRC_RX])
			return -ENODEV;
		ret = regmap_read(st->map[AXI_FSRC_RX], REG_RX_ENABLE, &val);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%lu\n", FIELD_GET(REG_RX_ENABLE_ENABLE, val));
	case AXI_FSRC_TX_ENABLE:
		if (!st->map[AXI_FSRC_TX])
			return -ENODEV;
		ret = regmap_read(st->map[AXI_FSRC_TX], REG_TX_ENABLE, &val);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%lu\n", FIELD_GET(REG_TX_ENABLE_ENABLE, val));
	case AXI_FSRC_TX_TRANSMIT:
		if (!st->map[AXI_FSRC_TX])
			return -ENODEV;
		return sysfs_emit(buf, "%x\n", st->tx_active);

	case AXI_FSRC_TX_RATIO_SET:
		return sysfs_emit(buf, "%u %u %u\n", st->n, st->m, st->ns);
	case AXI_FSRC_SEQ_START:
		return sysfs_emit(buf, "0\n");
	default:
		return -EINVAL;
	}
}

static ssize_t axi_fsrc_ext_write(struct iio_dev *indio_dev,
				  uintptr_t private,
				  const struct iio_chan_spec *chan,
				  const char *buf, size_t len)
{
	struct axi_fsrc *st = iio_priv(indio_dev);
	unsigned int n = 0, m = 0, ns = 1;
	bool enable;
	int size, ret = 0;

	switch ((u32)private) {
	case AXI_FSRC_RX_ENABLE:
	case AXI_FSRC_TX_ENABLE:
	case AXI_FSRC_TX_TRANSMIT:
	case AXI_FSRC_SEQ_START:
		ret = kstrtobool(buf, &enable);
		if (ret)
			return ret;
		break;
	case AXI_FSRC_TX_RATIO_SET:
		size = sscanf(buf, "%u %u %u", &n, &m, &ns);
		if (size < 2)
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
	case AXI_FSRC_TX_TRANSMIT:
		ret = axi_fsrc_tx_active(st, enable);
		break;
	case AXI_FSRC_TX_RATIO_SET:
		ret = axi_fsrc_tx_set_ratio(st, n, m, ns);
		break;
	case AXI_FSRC_SEQ_START:
		if (enable)
			ret = axi_fsrc_seq_start(st);
		break;
	}

	return ret ? ret : len;
}

#define AXI_FSRC_EXT_INFO(_name, _ident, _shared) { \
	.name = _name, \
	.read = axi_fsrc_ext_read, \
	.write = axi_fsrc_ext_write, \
	.private = _ident, \
	.shared = _shared, \
}

/* The sequencer aligns both directions, so seq_start is shared */
static const struct iio_chan_spec_ext_info axi_fsrc_tx_ext_info[] = {
	AXI_FSRC_EXT_INFO("enable", AXI_FSRC_TX_ENABLE, IIO_SEPARATE),
	AXI_FSRC_EXT_INFO("transmit", AXI_FSRC_TX_TRANSMIT, IIO_SEPARATE),
	AXI_FSRC_EXT_INFO("ratio_set", AXI_FSRC_TX_RATIO_SET, IIO_SEPARATE),
	AXI_FSRC_EXT_INFO("seq_start", AXI_FSRC_SEQ_START, IIO_SHARED_BY_ALL),
	{ }
};

static const struct iio_chan_spec_ext_info axi_fsrc_rx_ext_info[] = {
	AXI_FSRC_EXT_INFO("enable", AXI_FSRC_RX_ENABLE, IIO_SEPARATE),
	AXI_FSRC_EXT_INFO("seq_start", AXI_FSRC_SEQ_START, IIO_SHARED_BY_ALL),
	{ }
};

/* Channel index is the io-channels specifier: 0 = TX core, 1 = RX core */
static const struct iio_chan_spec axi_fsrc_chans[] = {
	{
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.channel = 0,
		.output = 1,
		.ext_info = axi_fsrc_tx_ext_info,
	}, {
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.channel = 1,
		.ext_info = axi_fsrc_rx_ext_info,
	},
};

static int axi_fsrc_debugfs_reg_access(struct iio_dev *indio_dev, unsigned int reg,
				       unsigned int writeval, unsigned int *readval)
{
	struct axi_fsrc *st = iio_priv(indio_dev);
	u8 core = reg >> 16;

	reg &= GENMASK(15, 0);
	if (core >= ARRAY_SIZE(st->map))
		return -EINVAL;
	if (!st->map[core])
		return -ENODEV;

	if (readval)
		return regmap_read(st->map[core], reg, readval);

	return regmap_write(st->map[core], reg, writeval);
}

static const struct iio_info axi_fsrc_info = {
	.debugfs_reg_access = &axi_fsrc_debugfs_reg_access,
};

static int axi_fsrc_sequencer_add_to_topology(struct device *dev,
					      struct axi_fsrc *st,
					      struct device_node *np)
{
	/* In the order of the st->map cores after AXI_FSRC_CTRL */
	static const char * const props[] = {
		"adi,fsrc_rx", "adi,fsrc_tx", "adi,fsrc_rx_a", "adi,fsrc_tx_a"
	};
	void __iomem *base;
	struct resource res;
	unsigned long roles = 0;
	int ret, i;

	for (i = 0; i < ARRAY_SIZE(props); i++) {
		if (!of_property_read_bool(np, props[i]))
			continue;
		if (st->map[i + 1])
			return dev_err_probe(dev, -EINVAL,
					     "%pOF: %s already assigned\n", np, props[i]);
		roles |= BIT(i);
	}

	if (!roles)
		return dev_err_probe(dev, -EINVAL, "%pOF: missing fsrc role\n", np);

	ret = of_address_to_resource(np, 0, &res);
	if (ret)
		return dev_err_probe(dev, ret, "%pOF: failed to get resource\n", np);

	base = devm_ioremap_resource(dev, &res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	for_each_set_bit(i, &roles, ARRAY_SIZE(props)) {
		ret = axi_fsrc_regmap_init(st, i + 1, base, resource_size(&res));
		if (ret)
			return ret;
	}

	return 0;
}

struct axi_fsrc_seq_count {
	u16 first_trig_cnt;
	u16 fsrc_accum_reset_cnt;
	u16 rx_delay_cnt;
};

static int axi_fsrc_seq_configure(struct axi_fsrc *st, const struct axi_fsrc_seq_count *count)
{
	struct regmap *map = st->map[AXI_FSRC_CTRL];
	int ret;

	ret = regmap_update_bits(map, REG_SEQ_CTRL_2, REG_SEQ_FIRST_TRIG_CNT,
				 FIELD_PREP(REG_SEQ_FIRST_TRIG_CNT,
					    SEQ_TRIG_CNT(count->first_trig_cnt)));
	if (ret)
		return ret;

	ret = regmap_update_bits(map, REG_SEQ_CTRL_5, REG_SEQ_SECOND_TRIG_CNT,
				 FIELD_PREP(REG_SEQ_SECOND_TRIG_CNT,
					    SEQ_TRIG_CNT(count->first_trig_cnt)));
	if (ret)
		return ret;

	ret = regmap_update_bits(map, REG_SEQ_CTRL_3, REG_SEQ_TX_ACCUM_RST_CNT,
				 FIELD_PREP(REG_SEQ_TX_ACCUM_RST_CNT,
					    count->fsrc_accum_reset_cnt));
	if (ret)
		return ret;

	ret = regmap_update_bits(map, REG_SEQ_CTRL_4, REG_SEQ_RX_DELAY,
				 FIELD_PREP(REG_SEQ_RX_DELAY, count->rx_delay_cnt));
	if (ret)
		return ret;

	return regmap_set_bits(map, REG_SEQ_CTRL_3, REG_SEQ_EN);
}

/*
 * Arm the TX FSRC to start on the sequencer's SYSREF-aligned tx_data_start.
 */
static int axi_fsrc_seq_start(struct axi_fsrc *st)
{
	struct regmap *tx = st->map[AXI_FSRC_TX];
	int ret;

	if (tx) {
		ret = regmap_write(tx, REG_CTRL_TRANSMIT, REG_CTRL_TRANSMIT_STOP);
		if (ret)
			return ret;

		ret = regmap_set_bits(tx, REG_TX_ENABLE, REG_TX_ENABLE_EXT_TRIG_EN);
		if (ret)
			return ret;

		ret = regmap_write(tx, REG_CTRL_TRANSMIT, REG_CTRL_TRANSMIT_ACCUM_SET);
		if (ret)
			return ret;

		st->tx_active = true;
	}

	ret = regmap_clear_bits(st->map[AXI_FSRC_CTRL], REG_SEQ_CTRL_3, REG_SEQ_START);
	if (ret)
		return ret;

	return regmap_set_bits(st->map[AXI_FSRC_CTRL], REG_SEQ_CTRL_3, REG_SEQ_START);
}

static int axi_fsrc_tx_configure(struct axi_fsrc *st)
{
	struct regmap *map = st->map[AXI_FSRC_TX];
	unsigned int val;
	int ret;

	if (!map)
		return 0;

	ret = regmap_read(map, REG_ACCUM_WIDTH, &val);
	if (ret)
		return ret;
	st->accum_width = val;

	ret = regmap_read(map, REG_NUM_SAMPLES, &val);
	if (ret)
		return ret;
	st->num_samples = val;

	if (!st->num_samples || st->num_samples > ACCUM_MAX_SLOTS) {
		dev_warn(st->dev, "TX FSRC has no programmable step, hole pattern only valid for 1:1\n");
		st->num_samples = 0;
	}
	return axi_fsrc_tx_set_ratio(st, 1, 1, 1);
}

static int axi_fsrc_init(struct axi_fsrc *st)
{
	/* Counts are in SYSREF periods, 4 bits each in the IP Core */
	static const struct axi_fsrc_seq_count count = {
		.first_trig_cnt = 2,
		.fsrc_accum_reset_cnt = 4,
		.rx_delay_cnt = 4
	};

	int ret;

	ret = axi_fsrc_seq_configure(st, &count);
	if (ret)
		return ret;

	return axi_fsrc_tx_configure(st);
}

static int axi_fsrc_sequencer_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *np_;
	struct iio_dev *indio_dev;
	struct axi_fsrc *st;
	void __iomem *base;
	struct resource *res;
	unsigned int val;
	int ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	indio_dev->info = &axi_fsrc_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = axi_fsrc_chans;
	indio_dev->num_channels = ARRAY_SIZE(axi_fsrc_chans);
	indio_dev->name = "axi_fsrc";
	st->dev = &pdev->dev;

	base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	ret = axi_fsrc_regmap_init(st, AXI_FSRC_CTRL, base, resource_size(res));
	if (ret)
		return ret;

	for (int i = 0; ; i++) {
		np_ = of_parse_phandle(np, "fsrc-topology", i);
		if (!np_)
			break;

		ret = axi_fsrc_sequencer_add_to_topology(&pdev->dev, st, np_);
		of_node_put(np_);
		if (ret)
			return ret;
	}

	ret = regmap_write(st->map[AXI_FSRC_CTRL], REG_SCRATCH, 0xBE);
	if (ret)
		return ret;

	ret = regmap_read(st->map[AXI_FSRC_CTRL], REG_SCRATCH, &val);
	if (ret)
		return ret;

	if (val != 0xBE)
		return dev_err_probe(&pdev->dev, -EINVAL, "Failed sanity test\n");

	ret = axi_fsrc_init(st);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, indio_dev);

	return devm_iio_device_register(&pdev->dev, indio_dev);
}

static const struct of_device_id axi_fsrc_sequencer_of_match[] = {
	{ .compatible = "adi,axi-fsrc-sequencer" },
	{ }
};
MODULE_DEVICE_TABLE(of, axi_fsrc_sequencer_of_match);

static struct platform_driver axi_fsrc_sequencer = {
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = axi_fsrc_sequencer_of_match,
	},
	.probe = axi_fsrc_sequencer_probe,
};
module_platform_driver(axi_fsrc_sequencer);

MODULE_AUTHOR("Jorge Marques <jorge.marques@analog.com>");
MODULE_DESCRIPTION("Analog Devices AXI FSRC Sequencer device driver");
MODULE_LICENSE("GPL");
