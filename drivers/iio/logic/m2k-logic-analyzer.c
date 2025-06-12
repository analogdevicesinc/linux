#include <linux/module.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/dmaengine.h>
#include <linux/bitfield.h>
#include <linux/mod_devicetable.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>

#define M2K_LA_REG_VERSION		0x00
#define M2K_LA_REG_DIVIDER_LA		0x08
#define M2K_LA_REG_DIVIDER_PG		0x0c
#define M2K_LA_REG_IO_SEL		0x10
#define M2K_LA_REG_TRIGGER_ENABLE(x)	(0x14 + (x) * 4)
#define M2K_LA_REG_FIFO_DEPTH		0x28
#define M2K_LA_REG_TRIGGER_LOGIC_MODE	0x2c
#define M2K_LA_REG_CLOCK_SEL		0x30
#define M2K_LA_REG_GPO_EN		0x34
#define M2K_LA_REG_GPO			0x38
#define M2K_LA_REG_GPI			0x3c
#define M2K_LA_REG_OUTPUT_MODE		0x40
#define M2K_LA_REG_TRIGGER_DELAY	0x44
#define M2K_LA_REG_TRIGGERED		0x48
#define M2K_LA_REG_STREAMING		0x4c
#define M2K_LA_REG_INSTRUMENT_TRIGGER	0x54
#define M2K_LA_REG_DATA_DELAY_CONFIG	0x58

#define M2K_LA_TRIGGER_EDGE_ANY		0
#define M2K_LA_TRIGGER_EDGE_RISING	1
#define M2K_LA_TRIGGER_EDGE_FALLING	2
#define M2K_LA_TRIGGER_LEVEL_LOW	3
#define M2K_LA_TRIGGER_LEVEL_HIGH	4

#define M2K_LA_TRIGGER_LOGIC_MODE_OR 0
#define M2K_LA_TRIGGER_LOGIC_MODE_AND 1

#define M2K_LA_TRIGGER_CONDITION_MASK(x)	(0x155 << x)
#define M2K_LA_TRIGGER_SOURCE_MASK		GENMASK(19, 16)
#define M2K_LA_TRIGGER_EXT_SOURCE_MASK		GENMASK(17, 16)

#define M2K_LA_IN_DATA_DELAY_MASK		GENMASK(5, 0)
#define M2K_LA_IN_DATA_AUTO_DELAY_MASK		BIT(8)
#define M2K_LA_IN_DATA_DELAY_MUX_MASK		BIT(9)

struct m2k_la {
	void __iomem *regs;

	struct clk *clk;

	bool powerdown;
	unsigned int clocksource;

	struct mutex lock;
	unsigned int io_sel;
	unsigned int gpo;
	unsigned int output_mode;

	unsigned int trigger_logic_mode;
	unsigned int triggers[18];
	unsigned int trigger_regs[5];
	unsigned int trigger_num_enabled;
	int trigger_delay;
};

static void m2k_la_write(struct m2k_la *m2k_la, unsigned int reg,
	unsigned int val)
{
	writel_relaxed(val, m2k_la->regs + reg);
}

static unsigned int m2k_la_read(struct m2k_la *m2k_la, unsigned int reg)
{
	return readl_relaxed(m2k_la->regs + reg);
}

static void m2k_la_update(struct m2k_la *m2k_la, unsigned int reg,
			      unsigned int writeval, const unsigned int mask)
{
	unsigned int regval;

	regval = m2k_la_read(m2k_la, reg);
	regval &= ~mask;
	writeval &= mask;
	m2k_la_write(m2k_la, reg, writeval | regval);
}

static int m2k_la_read_raw(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, int *val, int *val2, long info)
{
	struct m2k_la *m2k_la = iio_priv(indio_dev);
	unsigned int ch_mask = BIT(chan->address);
	unsigned int raw;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&m2k_la->lock);
		if (m2k_la->io_sel & ch_mask)
			raw = m2k_la_read(m2k_la, M2K_LA_REG_GPI);
		else
			raw = m2k_la->gpo;
		mutex_unlock(&m2k_la->lock);

		*val = (raw >> chan->address) & 1;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = clk_get_rate(m2k_la->clk);
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int m2k_la_write_raw(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, int val, int val2, long info)
{
	struct m2k_la *m2k_la = iio_priv(indio_dev);
	unsigned int ch_mask = BIT(chan->address);

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&m2k_la->lock);
		if (val)
			m2k_la->gpo |= ch_mask;
		else
			m2k_la->gpo &= ~ch_mask;
		m2k_la_write(m2k_la, M2K_LA_REG_GPO, m2k_la->gpo);
		mutex_unlock(&m2k_la->lock);

		return 0;
	default:
		return -EINVAL;
	}
}

static int m2k_la_txrx_read_raw(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, int *val, int *val2, long info)
{
	struct m2k_la *m2k_la = iio_device_get_drvdata(indio_dev);
	unsigned int div, reg;

	if (indio_dev->buffer->direction == IIO_BUFFER_DIRECTION_IN)
		reg = M2K_LA_REG_DIVIDER_LA;
	else
		reg = M2K_LA_REG_DIVIDER_PG;

	div = m2k_la_read(m2k_la, reg) + 1;

	*val = DIV_ROUND_CLOSEST(clk_get_rate(m2k_la->clk), div*10) * 10;

	return IIO_VAL_INT;
}

static int m2k_la_txrx_write_raw(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, int val, int val2, long info)
{
	struct m2k_la *m2k_la = iio_device_get_drvdata(indio_dev);
	unsigned int div, reg;

	if (val <= 0)
		val = 1;

	div = DIV_ROUND_UP(clk_get_rate(m2k_la->clk), val);
	if (indio_dev->buffer->direction == IIO_BUFFER_DIRECTION_IN)
		reg = M2K_LA_REG_DIVIDER_LA;
	else
		reg = M2K_LA_REG_DIVIDER_PG;

	m2k_la_write(m2k_la, reg, div - 1);

	return 0;
}

static void m2k_la_update_logic_mode(struct m2k_la *m2k_la)
{
	unsigned int mode, mask;

	/*
	 * If all triggers are set to none switch to AND mode to always generate
	 * a event for each incoming sample.
	 */
	if (m2k_la->trigger_num_enabled == 0)
		mode = M2K_LA_TRIGGER_LOGIC_MODE_AND;
	else
		mode = m2k_la->trigger_logic_mode;

	mask = m2k_la_read(m2k_la, M2K_LA_REG_TRIGGER_LOGIC_MODE);
	mask &= ~BIT(0);

	m2k_la_write(m2k_la, M2K_LA_REG_TRIGGER_LOGIC_MODE, mask | mode);
}

static int m2k_la_set_trigger(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int val)
{
	struct m2k_la *m2k_la = iio_device_get_drvdata(indio_dev);
	unsigned int ch_mask = BIT(chan->address);
	unsigned int old;

	mutex_lock(&m2k_la->lock);
	old = m2k_la->triggers[chan->address];
	if (old == val) {
		mutex_unlock(&m2k_la->lock);
		return 0;
	}

	m2k_la->triggers[chan->address] = val;
	if (old != 0) {
		old -= 1;
		m2k_la->trigger_regs[old] &= ~ch_mask;
		m2k_la_write(m2k_la, M2K_LA_REG_TRIGGER_ENABLE(old),
			m2k_la->trigger_regs[old]);
		m2k_la->trigger_num_enabled--;
	}
	if (val != 0) {
		val -= 1;
		m2k_la->trigger_regs[val] |= ch_mask;
		m2k_la_write(m2k_la, M2K_LA_REG_TRIGGER_ENABLE(val),
			m2k_la->trigger_regs[val]);
		m2k_la->trigger_num_enabled++;
	}

	m2k_la_update_logic_mode(m2k_la);

	mutex_unlock(&m2k_la->lock);

	return 0;
}

static int m2k_la_get_trigger(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct m2k_la *m2k_la = iio_device_get_drvdata(indio_dev);

	return m2k_la->triggers[chan->address];
}

static const char * const m2k_la_trigger_items[] = {
	"none",
	"edge-any",
	"edge-rising",
	"edge-falling",
	"level-low",
	"level-high",
};

static const struct iio_enum m2k_la_trigger_enum = {
	.items = m2k_la_trigger_items,
	.num_items = ARRAY_SIZE(m2k_la_trigger_items),
	.set = m2k_la_set_trigger,
	.get = m2k_la_get_trigger,
};

static int m2k_la_set_trigger_logic_mode(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int val)
{
	struct m2k_la *m2k_la = iio_device_get_drvdata(indio_dev);

	mutex_lock(&m2k_la->lock);
	m2k_la->trigger_logic_mode = val;
	m2k_la_update_logic_mode(m2k_la);
	mutex_unlock(&m2k_la->lock);

	return 0;
}

static int m2k_la_get_trigger_logic_mode(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct m2k_la *m2k_la = iio_device_get_drvdata(indio_dev);

	return m2k_la->trigger_logic_mode;
}

static const char * const m2k_la_trigger_logic_mode_items[] = {
	"or",
	"and",
};

static const struct iio_enum m2k_la_trigger_logic_mode_enum = {
	.items = m2k_la_trigger_logic_mode_items,
	.num_items = ARRAY_SIZE(m2k_la_trigger_logic_mode_items),
	.set = m2k_la_set_trigger_logic_mode,
	.get = m2k_la_get_trigger_logic_mode,
};

static int m2k_la_set_trigger_mux_out(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int val)
{
	struct m2k_la *m2k_la = iio_device_get_drvdata(indio_dev);
	unsigned int mask;

	mutex_lock(&m2k_la->lock);
	mask = m2k_la_read(m2k_la, M2K_LA_REG_TRIGGER_LOGIC_MODE);
	mask &= ~GENMASK(6, 4);
	val = val << 4;
	m2k_la_write(m2k_la, M2K_LA_REG_TRIGGER_LOGIC_MODE, mask | val);
	mutex_unlock(&m2k_la->lock);

	return 0;
}

static int m2k_la_get_trigger_mux_out(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct m2k_la *m2k_la = iio_device_get_drvdata(indio_dev);
	unsigned int val;

	val = m2k_la_read(m2k_la, M2K_LA_REG_TRIGGER_LOGIC_MODE);
	return (val >> 4);
}

static const char * const m2k_la_trigger_mux_out_items[] = {
	"trigger-logic",
	"trigger-in",
	"trigger-logic-and-trigger-in",
	"trigger-logic-or-trigger-in",
	"trigger-logic-xor-trigger-in",
	"disabled"
};

static const struct iio_enum m2k_la_trigger_mux_out_enum = {
	.items = m2k_la_trigger_mux_out_items,
	.num_items = ARRAY_SIZE(m2k_la_trigger_mux_out_items),
	.set = m2k_la_set_trigger_mux_out,
	.get = m2k_la_get_trigger_mux_out,
};

static ssize_t m2k_la_set_trigger_delay(struct iio_dev *indio_dev,
	uintptr_t priv, const struct iio_chan_spec *chan, const char *buf,
	size_t len)
{
	struct m2k_la *m2k_la = iio_device_get_drvdata(indio_dev);
	int ret, val;

	ret = kstrtoint(buf, 10, &val);
	if (ret < 0)
		return ret;

	if (val < -8192)
		return -EINVAL;

	mutex_lock(&m2k_la->lock);
	if (val < 0) {
		m2k_la_write(m2k_la, M2K_LA_REG_FIFO_DEPTH, -val);
		m2k_la_write(m2k_la, M2K_LA_REG_TRIGGER_DELAY, 0);
	} else {
		m2k_la_write(m2k_la, M2K_LA_REG_FIFO_DEPTH, 0);
		m2k_la_write(m2k_la, M2K_LA_REG_TRIGGER_DELAY, val);
	}
	m2k_la->trigger_delay = val;
	mutex_unlock(&m2k_la->lock);

	return len;
}

static ssize_t m2k_la_get_trigger_delay(struct iio_dev *indio_dev,
	uintptr_t priv, const struct iio_chan_spec *chan, char *buf)
{
	struct m2k_la *m2k_la = iio_device_get_drvdata(indio_dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", m2k_la->trigger_delay);
}

static ssize_t m2k_la_set_triggered(struct iio_dev *indio_dev,
	uintptr_t priv, const struct iio_chan_spec *chan, const char *buf,
	size_t len)
{
	struct m2k_la *m2k_la = iio_device_get_drvdata(indio_dev);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 10, &val);
	if (ret < 0)
		return ret;

	m2k_la_write(m2k_la, M2K_LA_REG_TRIGGERED, val);

	return len;
}

static ssize_t m2k_la_get_triggered(struct iio_dev *indio_dev,
	uintptr_t priv, const struct iio_chan_spec *chan, char *buf)
{
	struct m2k_la *m2k_la = iio_device_get_drvdata(indio_dev);
	unsigned int val;

	val = m2k_la_read(m2k_la, M2K_LA_REG_TRIGGERED) & 1;

	return scnprintf(buf, PAGE_SIZE, "%d\n", val);
}

static int m2k_la_set_data_delay_auto(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int val)
{
	struct m2k_la *m2k_la = iio_device_get_drvdata(indio_dev);

	mutex_lock(&m2k_la->lock);

	m2k_la_update(m2k_la, M2K_LA_REG_DATA_DELAY_CONFIG,
		      FIELD_PREP(M2K_LA_IN_DATA_AUTO_DELAY_MASK, val),
		      M2K_LA_IN_DATA_AUTO_DELAY_MASK);

	mutex_unlock(&m2k_la->lock);

	return 0;
}

static int m2k_la_get_data_delay_auto(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct m2k_la *m2k_la = iio_device_get_drvdata(indio_dev);
	unsigned int val;

	val = m2k_la_read(m2k_la, M2K_LA_REG_DATA_DELAY_CONFIG);
	val = FIELD_GET(M2K_LA_IN_DATA_AUTO_DELAY_MASK, val);

	return val;
}

static const char * const m2k_la_data_delay_auto_items[] = {
	"auto",
	"manual"
};

static const struct iio_enum m2k_la_data_delay_auto_enum = {
	.items = m2k_la_data_delay_auto_items,
	.num_items = ARRAY_SIZE(m2k_la_data_delay_auto_items),
	.set = m2k_la_set_data_delay_auto,
	.get = m2k_la_get_data_delay_auto,
};

static int m2k_la_set_rate_mux(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int val)
{
	struct m2k_la *m2k_la = iio_device_get_drvdata(indio_dev);

	mutex_lock(&m2k_la->lock);

	m2k_la_update(m2k_la, M2K_LA_REG_DATA_DELAY_CONFIG,
		      FIELD_PREP(M2K_LA_IN_DATA_DELAY_MUX_MASK, val),
		      M2K_LA_IN_DATA_DELAY_MUX_MASK);

	mutex_unlock(&m2k_la->lock);

	return 0;
}

static int m2k_la_get_rate_mux(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct m2k_la *m2k_la = iio_device_get_drvdata(indio_dev);
	unsigned int val;

	val = m2k_la_read(m2k_la, M2K_LA_REG_DATA_DELAY_CONFIG);
	val = FIELD_GET(M2K_LA_IN_DATA_DELAY_MUX_MASK, val);

	return val;
}

static const char * const m2k_la_rate_mux_items[] = {
	"logic_analyzer",
	"oscilloscope"
};

static const struct iio_enum m2k_la_rate_mux_enum = {
	.items = m2k_la_rate_mux_items,
	.num_items = ARRAY_SIZE(m2k_la_rate_mux_items),
	.set = m2k_la_set_rate_mux,
	.get = m2k_la_get_rate_mux,
};

static ssize_t m2k_la_set_in_data_delay(struct iio_dev *indio_dev,
	uintptr_t priv, const struct iio_chan_spec *chan, const char *buf,
	size_t len)
{
	struct m2k_la *m2k_la = iio_device_get_drvdata(indio_dev);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 10, &val);
	if (ret < 0)
		return ret;

	mutex_lock(&m2k_la->lock);

	m2k_la_update(m2k_la, M2K_LA_REG_DATA_DELAY_CONFIG, val,
		      M2K_LA_IN_DATA_DELAY_MASK);

	mutex_unlock(&m2k_la->lock);

	return len;
}

static ssize_t m2k_la_get_in_data_delay(struct iio_dev *indio_dev,
	uintptr_t priv, const struct iio_chan_spec *chan, char *buf)
{
	struct m2k_la *m2k_la = iio_device_get_drvdata(indio_dev);
	unsigned int val;

	val = m2k_la_read(m2k_la, M2K_LA_REG_DATA_DELAY_CONFIG);
	val = FIELD_GET(M2K_LA_IN_DATA_DELAY_MASK, val);

	return scnprintf(buf, PAGE_SIZE, "%u\n", val);
}

static ssize_t m2k_la_get_streaming(struct iio_dev *indio_dev,
	uintptr_t priv, const struct iio_chan_spec *chan, char *buf)
{
	struct m2k_la *m2k_la = iio_device_get_drvdata(indio_dev);
	unsigned int val;

	val = m2k_la_read(m2k_la, M2K_LA_REG_STREAMING) & 1;

	return scnprintf(buf, PAGE_SIZE, "%d\n", val);
}

static ssize_t m2k_la_set_streaming(struct iio_dev *indio_dev,
	uintptr_t priv, const struct iio_chan_spec *chan, const char *buf,
	size_t len)
{
	struct m2k_la *m2k_la = iio_device_get_drvdata(indio_dev);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 10, &val);
	if (ret < 0)
		return ret;

	m2k_la_write(m2k_la, M2K_LA_REG_STREAMING, val);

	return len;
}

static const struct iio_chan_spec_ext_info m2k_la_rx_ext_info[] = {
	IIO_ENUM("trigger", IIO_SEPARATE, &m2k_la_trigger_enum),
	IIO_ENUM_AVAILABLE("trigger", IIO_SHARED_BY_TYPE, &m2k_la_trigger_enum),
	IIO_ENUM("trigger_logic_mode", IIO_SHARED_BY_TYPE,
		&m2k_la_trigger_logic_mode_enum),
	IIO_ENUM_AVAILABLE("trigger_logic_mode", IIO_SHARED_BY_TYPE,
		&m2k_la_trigger_logic_mode_enum),
	IIO_ENUM("trigger_mux_out", IIO_SHARED_BY_TYPE,
		&m2k_la_trigger_mux_out_enum),
	IIO_ENUM_AVAILABLE("trigger_mux_out", IIO_SHARED_BY_TYPE,
		&m2k_la_trigger_mux_out_enum),
	IIO_ENUM("data_delay_auto", IIO_SHARED_BY_ALL,
		&m2k_la_data_delay_auto_enum),
	IIO_ENUM_AVAILABLE("data_delay_auto", IIO_SHARED_BY_ALL,
		&m2k_la_data_delay_auto_enum),
	IIO_ENUM("rate_mux", IIO_SHARED_BY_ALL,
		&m2k_la_rate_mux_enum),
	IIO_ENUM_AVAILABLE("rate_mux", IIO_SHARED_BY_ALL,
		&m2k_la_rate_mux_enum),
	{
		.name = "trigger_delay",
		.shared = IIO_SHARED_BY_TYPE,
		.write = m2k_la_set_trigger_delay,
		.read = m2k_la_get_trigger_delay,
	},
	{
		.name = "triggered",
		.shared = IIO_SHARED_BY_ALL,
		.read = m2k_la_get_triggered,
		.write = m2k_la_set_triggered,
	},
	{
		.name = "data_in_delay",
		.shared = IIO_SHARED_BY_ALL,
		.read = m2k_la_get_in_data_delay,
		.write = m2k_la_set_in_data_delay,
	},
	{
		.name = "streaming",
		.shared = IIO_SHARED_BY_ALL,
		.write = m2k_la_set_streaming,
		.read = m2k_la_get_streaming,
	},
	{}
};

static int m2k_la_set_trig_condition(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int val)
{
	struct m2k_la *m2k_la = iio_device_get_drvdata(indio_dev);
	int trig_src;

	mutex_lock(&m2k_la->lock);
	/* Read Trig SRC */
	trig_src = m2k_la_read(m2k_la, M2K_LA_REG_INSTRUMENT_TRIGGER);
	trig_src = FIELD_GET(M2K_LA_TRIGGER_EXT_SOURCE_MASK, trig_src);

	if (trig_src) {
		trig_src -= 1;
		if (val)
			/* Subtract 1 because of the none item. Multiply with 2
			 * and shift with trig_src due to channel selection
			 */
			val = BIT(2 * (val - 1)) << trig_src;

		m2k_la_update(m2k_la, M2K_LA_REG_INSTRUMENT_TRIGGER, val,
				  M2K_LA_TRIGGER_CONDITION_MASK(trig_src));
	}

	mutex_unlock(&m2k_la->lock);

	return 0;
}

static int m2k_la_get_trig_condition(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct m2k_la *m2k_la = iio_device_get_drvdata(indio_dev);
	int val, trig_src;

	/* Read Trig SRC */
	trig_src = val = m2k_la_read(m2k_la, M2K_LA_REG_INSTRUMENT_TRIGGER);
	trig_src = FIELD_GET(M2K_LA_TRIGGER_EXT_SOURCE_MASK, trig_src);

	if (trig_src) {
		trig_src -= 1;
		val &= M2K_LA_TRIGGER_CONDITION_MASK(trig_src);
		return ((fls(val >> trig_src) + 1) / 2);
	}

	return 0;
}

static const char * const m2k_la_trigger_tx_items[] = {
	"none",
	"level-low",
	"level-high",
	"edge-any",
	"edge-rising",
	"edge-falling",
};

static const struct iio_enum m2k_la_trig_condition_enum = {
	.items = m2k_la_trigger_tx_items,
	.num_items = ARRAY_SIZE(m2k_la_trigger_tx_items),
	.set = m2k_la_set_trig_condition,
	.get = m2k_la_get_trig_condition,
};

static int m2k_la_set_trig_src(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int val)
{
	struct m2k_la *m2k_la = iio_device_get_drvdata(indio_dev);

	mutex_lock(&m2k_la->lock);

	/* reset required by HDL */
	m2k_la_update(m2k_la, M2K_LA_REG_INSTRUMENT_TRIGGER, 0x0,
			   M2K_LA_TRIGGER_SOURCE_MASK);

	val = BIT(val) << 15;
	m2k_la_update(m2k_la, M2K_LA_REG_INSTRUMENT_TRIGGER, val,
			   M2K_LA_TRIGGER_SOURCE_MASK);

	mutex_unlock(&m2k_la->lock);

	return 0;
}

static int m2k_la_get_trig_src(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct m2k_la *m2k_la = iio_device_get_drvdata(indio_dev);
	int val;

	val = m2k_la_read(m2k_la, M2K_LA_REG_INSTRUMENT_TRIGGER);

	if (val & M2K_LA_TRIGGER_SOURCE_MASK)
		return fls(val) - 16;

	return 0;
}

static const char * const m2k_la_trig_src_items[] = {
	"none",
	"trigger-i_0",
	"trigger-i_1",
	"trigger-adc",
	"trigger-la",
};

static const struct iio_enum m2k_la_trig_src_enum = {
	.items = m2k_la_trig_src_items,
	.num_items = ARRAY_SIZE(m2k_la_trig_src_items),
	.set = m2k_la_set_trig_src,
	.get = m2k_la_get_trig_src,
};

static const struct iio_chan_spec_ext_info m2k_la_tx_ext_info[] = {
	IIO_ENUM_AVAILABLE("trigger_src", IIO_SHARED_BY_ALL,
		&m2k_la_trig_src_enum),
	IIO_ENUM("trigger_src", IIO_SHARED_BY_ALL, &m2k_la_trig_src_enum),
	IIO_ENUM_AVAILABLE("trigger_condition", IIO_SHARED_BY_ALL,
		&m2k_la_trig_condition_enum),
	IIO_ENUM("trigger_condition", IIO_SHARED_BY_ALL,
		&m2k_la_trig_condition_enum),
	{ },
};

static int m2k_la_set_direction(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int val)
{
	struct m2k_la *m2k_la = iio_priv(indio_dev);

	mutex_lock(&m2k_la->lock);
	if (val)
		m2k_la->io_sel &= ~BIT(chan->address);
	else
		m2k_la->io_sel |= BIT(chan->address);
	m2k_la_write(m2k_la, M2K_LA_REG_IO_SEL, m2k_la->io_sel);
	mutex_unlock(&m2k_la->lock);

	return 0;
}

static int m2k_la_get_direction(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct m2k_la *m2k_la = iio_priv(indio_dev);

	if (m2k_la->io_sel & BIT(chan->address))
		return 0;
	else
		return 1;
}

static const char *const m2k_la_direction_items[] = {
	"in",
	"out"
};

static const struct iio_enum m2k_la_direction_enum = {
	.items = m2k_la_direction_items,
	.num_items = ARRAY_SIZE(m2k_la_direction_items),
	.set = m2k_la_set_direction,
	.get = m2k_la_get_direction,
};

static ssize_t m2k_la_read_powerdown(struct iio_dev *indio_dev,
	uintptr_t private, const struct iio_chan_spec *chan, char *buf)
{
	struct m2k_la *m2k_la = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", m2k_la->powerdown);
}

static ssize_t m2k_la_write_powerdown(struct iio_dev *indio_dev,
	 uintptr_t private, const struct iio_chan_spec *chan, const char *buf,
	 size_t len)
{
	struct m2k_la *m2k_la = iio_priv(indio_dev);
	bool powerdown;
	int ret;

	ret = strtobool(buf, &powerdown);
	if (ret)
		return ret;

	mutex_lock(&m2k_la->lock);

	if (powerdown == m2k_la->powerdown)
		goto out_unlock;

	if (powerdown)
		clk_disable_unprepare(m2k_la->clk);
	else
		clk_prepare_enable(m2k_la->clk);

	m2k_la->powerdown = powerdown;

out_unlock:
	mutex_unlock(&m2k_la->lock);

	return len;
}

static int m2k_la_set_output_mode(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int val)
{
	struct m2k_la *m2k_la = iio_priv(indio_dev);

	mutex_lock(&m2k_la->lock);
	m2k_la->output_mode &= ~BIT(chan->address);
	m2k_la->output_mode |= val << chan->address;
	m2k_la_write(m2k_la, M2K_LA_REG_OUTPUT_MODE, m2k_la->output_mode);
	mutex_unlock(&m2k_la->lock);

	return 0;
}

static int m2k_la_get_output_mode(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct m2k_la *m2k_la = iio_priv(indio_dev);

	return (m2k_la->output_mode >> chan->address) & 1;
}

static const char * const m2k_la_output_mode_items[] = {
	"push-pull",
	"open-drain",
};

static const struct iio_enum m2k_la_output_mode_enum = {
	.items = m2k_la_output_mode_items,
	.num_items = ARRAY_SIZE(m2k_la_output_mode_items),
	.set = m2k_la_set_output_mode,
	.get = m2k_la_get_output_mode,
};

static int m2k_la_set_clocksource(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int val)
{
	struct m2k_la *m2k_la = iio_priv(indio_dev);

	mutex_lock(&m2k_la->lock);
	m2k_la_write(m2k_la, M2K_LA_REG_CLOCK_SEL, val);
	m2k_la->clocksource = val;
	mutex_unlock(&m2k_la->lock);

	return 0;
}

static int m2k_la_get_clocksource(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct m2k_la *m2k_la = iio_priv(indio_dev);

	return m2k_la->clocksource;
}

static const char * const m2k_la_clocksource_items[] = {
	"internal",
	"external",
};

static const struct iio_enum m2k_la_clocksource_enum = {
	.items = m2k_la_clocksource_items,
	.num_items = ARRAY_SIZE(m2k_la_clocksource_items),
	.set = m2k_la_set_clocksource,
	.get = m2k_la_get_clocksource,
};

static const struct iio_chan_spec_ext_info m2k_la_ext_info[] = {
	{
		.name = "powerdown",
		.read = m2k_la_read_powerdown,
		.write = m2k_la_write_powerdown,
		.shared = IIO_SHARED_BY_ALL,
	},
	IIO_ENUM("clocksource", IIO_SHARED_BY_ALL, &m2k_la_clocksource_enum),
	IIO_ENUM_AVAILABLE("clocksource", IIO_SHARED_BY_ALL,
		&m2k_la_clocksource_enum),

	IIO_ENUM("direction", IIO_SEPARATE, &m2k_la_direction_enum),
	IIO_ENUM_AVAILABLE("direction", IIO_SHARED_BY_TYPE, &m2k_la_direction_enum),
	IIO_ENUM("outputmode", IIO_SEPARATE, &m2k_la_output_mode_enum),
	IIO_ENUM_AVAILABLE("outputmode", IIO_SHARED_BY_TYPE, &m2k_la_output_mode_enum),
	{}
};

#define M2K_LA_TX_CHAN(x) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.channel = (x), \
	.address = (x), \
	.output = 1, \
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ), \
	.scan_index = 0, \
	.ext_info = m2k_la_tx_ext_info, \
	.scan_type = { \
		.sign = 'u', \
		.realbits = 1, \
		.storagebits = 16, \
		.shift = (x), \
	}, \
}

#define M2K_LA_RX_CHAN(x) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.channel = (x), \
	.address = (x), \
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ), \
	.scan_index = 0, \
	.ext_info = m2k_la_rx_ext_info, \
	.scan_type = { \
		.sign = 'u', \
		.realbits = 1, \
		.storagebits = 16, \
		.shift = (x), \
	}, \
}

#define M2K_LA_RX_CHAN_TRIGGER(x) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.channel = (x), \
	.address = (x), \
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ), \
	.scan_index = -1, \
	.ext_info = m2k_la_rx_ext_info, \
}

#define M2K_LA_CHAN(x) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.channel = (x), \
	.address = (x), \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ), \
	.scan_index = 0, \
	.ext_info = m2k_la_ext_info, \
	.scan_type = { \
		.sign = 'u', \
		.realbits = 1, \
		.storagebits = 16, \
		.shift = (x), \
	}, \
}

static const struct iio_chan_spec m2k_la_chan_spec[] = {
	M2K_LA_CHAN(0),
	M2K_LA_CHAN(1),
	M2K_LA_CHAN(2),
	M2K_LA_CHAN(3),
	M2K_LA_CHAN(4),
	M2K_LA_CHAN(5),
	M2K_LA_CHAN(6),
	M2K_LA_CHAN(7),
	M2K_LA_CHAN(8),
	M2K_LA_CHAN(9),
	M2K_LA_CHAN(10),
	M2K_LA_CHAN(11),
	M2K_LA_CHAN(12),
	M2K_LA_CHAN(13),
	M2K_LA_CHAN(14),
	M2K_LA_CHAN(15),
};

static const struct iio_chan_spec m2k_la_tx_chan_spec[] = {
	M2K_LA_TX_CHAN(0),
	M2K_LA_TX_CHAN(1),
	M2K_LA_TX_CHAN(2),
	M2K_LA_TX_CHAN(3),
	M2K_LA_TX_CHAN(4),
	M2K_LA_TX_CHAN(5),
	M2K_LA_TX_CHAN(6),
	M2K_LA_TX_CHAN(7),
	M2K_LA_TX_CHAN(8),
	M2K_LA_TX_CHAN(9),
	M2K_LA_TX_CHAN(10),
	M2K_LA_TX_CHAN(11),
	M2K_LA_TX_CHAN(12),
	M2K_LA_TX_CHAN(13),
	M2K_LA_TX_CHAN(14),
	M2K_LA_TX_CHAN(15),
};

static const struct iio_chan_spec m2k_la_rx_chan_spec[] = {
	M2K_LA_RX_CHAN(0),
	M2K_LA_RX_CHAN(1),
	M2K_LA_RX_CHAN(2),
	M2K_LA_RX_CHAN(3),
	M2K_LA_RX_CHAN(4),
	M2K_LA_RX_CHAN(5),
	M2K_LA_RX_CHAN(6),
	M2K_LA_RX_CHAN(7),
	M2K_LA_RX_CHAN(8),
	M2K_LA_RX_CHAN(9),
	M2K_LA_RX_CHAN(10),
	M2K_LA_RX_CHAN(11),
	M2K_LA_RX_CHAN(12),
	M2K_LA_RX_CHAN(13),
	M2K_LA_RX_CHAN(14),
	M2K_LA_RX_CHAN(15),
	M2K_LA_RX_CHAN_TRIGGER(16),
	M2K_LA_RX_CHAN_TRIGGER(17),
};

static int m2k_la_reg_access(struct iio_dev *indio_dev, unsigned int reg,
	unsigned int writeval, unsigned int *readval)
{
	struct m2k_la *m2k_la = iio_priv(indio_dev);

	reg &= 0xffff;

	if (readval == NULL)
		m2k_la_write(m2k_la, reg, writeval);
	else
		*readval = m2k_la_read(m2k_la, reg);

	return 0;
}

static const struct iio_info m2k_la_iio_info = {
	.read_raw = m2k_la_read_raw,
	.write_raw = m2k_la_write_raw,
	.debugfs_reg_access = m2k_la_reg_access,
};

static const struct iio_info m2k_la_txrx_iio_info = {
	.read_raw = m2k_la_txrx_read_raw,
	.write_raw = m2k_la_txrx_write_raw,
};

static int m2k_la_tx_preenable(struct iio_dev *indio_dev)
{
	struct m2k_la *m2k_la = iio_device_get_drvdata(indio_dev);
	uint32_t mask = *indio_dev->buffer->channel_mask;

	m2k_la_write(m2k_la, M2K_LA_REG_GPO_EN, ~mask);
	return 0;
}

static int m2k_la_tx_postdisable(struct iio_dev *indio_dev)
{
	struct m2k_la *m2k_la = iio_device_get_drvdata(indio_dev);

	m2k_la_write(m2k_la, M2K_LA_REG_GPO_EN, 0xffff);
	return 0;
}

static const struct iio_buffer_setup_ops m2k_la_tx_setup_ops = {
	.preenable = m2k_la_tx_preenable,
	.postdisable = m2k_la_tx_postdisable,
};

static void m2k_la_disable_clk(void *data)
{
	struct m2k_la *m2k_la = data;

	if (!m2k_la->powerdown)
		clk_disable_unprepare(m2k_la->clk);
}

static int m2k_la_probe(struct platform_device *pdev)
{
	struct iio_dev *indio_dev, *indio_dev_tx, *indio_dev_rx;
	struct m2k_la *m2k_la;
	struct resource *mem;
	int ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*m2k_la));
	if (!indio_dev)
		return -ENOMEM;

	indio_dev_tx = devm_iio_device_alloc(&pdev->dev, 0);
	if (!indio_dev)
		return -ENOMEM;

	indio_dev_rx = devm_iio_device_alloc(&pdev->dev, 0);
	if (!indio_dev)
		return -ENOMEM;

	m2k_la = iio_priv(indio_dev);
	mutex_init(&m2k_la->lock);

	m2k_la->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(m2k_la->clk))
		return PTR_ERR(m2k_la->clk);

	ret = clk_prepare_enable(m2k_la->clk);
	if (ret < 0)
		return -EINVAL;

	m2k_la->powerdown = false;

	ret = devm_add_action_or_reset(&pdev->dev, m2k_la_disable_clk, m2k_la);
	if (ret)
		return ret;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	m2k_la->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(m2k_la->regs))
		return PTR_ERR(m2k_la->regs);

	/* Set all to input */
	m2k_la->io_sel = 0xffff;
	m2k_la_write(m2k_la, M2K_LA_REG_IO_SEL, m2k_la->io_sel);
	m2k_la_write(m2k_la, M2K_LA_REG_GPO_EN, 0xffff);
	m2k_la_write(m2k_la, M2K_LA_REG_OUTPUT_MODE, 0x0);

	m2k_la_write(m2k_la, M2K_LA_REG_CLOCK_SEL, 0x0);
	m2k_la_write(m2k_la, M2K_LA_REG_FIFO_DEPTH, 0x0);
	m2k_la_write(m2k_la, M2K_LA_REG_TRIGGER_DELAY, 0x0);

	m2k_la_update_logic_mode(m2k_la);

	platform_set_drvdata(pdev, indio_dev);

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = "m2k-logic-analyzer";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &m2k_la_iio_info;
	indio_dev->channels = m2k_la_chan_spec,
	indio_dev->num_channels = ARRAY_SIZE(m2k_la_chan_spec);

	ret = devm_iio_device_register(&pdev->dev, indio_dev);
	if (ret)
		return ret;

	indio_dev_tx->dev.parent = &pdev->dev;
	indio_dev_tx->name = "m2k-logic-analyzer-tx";
	indio_dev_tx->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_HARDWARE;
	indio_dev_tx->info = &m2k_la_txrx_iio_info;
	indio_dev_tx->channels = m2k_la_tx_chan_spec,
	indio_dev_tx->num_channels = ARRAY_SIZE(m2k_la_tx_chan_spec);
	indio_dev_tx->setup_ops = &m2k_la_tx_setup_ops;

	ret = devm_iio_dmaengine_buffer_setup(&pdev->dev, indio_dev_tx, "tx",
					      IIO_BUFFER_DIRECTION_OUT);
	if (ret)
		return ret;

	iio_device_set_drvdata(indio_dev_tx, m2k_la);

	ret = devm_iio_device_register(&pdev->dev, indio_dev_tx);
	if (ret)
		return ret;

	indio_dev_rx->dev.parent = &pdev->dev;
	indio_dev_rx->name = "m2k-logic-analyzer-rx";
	indio_dev_rx->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_HARDWARE;
	indio_dev_rx->info = &m2k_la_txrx_iio_info;
	indio_dev_rx->channels = m2k_la_rx_chan_spec,
	indio_dev_rx->num_channels = ARRAY_SIZE(m2k_la_rx_chan_spec);

	ret = devm_iio_dmaengine_buffer_setup(&pdev->dev, indio_dev_rx, "rx",
					      IIO_BUFFER_DIRECTION_IN);
	if (ret)
		return ret;

	iio_device_set_drvdata(indio_dev_rx, m2k_la);

	return devm_iio_device_register(&pdev->dev, indio_dev_rx);
}

static const struct of_device_id m2k_la_of_match[] = {
	{ .compatible = "adi,m2k-logic-analyzer" },
	{},
};

static struct platform_driver m2k_la_driver = {
	.driver = {
		.name = "m2k_logic_analyzer",
		.of_match_table = m2k_la_of_match,
	},
	.probe = m2k_la_probe,
};
module_platform_driver(m2k_la_driver);

MODULE_AUTHOR("");
MODULE_DESCRIPTION("");
MODULE_LICENSE("GPL v2");
MODULE_IMPORT_NS(IIO_DMAENGINE_BUFFER);
