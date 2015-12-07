#include <linux/module.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/dmaengine.h>

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
#define M2K_LA_REG_TRIGGER_DELAY	0x28
#define M2K_LA_REG_TRIGGER_LOGIC_MODE	0x2c
#define M2K_LA_REG_CLOCK_SEL		0x30
#define M2K_LA_REG_GPO_EN		0x34
#define M2K_LA_REG_GPO			0x38
#define M2K_LA_REG_GPI			0x3c

#define M2K_LA_TRIGGER_EDGE_ANY		0
#define M2K_LA_TRIGGER_EDGE_RISING	1
#define M2K_LA_TRIGGER_EDGE_FALLING	2
#define M2K_LA_TRIGGER_LEVEL_LOW	3
#define M2K_LA_TRIGGER_LEVEL_HIGH	4

#define M2K_LA_TRIGGER_LOGIC_MODE_OR 0
#define M2K_LA_TRIGGER_LOGIC_MODE_AND 1

struct m2k_la {
	void __iomem *regs;

	struct clk *clk;

	struct mutex lock;
	unsigned int io_sel;
	unsigned int gpo;

	unsigned int trigger_logic_mode;
	unsigned int triggers[16];
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

	if (indio_dev->direction == IIO_DEVICE_DIRECTION_IN)
		reg = M2K_LA_REG_DIVIDER_LA;
	else
		reg = M2K_LA_REG_DIVIDER_PG;

	div = m2k_la_read(m2k_la, reg) + 1;

	*val = DIV_ROUND_CLOSEST(clk_get_rate(m2k_la->clk), div);

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
	if (indio_dev->direction == IIO_DEVICE_DIRECTION_IN)
		reg = M2K_LA_REG_DIVIDER_LA;
	else
		reg = M2K_LA_REG_DIVIDER_PG;

	m2k_la_write(m2k_la, reg, div - 1);

	return 0;
}

static void m2k_la_update_logic_mode(struct m2k_la *m2k_la)
{
	unsigned int mode;

	/*
	 * If all triggers are set to none switch to AND mode to always generate
	 * a event for each incoming sample.
	 */
	if (m2k_la->trigger_num_enabled == 0)
		mode = M2K_LA_TRIGGER_LOGIC_MODE_AND;
	else
		mode = m2k_la->trigger_logic_mode;

	m2k_la_write(m2k_la, M2K_LA_REG_TRIGGER_LOGIC_MODE, mode);
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

static ssize_t m2k_la_set_trigger_delay(struct iio_dev *indio_dev,
	uintptr_t priv, const struct iio_chan_spec *chan, const char *buf,
	size_t len)
{
	struct m2k_la *m2k_la = iio_device_get_drvdata(indio_dev);
	int ret, val;

	ret = kstrtoint(buf, 10, &val);
	if (ret < 0)
		return ret;

	if (val < -8192 || val > 8192)
		return -EINVAL;


	mutex_lock(&m2k_la->lock);
	m2k_la->trigger_delay = val;
	val += 8192;
	m2k_la_write(m2k_la, M2K_LA_REG_TRIGGER_DELAY, val);
	mutex_unlock(&m2k_la->lock);

	return len;
}

static ssize_t m2k_la_get_trigger_delay(struct iio_dev *indio_dev,
	uintptr_t priv, const struct iio_chan_spec *chan, char *buf)
{
	struct m2k_la *m2k_la = iio_device_get_drvdata(indio_dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", m2k_la->trigger_delay);
}

static const struct iio_chan_spec_ext_info m2k_la_rx_ext_info[] = {
	IIO_ENUM("trigger", IIO_SEPARATE, &m2k_la_trigger_enum),
	IIO_ENUM_AVAILABLE("trigger", &m2k_la_trigger_enum),
	IIO_ENUM("trigger_logic_mode", IIO_SHARED_BY_TYPE,
		&m2k_la_trigger_logic_mode_enum),
	IIO_ENUM_AVAILABLE("trigger_logic_mode",
		&m2k_la_trigger_logic_mode_enum),
	{
		.name = "trigger_delay",
		.shared = IIO_SHARED_BY_TYPE,
		.write = m2k_la_set_trigger_delay,
		.read = m2k_la_get_trigger_delay,
	},
	{}
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

static const char* const m2k_la_direction_items[] = {
	"in",
	"out"
};

static const struct iio_enum m2k_la_direction_enum = {
	.items = m2k_la_direction_items,
	.num_items = ARRAY_SIZE(m2k_la_direction_items),
	.set = m2k_la_set_direction,
	.get = m2k_la_get_direction,
};

static const struct iio_chan_spec_ext_info m2k_la_ext_info[] = {
	IIO_ENUM("direction", IIO_SEPARATE, &m2k_la_direction_enum),
	IIO_ENUM_AVAILABLE("direction", &m2k_la_direction_enum),
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

static int m2k_la_submit_block(struct iio_dma_buffer_queue *queue,
	struct iio_dma_buffer_block *block)
{
	struct iio_dev *indio_dev = queue->driver_data;

	if (indio_dev->direction == IIO_DEVICE_DIRECTION_IN) {
		block->block.bytes_used = block->block.size;
		iio_dmaengine_buffer_submit_block(queue, block, DMA_DEV_TO_MEM);
	} else {
		iio_dmaengine_buffer_submit_block(queue, block, DMA_MEM_TO_DEV);
	}

	return 0;
}

static const struct iio_dma_buffer_ops m2k_la_dma_buffer_ops = {
	.submit = m2k_la_submit_block,
	.abort = iio_dmaengine_buffer_abort,
};

static const struct iio_info m2k_la_iio_info = {
	.read_raw = m2k_la_read_raw,
	.write_raw = m2k_la_write_raw,
/*	.debugfs_reg_access = m2k_la_reg_access,*/
};

static const struct iio_info m2k_la_txrx_iio_info = {
	.read_raw = m2k_la_txrx_read_raw,
	.write_raw = m2k_la_txrx_write_raw,
};

static int m2k_la_probe(struct platform_device *pdev)
{
	struct iio_dev *indio_dev, *indio_dev_tx, *indio_dev_rx;
	struct iio_buffer *buffer_tx, *buffer_rx;
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

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	m2k_la->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(m2k_la->regs))
		return PTR_ERR(m2k_la->regs);

	/* Set all to input */
	m2k_la->io_sel = 0xffff;
	m2k_la_write(m2k_la, M2K_LA_REG_IO_SEL, m2k_la->io_sel);
	m2k_la_write(m2k_la, M2K_LA_REG_GPO_EN, 0xffff);

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
	indio_dev_tx->direction = IIO_DEVICE_DIRECTION_OUT;

	buffer_tx = iio_dmaengine_buffer_alloc(&pdev->dev, "tx",
			&m2k_la_dma_buffer_ops, indio_dev_tx);
	if (IS_ERR(buffer_tx))
		return PTR_ERR(buffer_tx);
	iio_device_attach_buffer(indio_dev_tx, buffer_tx);

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
	iio_device_attach_buffer(indio_dev_rx, buffer_rx);

	buffer_rx = iio_dmaengine_buffer_alloc(&pdev->dev, "rx",
			&m2k_la_dma_buffer_ops, indio_dev_rx);
	if (IS_ERR(buffer_rx))
		return PTR_ERR(buffer_rx);
	iio_device_attach_buffer(indio_dev_rx, buffer_rx);

	iio_device_set_drvdata(indio_dev_rx, m2k_la);

	ret = devm_iio_device_register(&pdev->dev, indio_dev_rx);
	if (ret)
		return ret;

	return 0;
}

static int m2k_la_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct m2k_la *m2k_la = iio_priv(indio_dev);

//	iio_dmaengine_buffer_free(indio_dev_rx->buffer);
//	iio_dmaengine_buffer_free(indio_dev_tx->buffer);
//	iio_device_unregister(indio_dev_rx);
//	iio_device_unregister(indio_dev_tx);
	iio_device_unregister(indio_dev);
	clk_disable_unprepare(m2k_la->clk);

	return 0;
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
	.remove = m2k_la_remove,
};
module_platform_driver(m2k_la_driver);

MODULE_AUTHOR("");
MODULE_DESCRIPTION("");
MODULE_LICENSE("GPL v2");
