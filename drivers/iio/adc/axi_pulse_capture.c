// SPDX-License-Identifier: GPL-2.0
/*
 * LASER HDL CORE driver
 *
 * Copyright 2016 Analog Devices Inc.
 */
#include <linux/clk.h>
#include <linux/fpga/adi-axi-common.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>

/* pulse generator register map */
#define ADI_REG_CONFIG		0x10
#define ADI_REG_PULSE_PERIOD	0x14
#define ADI_REG_PULSE_WIDTH	0x18
/* laser driver register map */
#define ADI_REG_DRIVER_ENABLE	0x80
#define ADI_REG_DRIVER_OTW	0x84
#define ADI_REG_IRQ_MASK	0xA0
#define ADI_REG_IRQ_SRC		0xA4
#define ADI_REG_IRQ_PENDING	0xA8
/* sequencer register map */
#define ADI_REG_SEQ_CTRL	0xAC
#define ADI_SEQ_CTRL_EN		BIT(0)
#define ADI_SEQ_CTRL_AUTO_EN	BIT(1)
#define ADI_REG_SEQ_OFFSET	0xB0
#define ADI_REG_SEQ_AUTO_CFG	0xB4
#define ADI_SEQ_VAL_MASK	0x03
#define ADI_SEQ_VAL_SHIFT(x)	((x) * 4)
#define ADI_REG_TIA_MANUAL_CFG	0xB8
#define ADI_TIA_SEL_MASK	0x03
#define ADI_TIA_SEL_SHIFT(x)	((x) * 4)

/* IRQ sources */
#define ADI_IRQ_SRC_OTW_ENTER	0x02
#define ADI_IRQ_SRC_OTW_EXIT	0x04
#define ADI_IRQ_SRC_PULSE	0x01
#define ADI_IRQ_MASK_OUT_ALL	0xFFFFFFFFU

#define LASER_ENABLE		0x00
#define LASER_DISABLE		0x01
#define PULSE_GEN_LOAD		0x02
#define PULSE_GEN_RESET		0x01

struct axi_pulse_capture {
	struct clk *clk;
	struct mutex pulse_lock;
	void __iomem *base;
	int irq;
	u32 version;
	bool otw;	/* over temperature warning flag */
	u32 saved_en;	/* saved enable state when entering OTW */
};

static inline void pulse_capture_iowrite(const struct axi_pulse_capture *pulse,
					 const u32 reg, const u32 value)
{
	iowrite32(value, pulse->base + reg);
}

static inline u32 pulse_capture_ioread(const struct axi_pulse_capture *pulse,
				       const u32 reg)
{
	return ioread32(pulse->base + reg);
}

static int axi_pulse_capture_params_set(struct axi_pulse_capture *pulse,
					const u32 reg,
					const unsigned long freq)
{
	unsigned long clk_rate = clk_get_rate(pulse->clk);
	u32 cnt = 0;

	if (!freq || !clk_rate)
		return -EINVAL;

	/*
	 * To counter is the desired period divided by the clock period.
	 * Since we are using frequency, the division is inverted...
	 */
	cnt = DIV_ROUND_CLOSEST(clk_rate, freq);

	pr_debug("Set value (%u), laser EXT Clk rate (%lu)\n",
		cnt, clk_rate);

	pulse_capture_iowrite(pulse, reg, cnt);
	/*
	 * Automatically reload the configurations when in enable state.
	 * Saves us from having to re-enable the device when changing the
	 * parameters (period or pulse width).
	 */
	mutex_lock(&pulse->pulse_lock);
	if (pulse_capture_ioread(pulse, ADI_REG_DRIVER_ENABLE)
	    == LASER_ENABLE) {
		pulse_capture_iowrite(pulse,
		ADI_REG_CONFIG, PULSE_GEN_LOAD);
	}
	mutex_unlock(&pulse->pulse_lock);

	return 0;
}

static ssize_t axi_pulse_capture_read_pulse_w(struct iio_dev *indio_dev,
	uintptr_t private, const struct iio_chan_spec *chan, char *buf)
{
	struct axi_pulse_capture *pulse = iio_priv(indio_dev);
	unsigned long clk_rate;
	u32 pulse_time, cnt;

	clk_rate = clk_get_rate(pulse->clk);
	if (!clk_rate) {
		pulse_time = 0;
	} else {
		cnt = pulse_capture_ioread(pulse, ADI_REG_PULSE_WIDTH);
		pulse_time = (1000000000U / clk_rate) * cnt;
	}

	return sprintf(buf, "%d\n", pulse_time);
}

static ssize_t axi_pulse_capture_write_pulse_w(struct iio_dev *indio_dev,
	 uintptr_t private, const struct iio_chan_spec *chan, const char *buf,
	 size_t len)
{
	struct axi_pulse_capture *pulse = iio_priv(indio_dev);
	int ret;
	unsigned long pulse_time = 0;

	ret = kstrtoul(buf, 0, &pulse_time);
	if (ret || !pulse_time) {
		ret = -EINVAL;
		goto exit;
	}

	/* The api expects hz, so let's pass ns to hz */
	ret = axi_pulse_capture_params_set(pulse, ADI_REG_PULSE_WIDTH,
					   (1000000000U/pulse_time));
exit:
	return ret ? ret : len;
}

static int axi_pulse_capture_read_raw(struct iio_dev *indio_dev,
				      struct iio_chan_spec const *chan,
				      int *val, int *val2, long mask)
{
	struct axi_pulse_capture *pulse = iio_priv(indio_dev);
	unsigned long clk_rate;
	u32 cnt;

	dev_dbg(indio_dev->dev.parent, "Read attr %ld\n", mask);

	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		*val = !pulse_capture_ioread(pulse, ADI_REG_DRIVER_ENABLE);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_FREQUENCY:
		clk_rate = clk_get_rate(pulse->clk);
		if (!clk_rate) {
			*val = 0;
		} else {
			cnt = pulse_capture_ioread(pulse, ADI_REG_PULSE_PERIOD);
			*val = DIV_ROUND_CLOSEST(clk_rate, cnt);
		}
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int axi_pulse_capture_write_raw(struct iio_dev *indio_dev,
				       struct iio_chan_spec const *chan,
				       int val, int val2, long mask)
{
	struct axi_pulse_capture *pulse = iio_priv(indio_dev);
	u32 config;

	dev_dbg(indio_dev->dev.parent, "IIO write attr %ld, val=%08X\n", mask,
			val);

	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		if (val == 1)
			val = LASER_ENABLE;
		else
			val = LASER_DISABLE;
		mutex_lock(&pulse->pulse_lock);
		if (pulse->otw) {
			dev_warn(indio_dev->dev.parent,
				 "Don't touch the enable register while in over temperature state!\n");
			mutex_unlock(&pulse->pulse_lock);
			return -EPERM;
		}
		/*
		 * When enabling the laser driver, we also have to
		 * load the configurations for the pulse generator
		 * (0x02): When disabling we have to reset the
		 * pulse generator (0x01)
		 */
		config = (val == LASER_ENABLE) ? PULSE_GEN_LOAD :
						PULSE_GEN_RESET;
		pulse_capture_iowrite(pulse, ADI_REG_CONFIG, config);
		pulse_capture_iowrite(pulse, ADI_REG_DRIVER_ENABLE, val);
		mutex_unlock(&pulse->pulse_lock);
		return 0;
	case IIO_CHAN_INFO_FREQUENCY:
		return axi_pulse_capture_params_set(pulse,
						    ADI_REG_PULSE_PERIOD, val);
	default:
		return -EINVAL;
	}
}

static int axi_pulse_capture_reg_access(struct iio_dev *indio_dev,
					unsigned int reg,
					unsigned int writeval,
					unsigned int *readval)
{
	struct axi_pulse_capture *pulse = iio_priv(indio_dev);
	int ret = 0;

	mutex_lock(&pulse->pulse_lock);
	if (!readval) {
		dev_dbg(indio_dev->dev.parent, "write addr 0x%08X, val=0x%08X\n",
				reg, writeval);
		if ((reg == ADI_REG_DRIVER_ENABLE || reg == ADI_REG_CONFIG)
		     && pulse->otw) {
			dev_warn(indio_dev->dev.parent,
				 "Don't touch the enable or config registers while in over temperature state!\n");
			ret = -EPERM;
			goto unlock;
		}
		pulse_capture_iowrite(pulse, reg, writeval);
	} else {
		*readval = pulse_capture_ioread(pulse, reg);
		dev_dbg(indio_dev->dev.parent, "read addr 0x%08X, val=0x%08X\n",
				reg, *readval);
	}
unlock:
	mutex_unlock(&pulse->pulse_lock);

	return ret;
}

static void axi_laser_irq_otw_state_exit(struct axi_pulse_capture *pulse)
{
	/* clear the irq */
	pulse_capture_iowrite(pulse, ADI_REG_IRQ_PENDING, ADI_IRQ_SRC_OTW_EXIT);
	pulse->otw = false;

	if (pulse->saved_en == LASER_DISABLE)
		return;

	pulse_capture_iowrite(pulse, ADI_REG_CONFIG, PULSE_GEN_LOAD);
	pulse_capture_iowrite(pulse, ADI_REG_DRIVER_ENABLE, LASER_ENABLE);
}

static void axi_laser_irq_otw_state_enter(struct axi_pulse_capture *pulse)
{
	/* save the current enable state */
	pulse->saved_en = pulse_capture_ioread(pulse, ADI_REG_DRIVER_ENABLE);
	pulse_capture_iowrite(pulse, ADI_REG_CONFIG, PULSE_GEN_RESET);
	pulse_capture_iowrite(pulse, ADI_REG_DRIVER_ENABLE, LASER_DISABLE);
	/* clear the irq */
	pulse_capture_iowrite(pulse, ADI_REG_IRQ_PENDING, ADI_IRQ_SRC_OTW_ENTER);
	pulse->otw = true;
}

static irqreturn_t axi_laser_fault_handler(int irq, void *data)
{
	struct iio_dev *indio_dev = (struct iio_dev *)data;
	struct axi_pulse_capture *pulse = iio_priv(indio_dev);
	u32 irq_pending = 0;

	mutex_lock(&pulse->pulse_lock);
	irq_pending = pulse_capture_ioread(pulse, ADI_REG_IRQ_PENDING);

	dev_warn_ratelimited(indio_dev->dev.parent,
			     "IRQ enter, pending:0x%02X, otw:%d, saved_en:%d\n",
			     irq_pending, pulse->otw, pulse->saved_en);

	if (pulse->otw) {
		/* we are waiting for an exit transition */
		if (!(irq_pending & ADI_IRQ_SRC_OTW_EXIT))
			goto unlock;

		axi_laser_irq_otw_state_exit(pulse);
	} else {
		/* we are waiting for an enter transition */
		if (!(irq_pending & ADI_IRQ_SRC_OTW_ENTER))
			goto unlock;

		axi_laser_irq_otw_state_enter(pulse);
	}
unlock:
	mutex_unlock(&pulse->pulse_lock);

	return IRQ_HANDLED;
}

static void axi_pulse_capture_config(struct axi_pulse_capture *pulse)
{
	u32 driver_otw;

	/* default period of 50 kHz */
	axi_pulse_capture_params_set(pulse, ADI_REG_PULSE_PERIOD, 50000);
	/* default pulse width of 20 ns (50Mhz) */
	axi_pulse_capture_params_set(pulse, ADI_REG_PULSE_WIDTH, 50000000);
	pulse_capture_iowrite(pulse, ADI_REG_CONFIG, PULSE_GEN_LOAD);
	pulse_capture_iowrite(pulse, ADI_REG_DRIVER_ENABLE, LASER_ENABLE);
	/*
	 * All interrupts are disabled on the core at startup.
	 * Let's only enable the otw irqs.
	 */
	driver_otw = pulse_capture_ioread(pulse, ADI_REG_DRIVER_OTW);
	pulse_capture_iowrite(pulse, ADI_REG_IRQ_PENDING,
		driver_otw ? ADI_IRQ_MASK_OUT_ALL : ADI_IRQ_SRC_OTW_EXIT);
	pulse_capture_iowrite(pulse, ADI_REG_IRQ_MASK,
			  (ADI_IRQ_MASK_OUT_ALL &
			  ~(ADI_IRQ_SRC_OTW_ENTER |
			    ADI_IRQ_SRC_OTW_EXIT)));

}

static const struct iio_chan_spec_ext_info ext_info = {
	.name = "pulse_width_ns",
	.read = axi_pulse_capture_read_pulse_w,
	.write = axi_pulse_capture_write_pulse_w,
};

static const struct iio_chan_spec axi_pulse_capture_channels[] = {
	{
		.type = IIO_ALTVOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_ENABLE) |
				BIT(IIO_CHAN_INFO_FREQUENCY),
		.channel = 0,
		.indexed = 1,
		.output = 1,
		.ext_info = &ext_info,
	},
};

enum axi_pulse_capture_iio_dev_attr {
	SEQUENCER_EN,
	SEQUENCER_MODE,
	SEQUENCER_PULSE_DELAY,
	SEQUENCER_AUTO_CFG,
	SEQUENCER_MANUAL_CHSEL,
};

static ssize_t axi_pulse_capture_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct axi_pulse_capture *pulse = iio_priv(indio_dev);
	unsigned long clk_rate, delay;
	u32 val, cnt;
	int ret = 0;

	switch ((u32)this_attr->address) {
	case SEQUENCER_EN:
		val = pulse_capture_ioread(pulse, ADI_REG_SEQ_CTRL);
		ret = sprintf(buf, "%ld\n", val & ADI_SEQ_CTRL_EN);
		break;
	case SEQUENCER_MODE:
		val = pulse_capture_ioread(pulse, ADI_REG_SEQ_CTRL);
		ret = sprintf(buf, "%s\n", val & ADI_SEQ_CTRL_AUTO_EN ?
			"auto" : "manual");
		break;
	case SEQUENCER_PULSE_DELAY:
		clk_rate = clk_get_rate(pulse->clk);
		if (!clk_rate) {
			delay = 0;
		} else {
			cnt = pulse_capture_ioread(pulse, ADI_REG_SEQ_OFFSET);
			delay = (1000000000U / clk_rate) * cnt;
		}
		ret = sprintf(buf, "%ld\n", delay);
		break;
	case SEQUENCER_AUTO_CFG:
		val = pulse_capture_ioread(pulse, ADI_REG_SEQ_AUTO_CFG);
		ret = sprintf(buf, "%d %d %d %d\n",
			(val >> ADI_SEQ_VAL_SHIFT(0)) & ADI_SEQ_VAL_MASK,
			(val >> ADI_SEQ_VAL_SHIFT(1)) & ADI_SEQ_VAL_MASK,
			(val >> ADI_SEQ_VAL_SHIFT(2)) & ADI_SEQ_VAL_MASK,
			(val >> ADI_SEQ_VAL_SHIFT(3)) & ADI_SEQ_VAL_MASK);
		break;
	case SEQUENCER_MANUAL_CHSEL:
		val = pulse_capture_ioread(pulse, ADI_REG_TIA_MANUAL_CFG);
		ret = sprintf(buf, "%d %d %d %d\n",
			(val >> ADI_TIA_SEL_SHIFT(0)) & ADI_TIA_SEL_MASK,
			(val >> ADI_TIA_SEL_SHIFT(1)) & ADI_TIA_SEL_MASK,
			(val >> ADI_TIA_SEL_SHIFT(2)) & ADI_TIA_SEL_MASK,
			(val >> ADI_TIA_SEL_SHIFT(3)) & ADI_TIA_SEL_MASK);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static ssize_t axi_pulse_capture_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct axi_pulse_capture *pulse = iio_priv(indio_dev);
	unsigned long clk_rate, delay;
	u32 val, cnt, cfg[4];
	bool en;
	int ret = 0;

	switch ((u32)this_attr->address) {
	case SEQUENCER_EN:
		ret = kstrtobool(buf, &en);
		if (ret)
			break;
		val = pulse_capture_ioread(pulse, ADI_REG_SEQ_CTRL);
		if (en)
			val |= ADI_SEQ_CTRL_EN;
		else
			val &= ~ADI_SEQ_CTRL_EN;
		pulse_capture_iowrite(pulse, ADI_REG_SEQ_CTRL, val);
		break;
	case SEQUENCER_MODE:
		val = pulse_capture_ioread(pulse, ADI_REG_SEQ_CTRL);
		if (sysfs_streq(buf, "auto"))
			val |= ADI_SEQ_CTRL_AUTO_EN;
		else if (sysfs_streq(buf, "manual"))
			val &= ~ADI_SEQ_CTRL_AUTO_EN;
		else
			break;
		pulse_capture_iowrite(pulse, ADI_REG_SEQ_CTRL, val);
		break;

	case SEQUENCER_PULSE_DELAY:
		ret = kstrtoul(buf, 0, &delay);
		if (ret)
			break;
		clk_rate = clk_get_rate(pulse->clk);
		if (!clk_rate)
			cnt = 0;
		else
			cnt = delay / (1000000000U / clk_rate);
		val = pulse_capture_ioread(pulse, ADI_REG_PULSE_PERIOD);
		if (cnt > val)
			cnt = val;
		pulse_capture_iowrite(pulse, ADI_REG_SEQ_OFFSET, cnt);
		break;
	case SEQUENCER_AUTO_CFG:
		ret = sscanf(buf, "%d %d %d %d",
			&cfg[0], &cfg[1], &cfg[2], &cfg[3]);
		if (ret == ARRAY_SIZE(cfg)) {
			ret = 0;
		} else {
			ret = -EINVAL;
			break;
		}
		val = ((cfg[0] & ADI_SEQ_VAL_MASK) << ADI_SEQ_VAL_SHIFT(0)) |
			((cfg[1] & ADI_SEQ_VAL_MASK) << ADI_SEQ_VAL_SHIFT(1)) |
			((cfg[2] & ADI_SEQ_VAL_MASK) << ADI_SEQ_VAL_SHIFT(2)) |
			((cfg[3] & ADI_SEQ_VAL_MASK) << ADI_SEQ_VAL_SHIFT(3));
		pulse_capture_iowrite(pulse, ADI_REG_SEQ_AUTO_CFG, val);
		break;
	case SEQUENCER_MANUAL_CHSEL:
		ret = sscanf(buf, "%d %d %d %d",
			&cfg[0], &cfg[1], &cfg[2], &cfg[3]);
		if (ret == 4) {
			ret = 0;
		} else {
			ret = -EINVAL;
			break;
		}
		val = ((cfg[0] & ADI_TIA_SEL_MASK) << ADI_TIA_SEL_SHIFT(0)) |
			((cfg[1] & ADI_TIA_SEL_MASK) << ADI_TIA_SEL_SHIFT(1)) |
			((cfg[2] & ADI_TIA_SEL_MASK) << ADI_TIA_SEL_SHIFT(2)) |
			((cfg[3] & ADI_TIA_SEL_MASK) << ADI_TIA_SEL_SHIFT(3));
		pulse_capture_iowrite(pulse, ADI_REG_TIA_MANUAL_CFG, val);
		break;
	default:
		ret = -EINVAL;
	}

	return ret ? ret : len;
}

static IIO_DEVICE_ATTR(sequencer_en, 0644,
	axi_pulse_capture_show,
	axi_pulse_capture_store,
	SEQUENCER_EN);

static IIO_CONST_ATTR_NAMED(sequencer_mode_available,
	sequencer_mode_available, "auto manual");

static IIO_DEVICE_ATTR(sequencer_mode, 0644,
	axi_pulse_capture_show,
	axi_pulse_capture_store,
	SEQUENCER_MODE);

static IIO_DEVICE_ATTR(sequencer_pulse_delay_ns, 0644,
	axi_pulse_capture_show,
	axi_pulse_capture_store,
	SEQUENCER_PULSE_DELAY);

static IIO_DEVICE_ATTR(sequencer_auto_cfg, 0644,
	axi_pulse_capture_show,
	axi_pulse_capture_store,
	SEQUENCER_AUTO_CFG);

static IIO_DEVICE_ATTR(sequencer_manual_chsel, 0644,
	axi_pulse_capture_show,
	axi_pulse_capture_store,
	SEQUENCER_MANUAL_CHSEL);

static struct attribute *axi_pulse_capture_attributes[] = {
	&iio_dev_attr_sequencer_en.dev_attr.attr,
	&iio_const_attr_sequencer_mode_available.dev_attr.attr,
	&iio_dev_attr_sequencer_mode.dev_attr.attr,
	&iio_dev_attr_sequencer_pulse_delay_ns.dev_attr.attr,
	&iio_dev_attr_sequencer_auto_cfg.dev_attr.attr,
	&iio_dev_attr_sequencer_manual_chsel.dev_attr.attr,
	NULL
};

static const struct attribute_group axi_pulse_capture_attribute_group = {
	.attrs = axi_pulse_capture_attributes,
};

static const struct iio_info axi_pulse_capture_info = {
	.debugfs_reg_access = &axi_pulse_capture_reg_access,
	.read_raw = axi_pulse_capture_read_raw,
	.write_raw = axi_pulse_capture_write_raw,
	.attrs = &axi_pulse_capture_attribute_group,
};

static const u32 version_1_0_0 = ADI_AXI_PCORE_VER(1, 0, 'a');

static const struct of_device_id axi_pulse_capture_of_match[] = {
	{ .compatible = "adi,axi-pulse-capture-1.00.a",
		.data = (void *)&version_1_0_0 },
	{},
};
MODULE_DEVICE_TABLE(of, axi_pulse_capture_of_match);

static int axi_pulse_capture_probe(struct platform_device *pdev)
{
	struct axi_pulse_capture *pulse = NULL;
	const struct of_device_id *id = NULL;
	struct iio_dev *indio_dev = NULL;
	struct resource *res = NULL;
	int ret = 0;

	dev_dbg(&pdev->dev, "ADI AXI_PULSE_CAPTURE probed \'%s\'\n",
					dev_name(&pdev->dev));

	id = of_match_node(axi_pulse_capture_of_match, pdev->dev.of_node);
	if (!id)
		return -EINVAL;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*pulse));
	if (!indio_dev) {
		dev_err(&pdev->dev, "Failed to allocated iio device\n");
		return -ENOMEM;
	}

	pulse = iio_priv(indio_dev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pulse->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pulse->base)) {
		dev_err(&pdev->dev, "ioremap failed with %ld\n",
			PTR_ERR(pulse->base));
		return PTR_ERR(pulse->base);
	}

	pulse->version = pulse_capture_ioread(pulse, ADI_AXI_REG_VERSION);
	if (ADI_AXI_PCORE_VER_MAJOR(pulse->version) !=
	    ADI_AXI_PCORE_VER_MAJOR((*(u32 *)id->data))) {
		dev_err(&pdev->dev, "Major version mismatch. Expected %d.%.2d.%c, Reported %d.%.2d.%c\n",
			ADI_AXI_PCORE_VER_MAJOR((*(u32 *)id->data)),
			ADI_AXI_PCORE_VER_MINOR((*(u32 *)id->data)),
			ADI_AXI_PCORE_VER_PATCH((*(u32 *)id->data)),
			ADI_AXI_PCORE_VER_MAJOR(pulse->version),
			ADI_AXI_PCORE_VER_MINOR(pulse->version),
			ADI_AXI_PCORE_VER_PATCH(pulse->version));
		return -ENODEV;
	}

	pulse->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(pulse->clk))
		return PTR_ERR(pulse->clk);

	pulse->irq = platform_get_irq(pdev, 0);
	if (pulse->irq < 0) {
		dev_err(&pdev->dev, "platform_get_irq failed with %d\n",
				pulse->irq);
		return pulse->irq;
	}

	ret = devm_request_threaded_irq(&pdev->dev, pulse->irq, NULL,
				axi_laser_fault_handler, IRQF_ONESHOT,
				"laser-driver-otw", indio_dev);
	if (ret) {
		dev_err(&pdev->dev, "failed to request an irq, %d", ret);
		return ret;
	}

	mutex_init(&pulse->pulse_lock);
	platform_set_drvdata(pdev, indio_dev);
	/* do initial configuration */
	axi_pulse_capture_config(pulse);

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = pdev->dev.of_node->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = axi_pulse_capture_channels;
	indio_dev->num_channels = ARRAY_SIZE(axi_pulse_capture_channels);
	indio_dev->info = &axi_pulse_capture_info;

	ret = devm_iio_device_register(&pdev->dev, indio_dev);
	if (ret < 0)
		return ret;

	dev_info(&pdev->dev,
		 "Analog Devices axi_pulse_capture (%d.%.2d.%c) at 0x%08llX mapped to 0x%p\n",
		 ADI_AXI_PCORE_VER_MAJOR(pulse->version),
		 ADI_AXI_PCORE_VER_MINOR(pulse->version),
		 ADI_AXI_PCORE_VER_PATCH(pulse->version),
		 (unsigned long long)res->start, pulse->base);

	return ret;
}

static struct platform_driver axi_pulse_capture_driver = {
	.driver = {
		.name = "axi-pulse-capture",
		.of_match_table = axi_pulse_capture_of_match,
	},
	.probe = axi_pulse_capture_probe,
};
module_platform_driver(axi_pulse_capture_driver);

MODULE_AUTHOR("Nuno Sa <nuno.sa@analog.com>");
MODULE_DESCRIPTION("Analog Devices Laser HDL CORE driver");
MODULE_LICENSE("GPL");
