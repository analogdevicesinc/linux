/*
 * Analog Devices ADC Module
 *
 * Copyright 2013 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

/**
 * Note:
 * This driver is an old copy from the cf_axi_adc/axi-adc driver.
 * And some things were common with that driver. The cf_axi_adc/axi-adc
 * driver is a more complete implementation, while this one is just caring
 * about Motor Control.
 * The code duplication [here] is intentional, as we try to cleanup the
 * AXI ADC and decouple it from this driver.
 */

#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/dmaengine.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/units.h>

#include <linux/clk.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/hw-consumer.h>

#include <linux/dma-direction.h>
#include <linux/iio/buffer_impl.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/jesd204/adi-common.h>

/* ADC Common */
#define ADI_REG_RSTN			0x0040
#define ADI_RSTN			(1 << 0)

#define ADI_REG_STATUS			0x005C
#define ADI_REG_DMA_STATUS		0x0088
#define ADI_REG_USR_CNTRL_1		0x00A0

/* ADC Channel */
#define ADI_REG_CHAN_CNTRL(c)		(0x0400 + (c) * 0x40)
#define ADI_IQCOR_ENB			(1 << 9)
#define ADI_FORMAT_SIGNEXT		(1 << 6)
#define ADI_FORMAT_ENABLE		(1 << 4)
#define ADI_ENABLE			(1 << 0)

#define ADI_REG_CHAN_CNTRL_2(c)		(0x0414 + (c) * 0x40)

#define ADI_REG_CORRECTION_ENABLE 0x48
#define ADI_REG_CORRECTION_COEFFICIENT(x) (0x4c + (x) * 4)

#define ADI_USR_CHANMAX(x)		(((x) & 0xFF) << 0)
#define ADI_TO_USR_CHANMAX(x)		(((x) >> 0) & 0xFF)

#define ADI_REG_CHAN_USR_CNTRL_1(c)		(0x0420 + (c) * 0x40)
#define ADI_USR_DATATYPE_BE			(1 << 25)
#define ADI_USR_DATATYPE_SIGNED			(1 << 24)
#define ADI_USR_DATATYPE_SHIFT(x)		(((x) & 0xFF) << 16)
#define ADI_TO_USR_DATATYPE_SHIFT(x)		(((x) >> 16) & 0xFF)
#define ADI_USR_DATATYPE_TOTAL_BITS(x)		(((x) & 0xFF) << 8)
#define ADI_TO_USR_DATATYPE_TOTAL_BITS(x)	(((x) >> 8) & 0xFF)
#define ADI_USR_DATATYPE_BITS(x)			(((x) & 0xFF) << 0)
#define ADI_TO_USR_DATATYPE_BITS(x)		(((x) >> 0) & 0xFF)

#define ADI_REG_CHAN_USR_CNTRL_2(c)		(0x0424 + (c) * 0x40)
#define ADI_USR_DECIMATION_M(x)			(((x) & 0xFFFF) << 16)
#define ADI_TO_USR_DECIMATION_M(x)		(((x) >> 16) & 0xFFFF)
#define ADI_USR_DECIMATION_N(x)			(((x) & 0xFFFF) << 0)
#define ADI_TO_USR_DECIMATION_N(x)		(((x) >> 0) & 0xFFFF)

#define ADI_MAX_CHANNEL			128

struct adc_chip_info {
	int (*special_probe)(struct platform_device *pdev);
	bool has_no_sample_clk;
	bool has_frontend;
	const struct iio_chan_spec *channels;
	unsigned int num_channels;
	unsigned int ctrl_flags;
};

struct axiadc_state {
	void __iomem			*regs;
	void __iomem			*slave_regs;
	struct iio_hw_consumer		*frontend;
	struct clk 			*clk;
	/* protect against device accesses */
	struct mutex			lock;
	unsigned int                    oversampling_ratio;
	unsigned int			adc_def_output_mode;
	unsigned int			max_usr_channel;
	struct iio_chan_spec		channels[ADI_MAX_CHANNEL];
	unsigned int			adc_calibbias[2];
	unsigned int			adc_calibscale[2][2];
	bool				calibrate;
};

#define CN0363_CHANNEL(_address, _type, _ch, _mod, _rb) { \
	.type = _type, \
	.indexed = 1, \
	.channel = _ch, \
	.modified = (_mod == 0) ? 0 : 1, \
	.channel2 = _mod, \
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ), \
	.address = _address, \
	.scan_index = _address, \
	.scan_type = { \
		.sign = (_type == IIO_ANGL) ? 'u' : 's', \
		.realbits = _rb, \
		.storagebits = 32, \
		.shift = 0, \
		.endianness = IIO_LE, \
	}, \
}

static const struct iio_chan_spec cn0363_channels[] = {
	CN0363_CHANNEL(0, IIO_ANGL, 0, 0, 32),
	CN0363_CHANNEL(1, IIO_VOLTAGE, 0, 0, 24),
	CN0363_CHANNEL(2, IIO_VOLTAGE, 1, 0, 32),
	CN0363_CHANNEL(3, IIO_VOLTAGE, 2, IIO_MOD_I, 32),
	CN0363_CHANNEL(4, IIO_VOLTAGE, 2, IIO_MOD_Q, 32),
	CN0363_CHANNEL(5, IIO_VOLTAGE, 3, IIO_MOD_I, 32),
	CN0363_CHANNEL(6, IIO_VOLTAGE, 3, IIO_MOD_Q, 32),
	CN0363_CHANNEL(7, IIO_ANGL, 1, 0, 32),
	CN0363_CHANNEL(8, IIO_VOLTAGE, 4, 0, 24),
	CN0363_CHANNEL(9, IIO_VOLTAGE, 5, 0, 32),
	CN0363_CHANNEL(10, IIO_VOLTAGE, 6, IIO_MOD_I, 32),
	CN0363_CHANNEL(11, IIO_VOLTAGE, 6, IIO_MOD_Q, 32),
	CN0363_CHANNEL(12, IIO_VOLTAGE, 7, IIO_MOD_I, 32),
	CN0363_CHANNEL(13, IIO_VOLTAGE, 7, IIO_MOD_Q, 32),
};

static const struct adc_chip_info cn0363_chip_info = {
	.special_probe = NULL,
	.has_no_sample_clk = false,
	.has_frontend = true,
	.channels = cn0363_channels,
	.num_channels = ARRAY_SIZE(cn0363_channels),
};

static int axiadc_m2k_special_probe(struct platform_device *pdev);

static const char * const m2k_samp_freq_available[] = {
	"1000",
	"10000",
	"100000",
	"1000000",
	"10000000",
	"100000000"
};

static const struct iio_enum m2k_samp_freq_available_enum = {
	.items = m2k_samp_freq_available,
	.num_items = ARRAY_SIZE(m2k_samp_freq_available),
};

static int m2k_get_calibrate(struct iio_dev *indio_dev,
			     const struct iio_chan_spec *chan)
{
	struct axiadc_state *st = iio_priv(indio_dev);

	return st->calibrate;
}

static int m2k_set_calibrate(struct iio_dev *indio_dev,
			     const struct iio_chan_spec *chan, unsigned int val)
{
	struct axiadc_state *st = iio_priv(indio_dev);

	st->calibrate = val;
	return 0;
}

static const char * const m2k_calibrate_items[] = {
	"false",
	"true"
};

static const struct iio_enum m2k_calibrate_enum = {
	.items = m2k_calibrate_items,
	.num_items = ARRAY_SIZE(m2k_calibrate_items),
	.set = m2k_set_calibrate,
	.get = m2k_get_calibrate,
};

static const struct iio_chan_spec_ext_info m2k_chan_ext_info[] = {
	IIO_ENUM_AVAILABLE("sampling_frequency", IIO_SHARED_BY_ALL,
			&m2k_samp_freq_available_enum),
	IIO_ENUM_AVAILABLE("calibrate", IIO_SHARED_BY_ALL, &m2k_calibrate_enum),
	IIO_ENUM("calibrate", IIO_SHARED_BY_ALL, &m2k_calibrate_enum),
	{ },
};

#define M2K_ADC_DO_SAMP_FREQ_READ(reg)	\
({					\
	uint32_t __reg = reg;		\
	int __val;			\
					\
	switch (__reg) {		\
	case 1:				\
		__val = 10000000;	\
		break;			\
	case 2:				\
		__val = 1000000;	\
		break;			\
	case 3:				\
		__val = 100000;		\
		break;			\
	case 6:				\
		__val = 10000;		\
		break;			\
	case 7:				\
		__val = 1000;		\
		break;			\
	default:			\
		__val = 100000000;	\
		break;			\
	}				\
	__val;				\
})

#define M2K_ADC_DO_SAMP_FREQ_WRITE(val)	\
({					\
	int __val = val;		\
	uint32_t __reg;			\
					\
	if (__val >= 100000000)		\
		__reg = 0;		\
	else if (__val >= 10000000)	\
		__reg = 1;		\
	else if (__val >= 1000000)	\
		__reg = 2;		\
	else if (__val >= 100000)	\
		__reg = 3;		\
	else if (__val >= 10000)	\
		__reg = 6;		\
	else				\
		__reg = 7;		\
	__reg;				\
})

#define M2K_ADC_CHANNEL(_ch) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.channel = _ch, \
	.address = _ch, \
	.scan_index = _ch, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_CALIBSCALE) | \
			BIT(IIO_CHAN_INFO_CALIBBIAS), \
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ) | \
			BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO), \
	.ext_info = m2k_chan_ext_info, \
	.scan_type = { \
		.sign = 's', \
		.realbits = 12, \
		.storagebits = 16, \
		.shift = 0, \
		.endianness = IIO_LE, \
	}, \
}

#define ADRV9002_ADC_CHANNEL(_chan, _mod, _si)	{			\
	.type = IIO_VOLTAGE,						\
	.indexed = 1,							\
	.modified = 1,							\
	.channel = _chan,						\
	.channel2 = _mod,						\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = _si,						\
	.scan_type = {							\
		.sign = 'S',						\
		.realbits = 16,						\
		.storagebits = 16,					\
		.shift = 0,						\
	},								\
}

static const struct adc_chip_info obs_rx_chip_info = {
	.special_probe = NULL,
	.has_no_sample_clk = false,
	.channels = NULL,
	.ctrl_flags = ADI_FORMAT_SIGNEXT | ADI_FORMAT_ENABLE,
};

static const struct iio_chan_spec adrv9002_rx_channels[] = {
	ADRV9002_ADC_CHANNEL(0, IIO_MOD_I, 0),
	ADRV9002_ADC_CHANNEL(0, IIO_MOD_Q, 1),
};

static const struct adc_chip_info adrv9002_rx_chip_info = {
	.special_probe = NULL,
	.has_no_sample_clk = false,
	.channels = adrv9002_rx_channels,
	.num_channels = ARRAY_SIZE(adrv9002_rx_channels),
	.ctrl_flags = ADI_FORMAT_SIGNEXT | ADI_FORMAT_ENABLE,
};

static const struct iio_chan_spec m2k_adc_channels[] = {
	M2K_ADC_CHANNEL(0),
	M2K_ADC_CHANNEL(1),
};

static const struct adc_chip_info m2k_adc_chip_info = {
	.special_probe = axiadc_m2k_special_probe,
	.has_no_sample_clk = true,
	.channels = m2k_adc_channels,
	.num_channels = ARRAY_SIZE(m2k_adc_channels),
	.ctrl_flags = ADI_FORMAT_SIGNEXT | ADI_FORMAT_ENABLE,
};

static inline void axiadc_write(struct axiadc_state *st, unsigned reg, unsigned val)
{
	iowrite32(val, st->regs + reg);
}

static inline unsigned int axiadc_read(struct axiadc_state *st, unsigned reg)
{
	return ioread32(st->regs + reg);
}

static inline void axiadc_slave_write(struct axiadc_state *st, unsigned reg, unsigned val)
{
	iowrite32(val, st->slave_regs + reg);
}

static inline unsigned int axiadc_slave_read(struct axiadc_state *st, unsigned reg)
{
	return ioread32(st->slave_regs + reg);
}

static int axiadc_hw_consumer_postenable(struct iio_dev *indio_dev)
{
	struct axiadc_state *st = iio_priv(indio_dev);

	return iio_hw_consumer_enable(st->frontend);
}

static int axiadc_hw_submit_block(struct iio_dma_buffer_queue *queue,
	struct iio_dma_buffer_block *block)
{
	struct iio_dev *indio_dev = queue->driver_data;
	struct axiadc_state *st = iio_priv(indio_dev);

	iio_dmaengine_buffer_submit_block(queue, block);

	axiadc_write(st, ADI_REG_STATUS, ~0);
	axiadc_write(st, ADI_REG_DMA_STATUS, ~0);

	return 0;
}

static const struct iio_dma_buffer_ops axiadc_dma_buffer_ops = {
	.submit = axiadc_hw_submit_block,
	.abort = iio_dmaengine_buffer_abort,
};

static int axiadc_configure_ring_stream(struct iio_dev *indio_dev,
	const char *dma_name)
{
	struct iio_buffer *buffer;

	if (dma_name == NULL)
		dma_name = "rx";

	buffer = devm_iio_dmaengine_buffer_alloc(indio_dev->dev.parent, dma_name,
						 &axiadc_dma_buffer_ops, indio_dev);
	if (IS_ERR(buffer))
		return PTR_ERR(buffer);

	indio_dev->modes |= INDIO_BUFFER_HARDWARE;
	iio_device_attach_buffer(indio_dev, buffer);

	return 0;
}

static int axiadc_hw_consumer_predisable(struct iio_dev *indio_dev)
{
	struct axiadc_state *st = iio_priv(indio_dev);

	iio_hw_consumer_disable(st->frontend);
	return 0;
}

static const struct iio_buffer_setup_ops axiadc_hw_consumer_setup_ops = {
	.postenable = &axiadc_hw_consumer_postenable,
	.predisable = &axiadc_hw_consumer_predisable,
};

static int axiadc_update_scan_mode(struct iio_dev *indio_dev,
				   const unsigned long *scan_mask)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	unsigned int i, ctrl;

	for (i = 0; i < indio_dev->masklength; i++) {
		ctrl = axiadc_read(st, ADI_REG_CHAN_CNTRL(i));

		if (test_bit(i, scan_mask))
			ctrl |= ADI_ENABLE;
		else
			ctrl &= ~ADI_ENABLE;

		ctrl |= st->adc_def_output_mode;

		axiadc_write(st, ADI_REG_CHAN_CNTRL(i), ctrl);
	}

	return 0;
}

static unsigned int cf_axi_dds_to_signed_mag_fmt(int val, int val2)
{
	unsigned int i;
	u64 val64;

	if (val > 1) {
		val = 1;
		val2 = 999999;
	} else if (val < -1) {
		val = -1;
		val2 = 999999;
	}

	/*  format is 1.1.14 (sign, integer and fractional bits) */
	switch (val) {
	case 1:
		i = 0x4000;
		break;
	case -1:
		i = 0xC000;
		break;
	default: /* val = 0 */
		i = 0;
		if (val2 < 0) {
			i = 0x8000;
			val2 *= -1;
		}
		break;
	}

	val64 = (unsigned long long)val2 * 0x4000UL + (1000000UL / 2);
		do_div(val64, 1000000UL);

	if (i & 0x4000 && val64 == 0x4000)
		val64 = 0x3fff;

	return i | val64;
}

static int cf_axi_dds_signed_mag_fmt_to_iio(unsigned int val, int *r_val,
					    int *r_val2)
{
	u64 val64;
	int sign;

	if (val & 0x8000)
		sign = -1;
	else
		sign = 1;

	if (val & 0x4000)
		*r_val = 1 * sign;
	else
		*r_val = 0;

	val &= ~0xC000;

	val64 = val * 1000000ULL + (0x4000 / 2);
	do_div(val64, 0x4000);

	if (*r_val == 0)
		*r_val2 = val64 * sign;
	else
		*r_val2 = val64;

	return IIO_VAL_INT_PLUS_MICRO;
}

static int axiadc_read_raw(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, int *val, int *val2, long info)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	uint32_t reg;

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (st->slave_regs) {
			reg = axiadc_slave_read(st, 0x44);
			*val = M2K_ADC_DO_SAMP_FREQ_READ(reg);
		} else if (!IS_ERR(st->clk)) {
			*val = clk_get_rate(st->clk);
		} else {
			return -ENODEV;
		}
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		*val = st->oversampling_ratio;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_CALIBSCALE:
		if (st->calibrate) {
			*val = st->adc_calibscale[chan->channel][0];
			*val2 = st->adc_calibscale[chan->channel][1];
			return IIO_VAL_INT_PLUS_MICRO;
		}

		if (!st->slave_regs)
			return -EINVAL;
		reg = axiadc_slave_read(st,
			       ADI_REG_CORRECTION_COEFFICIENT(chan->channel));
		return cf_axi_dds_signed_mag_fmt_to_iio(reg, val, val2);
	case IIO_CHAN_INFO_CALIBBIAS:
		*val = st->adc_calibbias[chan->channel];
		return IIO_VAL_INT;
	default:
		break;
	}

	return -EINVAL;
}

static int axiadc_m2k_special_probe(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct axiadc_state *st = iio_priv(indio_dev);
	struct resource *mem;
	int i;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (mem) {
		unsigned int val;

		st->slave_regs = devm_ioremap_resource(&pdev->dev, mem);
		if (IS_ERR(st->slave_regs))
			return PTR_ERR(st->slave_regs);

		val = cf_axi_dds_to_signed_mag_fmt(1, 0);
		axiadc_slave_write(st, ADI_REG_CORRECTION_COEFFICIENT(0), val);
		axiadc_slave_write(st, ADI_REG_CORRECTION_COEFFICIENT(1), val);

		st->adc_calibscale[0][0] = 1;
		st->adc_calibscale[1][0] = 1;
		st->adc_calibscale[0][1] = 0;
		st->adc_calibscale[1][1] = 0;
	}

	for (i = 0; i < indio_dev->num_channels; i++) {
		if (indio_dev->channels[i].info_mask_separate &
			BIT(IIO_CHAN_INFO_CALIBSCALE)) {
			unsigned int chan = indio_dev->channels[i].channel;

			axiadc_write(st, ADI_REG_CHAN_CNTRL_2(chan),
				     0x40000000);
			axiadc_write(st, ADI_REG_CHAN_CNTRL(chan),
				     ADI_IQCOR_ENB);
		}
		if (indio_dev->channels[i].info_mask_separate &
			BIT(IIO_CHAN_INFO_CALIBBIAS)) {
			unsigned int chan = indio_dev->channels[i].channel;
			//set initial claibration to neutral value
			st->adc_calibbias[chan] = 2048;
		}
	}

	return 0;
}

static int axiadc_write_raw(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, int val, int val2, long info)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	uint32_t reg;

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (!st->slave_regs)
			return -EINVAL;
		reg = M2K_ADC_DO_SAMP_FREQ_WRITE(val);
		axiadc_slave_write(st, 0x44, reg);
		return 0;
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		if (val == 0)
			return -EINVAL;
		st->oversampling_ratio = val;
		axiadc_slave_write(st, 0x40, val - 1);
		return 0;
	case IIO_CHAN_INFO_CALIBSCALE:
		if (st->calibrate) {
			st->adc_calibscale[chan->channel][0] = val;
			st->adc_calibscale[chan->channel][1] = val2;
			return 0;
		}

		if (!st->slave_regs)
			return -EINVAL;

		reg = axiadc_slave_read(st, ADI_REG_CORRECTION_ENABLE);
		if (val == 1 && val2 == 0)
			reg &= ~BIT(chan->channel);
		else
			reg |= BIT(chan->channel);
		axiadc_slave_write(st, ADI_REG_CORRECTION_ENABLE, reg);

		reg = cf_axi_dds_to_signed_mag_fmt(val, val2);
		axiadc_slave_write(st,
				ADI_REG_CORRECTION_COEFFICIENT(chan->channel),
				reg);
		return 0;
	case IIO_CHAN_INFO_CALIBBIAS:
		if (val < 0 || val > 0xFFFF)
			return -EINVAL;

		st->adc_calibbias[chan->channel] = val;
		return 0;
	default:
		break;
	}

	return -EINVAL;
}

static int adc_reg_access(struct iio_dev *indio_dev,
			  unsigned int reg, unsigned int writeval,
			  unsigned int *readval)
{
	struct axiadc_state *st = iio_priv(indio_dev);

	mutex_lock(&st->lock);
	if (st->slave_regs && (reg & 0x80000000)) {
		if (readval == NULL)
			axiadc_slave_write(st, (reg & 0xffff), writeval);
		else
			*readval = axiadc_slave_read(st, (reg & 0xffff));
	} else {
		if (readval == NULL)
			axiadc_write(st, reg & 0xFFFF, writeval);
		else
			*readval = axiadc_read(st, reg & 0xFFFF);
	}
	mutex_unlock(&st->lock);

	return 0;
}

static const struct iio_info adc_info = {
	.read_raw = axiadc_read_raw,
	.write_raw = axiadc_write_raw,
	.debugfs_reg_access = &adc_reg_access,
	.update_scan_mode = axiadc_update_scan_mode,
};

static const struct of_device_id adc_of_match[] = {
	{ .compatible = "adi,cn0363-adc-1.00.a", .data = &cn0363_chip_info },
	{ .compatible = "adi,axi-ad9371-obs-1.0",
				.data = &obs_rx_chip_info },
	{ .compatible = "adi,m2k-adc-1.00.a", .data = &m2k_adc_chip_info },
	{ .compatible = "adi,axi-adrv9009-obs-1.0",
				.data = &obs_rx_chip_info },
	{ .compatible = "adi,axi-adrv9009-obs-single-1.0",
				.data = &obs_rx_chip_info },
	{ .compatible = "adi,axi-adrv9002-rx2-1.0",
				.data = &adrv9002_rx_chip_info },
	{ .compatible = "adi,axi-adc-tpl-so-10.0.a",
		.data = &obs_rx_chip_info },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, adc_of_match);

static void adc_fill_channel_data(struct iio_dev *indio_dev)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	unsigned int usr_ctrl, jesd_np;
	int i;

	st->max_usr_channel = ADI_USR_CHANMAX(axiadc_read(st, ADI_REG_USR_CNTRL_1));

	/*
	 * In case JESD TPL (up_tpl_common) is available use
	 * JESD_NP to set realbits
	 */
	jesd_np = axiadc_read(st, ADI_JESD204_REG_TPL_DESCRIPTOR_2);
	if (jesd_np != 0xDEADDEAD)
		jesd_np = ADI_JESD204_TPL_TO_NP(jesd_np);

	if (st->max_usr_channel == 0)
		dev_warn(indio_dev->dev.parent,
			 "Zero read from REG_USR_CNTRL_1\n");

	for (i = 0; (i < st->max_usr_channel) && (i < ADI_MAX_CHANNEL); i++) {
		usr_ctrl = axiadc_read(st, ADI_REG_CHAN_USR_CNTRL_1(i));
		st->channels[i].type = IIO_VOLTAGE;
		st->channels[i].indexed = 1;
		st->channels[i].channel = i / 2;
		st->channels[i].address = i / 2;
		st->channels[i].channel2 = (i & 1) ? IIO_MOD_Q : IIO_MOD_I;
		st->channels[i].modified = 1;
		st->channels[i].scan_index = i;
		st->channels[i].info_mask_shared_by_all =
			BIT(IIO_CHAN_INFO_SAMP_FREQ);
		st->channels[i].scan_type.sign =
			(usr_ctrl & ADI_USR_DATATYPE_SIGNED) ? 's' : 'u';
		st->channels[i].scan_type.realbits =
			jesd_np ? jesd_np : ADI_TO_USR_DATATYPE_BITS(usr_ctrl);
		st->channels[i].scan_type.storagebits =
			ADI_TO_USR_DATATYPE_TOTAL_BITS(usr_ctrl);
		st->channels[i].scan_type.shift =
			ADI_TO_USR_DATATYPE_SHIFT(usr_ctrl);
		st->channels[i].scan_type.endianness =
			(usr_ctrl & ADI_USR_DATATYPE_BE) ? IIO_BE : IIO_LE;
	}

	indio_dev->channels = st->channels;
	indio_dev->num_channels = st->max_usr_channel;
}

static void adc_clk_disable(void *clk)
{
	clk_disable_unprepare(clk);
}

static int adc_probe(struct platform_device *pdev)
{
	const struct adc_chip_info *info;
	const struct of_device_id *id;
	struct iio_dev *indio_dev;
	struct axiadc_state *st;
	struct resource *mem;
	int ret;

	id = of_match_node(adc_of_match, pdev->dev.of_node);
	if (!id)
		return -ENODEV;

	info = id->data;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	mutex_init(&st->lock);

	st->clk = devm_clk_get(&pdev->dev, "sampl_clk");
	if (IS_ERR(st->clk)) {
		if (!info->has_no_sample_clk)
			return PTR_ERR(st->clk);
	} else {
		ret = clk_prepare_enable(st->clk);
		if (ret)
			return ret;

		ret = devm_add_action_or_reset(&pdev->dev, adc_clk_disable, st->clk);
		if (ret)
			return ret;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	st->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(st->regs))
		return PTR_ERR(st->regs);

	platform_set_drvdata(pdev, indio_dev);

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = pdev->dev.of_node->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &adc_info;

	/* Reset all HDL Cores */
	axiadc_write(st, ADI_REG_RSTN, 0);
	axiadc_write(st, ADI_REG_RSTN, ADI_RSTN);

	if (info->has_frontend) {
		st->frontend = devm_iio_hw_consumer_alloc(&pdev->dev);
		if (IS_ERR(st->frontend))
			return PTR_ERR(st->frontend);
		indio_dev->setup_ops = &axiadc_hw_consumer_setup_ops;
	}

	st->adc_def_output_mode = info->ctrl_flags;

	if (!info->channels) {
		adc_fill_channel_data(indio_dev);
	} else {
		indio_dev->channels = info->channels;
		indio_dev->num_channels = info->num_channels;
	}

	ret = axiadc_configure_ring_stream(indio_dev, "rx");
	if (ret)
		return ret;

	/* handle special probe */
	if (info->special_probe) {
		ret = info->special_probe(pdev);
		if (ret)
			return ret;
	}

	return devm_iio_device_register(&pdev->dev, indio_dev);
}

static struct platform_driver adc_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = adc_of_match,
	},
	.probe	  = adc_probe,
};

module_platform_driver(adc_driver);

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADC");
MODULE_LICENSE("GPL v2");
