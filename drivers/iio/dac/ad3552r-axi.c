// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices AD3552R
 * Digital to Analog converter driver, axi version
 *
 * Copyright 2024 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/buffer.h>
#include <linux/iio/backend.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "ad3552r.h"

#define AD3552R_MAX_SAMPLERATE	33000000

#define AD3552R_NAME		"ad3552r"

struct ad3552r_axi_state {
	enum ad3552r_id chip_id;
	struct gpio_desc *reset_gpio;
	struct clk *ref_clk;
	struct device *dev;
	bool ddr;
	bool single_channel;
	bool synced_transfer;
	struct iio_backend *back;
};

enum ad3552r_ext_info {
	AD3552R_AXI_SYNC,
};

static int ad3552r_axi_qspi_read(struct iio_backend *back, u32 reg, u32 *val)
{
	return iio_backend_bus_reg_read(back, &reg, sizeof(u32),
					val, sizeof(u32));
}

static int ad3552r_axi_qspi_write(struct iio_backend *back, u32 reg, u32 val)
{
	return iio_backend_bus_reg_write(back, &reg, sizeof(u32),
					 &val, sizeof(u32));
}

static int axi3552r_qspi_update_reg_bits(struct iio_backend *back,
					 u32 reg, u32 mask, u32 val)
{
	u32 rval;
	int err;

	err = ad3552r_axi_qspi_read(back, reg, &rval);
	if (err)
		return err;

	rval &= ~mask;
	rval |= val;

	return ad3552r_axi_qspi_write(back, reg, val);
}

static int ad3552r_axi_read_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int *val, int *val2, long mask)
{
	struct ad3552r_axi_state *st = iio_priv(indio_dev);
	int err, ch = chan->channel;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		/*
		 * As of now, keeping the default sample rate that is driven
		 * from the interface clock (actually 66000000, DDR).
		 * So we have 66000000 * 2 / 4 (for 16 bit samples).
		 */
		*val = AD3552R_MAX_SAMPLERATE;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_RAW:
		err = iio_backend_data_size_set(st->back, 16);
		if (err)
			return err;

		err = ad3552r_axi_qspi_read(st->back,
					    AD3552R_REG_ADDR_CH_DAC_16B(ch),
					    val);
		if (err)
			return err;

		/* Back to 8 bit, that's always used for all other commands */
		err = iio_backend_data_size_set(st->back, 8);
		if (err)
			return err;

		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int ad3552r_axi_write_raw(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan,
				 int val, int val2, long mask)
{
	struct ad3552r_axi_state *st = iio_priv(indio_dev);
	int err, ch = chan->channel;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		err = iio_backend_data_size_set(st->back, 16);
		if (err)
			return err;

		err = ad3552r_axi_qspi_write(st->back,
					     AD3552R_REG_ADDR_CH_DAC_16B(ch),
					     (u32)val);

		/* Back to 8 bit, that's always used for all other commands */
		return iio_backend_data_size_set(st->back, 8);
	default:
		return -EINVAL;
	}
}

static int ad3552r_axi_reg_access(struct iio_dev *indio_dev,
				  unsigned int reg, unsigned int writeval,
				  unsigned int *readval)
{
	/* TODO */
	return 0;
}

static int ad3552r_axi_update_scan_mode(struct iio_dev *indio_dev,
					const unsigned long *active_scan_mask)
{
	struct ad3552r_axi_state *st = iio_priv(indio_dev);
	u32 loop_len, val;
	int err;

	st->ddr = true;
	err = axi3552r_qspi_update_reg_bits(st->back,
					    AD3552R_REG_ADDR_INTERFACE_CONFIG_D,
					    AD3552R_MASK_SPI_CONFIG_DDR,
					    AD3552R_MASK_SPI_CONFIG_DDR);
	if (err)
		return err;

	switch (*active_scan_mask) {
	case AD3552R_CH0_ACTIVE:
		st->single_channel = true;
		loop_len = AD3552R_STREAM_2BYTE_LOOP;
		val = AD3552R_REG_ADDR_CH_DAC_16B(0);
		break;
	case AD3552R_CH1_ACTIVE:
		st->single_channel = true;
		loop_len = AD3552R_STREAM_2BYTE_LOOP;
		val = AD3552R_REG_ADDR_CH_DAC_16B(1);
		break;
	case AD3552R_CH0_CH1_ACTIVE:
		st->single_channel = false;
		loop_len = AD3552R_STREAM_4BYTE_LOOP;
		val = AD3552R_REG_ADDR_CH_DAC_16B(1);
		break;
	default:
		return -EINVAL;
	}

	err = iio_backend_ddr_enable(st->back, 1);
	if (err)
		return err;

	err = ad3552r_axi_qspi_write(st->back, AD3552R_REG_ADDR_STREAM_MODE,
				     loop_len);
	if (err)
		return err;

	err = iio_backend_ddr_enable(st->back, 0);
	if (err)
		return err;

	return iio_backend_data_transfer_addr(st->back, val);
}

static int ad3552r_axi_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ad3552r_axi_state *st = iio_priv(indio_dev);
	int err;

	if (st->synced_transfer) {
		err = iio_backend_ext_sync(st->back);
		if (err)
			return err;
	}

	err = iio_backend_ddr_enable(st->back, true);
	if (err)
		return err;

	return iio_backend_buffer_enable(st->back, 1);
}

static int ad3552r_axi_buffer_predisable(struct iio_dev *indio_dev)
{
	struct ad3552r_axi_state *st = iio_priv(indio_dev);

	return iio_backend_buffer_enable(st->back, 0);
}

static int ad3552r_axi_set_output_range(struct ad3552r_axi_state *st,
					unsigned int mode)
{
	int range_ch_0 = FIELD_PREP(AD3552R_MASK_CH0_RANGE, mode);
	int range_ch_1 = FIELD_PREP(AD3552R_MASK_CH1_RANGE, mode);

	return axi3552r_qspi_update_reg_bits(st->back,
				AD3552R_REG_ADDR_CH0_CH1_OUTPUT_RANGE,
				AD3552R_MASK_CH_OUTPUT_RANGE,
				range_ch_0 | range_ch_1);
}

static int ad3552r_axi_reset(struct ad3552r_axi_state *st)
{
	int err;

	/* AXI reset performed by backend enable() */

	st->reset_gpio = devm_gpiod_get_optional(st->dev,
						 "reset", GPIOD_OUT_LOW);
	if (IS_ERR(st->reset_gpio))
		return PTR_ERR(st->reset_gpio);

	if (st->reset_gpio) {
		gpiod_set_value_cansleep(st->reset_gpio, 1);
		fsleep(10);
		gpiod_set_value_cansleep(st->reset_gpio, 0);
	} else {
		err = axi3552r_qspi_update_reg_bits(st->back,
					AD3552R_REG_ADDR_INTERFACE_CONFIG_A,
					AD3552R_MASK_SOFTWARE_RESET,
					AD3552R_MASK_SOFTWARE_RESET);
		if (err)
			return err;
	}
	fsleep(100000);

	return 0;
}

static int ad3552r_axi_setup(struct ad3552r_axi_state *st)
{
	struct fwnode_handle *child;
	u32 val;
	u16 id;
	u32 range;
	int err;

	err = ad3552r_axi_reset(st);
	if (err)
		return err;

	/* Following commands are all sent 8bit, SDR */
	err = iio_backend_data_size_set(st->back, 8);
	if (err)
		return err;

	err = iio_backend_ddr_enable(st->back, 0);
	if (err)
		return err;

	err = ad3552r_axi_qspi_write(st->back, AD3552R_REG_ADDR_SCRATCH_PAD,
				     AD3552R_SCRATCH_PAD_TEST_VAL1);
	if (err)
		return err;

	err = ad3552r_axi_qspi_read(st->back, AD3552R_REG_ADDR_SCRATCH_PAD,
				    &val);
	if (err)
		return err;

	if (val != AD3552R_SCRATCH_PAD_TEST_VAL1) {
		dev_err(st->dev,
			"SCRATCH_PAD_TEST mismatch. Expected 0x%x, Read 0x%x\n",
			AD3552R_SCRATCH_PAD_TEST_VAL1, val);
		return -EIO;
	}

	err = ad3552r_axi_qspi_write(st->back,
				     AD3552R_REG_ADDR_SCRATCH_PAD,
				     AD3552R_SCRATCH_PAD_TEST_VAL2);
	if (err)
		return err;

	err = ad3552r_axi_qspi_read(st->back, AD3552R_REG_ADDR_SCRATCH_PAD,
				    &val);
	if (err)
		return err;

	if (val != AD3552R_SCRATCH_PAD_TEST_VAL2) {
		dev_err(st->dev,
			"SCRATCH_PAD_TEST mismatch. Expected 0x%x, Read 0x%x\n",
			AD3552R_SCRATCH_PAD_TEST_VAL2, val);
		return -EIO;
	}

	err = ad3552r_axi_qspi_read(st->back, AD3552R_REG_ADDR_PRODUCT_ID_L,
				    &val);
	if (err)
		return err;

	id = val;
	mdelay(100);

	err = ad3552r_axi_qspi_read(st->back, AD3552R_REG_ADDR_PRODUCT_ID_H,
				    &val);
	if (err)
		return err;

	id |= val << 8;
	if (id != AD3552R_ID) {
		dev_err(st->dev, "Chip ID mismatch. Expected 0x%x, Read 0x%x\n",
			AD3552R_ID, id);
		return -ENODEV;
	}

	st->chip_id = id;

	err = ad3552r_axi_qspi_write(st->back,
				     AD3552R_REG_ADDR_SH_REFERENCE_CONFIG,
				     AD3552R_REF_INIT);
	if (err)
		return err;

	err = ad3552r_axi_qspi_write(st->back,
				     AD3552R_REG_ADDR_TRANSFER_REGISTER,
				     AD3552R_TRANSFER_INIT);
	if (err)
		return err;

	err = iio_backend_data_source_set(st->back, 0, IIO_BACKEND_EXTERNAL);
	if (err)
		return err;

	err = iio_backend_data_source_set(st->back, 1, IIO_BACKEND_EXTERNAL);
	if (err)
		return err;

	child = device_get_named_child_node(st->dev, "channel");
	if (!child)
		return -EINVAL;

	err = ad3552r_get_output_range(st->dev, st->chip_id, child, &range);
	if (err)
		return err;

	return ad3552r_axi_set_output_range(st, range);
}

static const struct iio_buffer_setup_ops ad3552r_axi_buffer_setup_ops = {
	.postenable = ad3552r_axi_buffer_postenable,
	.predisable = ad3552r_axi_buffer_predisable,
};

static ssize_t ad3552r_axi_read_ext(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan, char *buf)
{
	struct ad3552r_axi_state *st = iio_priv(indio_dev);

	switch (private) {
	case AD3552R_AXI_SYNC:
		return sysfs_emit(buf, "%d\n", st->synced_transfer);
	default:
		return -EINVAL;
	}
}

static ssize_t ad3552r_axi_write_ext(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     const char *buf, size_t len)
{
	struct ad3552r_axi_state *st = iio_priv(indio_dev);
	bool enabled;
	int ret;

	ret = kstrtobool(buf, &enabled);
	if (ret)
		return ret;

	switch (private) {
	case AD3552R_AXI_SYNC:
		st->synced_transfer = enabled;
		return len;
	default:
		return -EINVAL;
	}
}

#define AD3552R_CHAN_EXT_INFO(_name, _what, _shared) { \
	.name = _name, \
	.read = ad3552r_axi_read_ext, \
	.write = ad3552r_axi_write_ext, \
	.private = _what, \
	.shared = _shared, \
}

static const struct iio_chan_spec_ext_info ad3552r_axi_ext_info[] = {
	AD3552R_CHAN_EXT_INFO("synchronous_mode", AD3552R_AXI_SYNC,
			      IIO_SHARED_BY_TYPE),
	{},
};

#define AD3552R_CHANNEL(ch) { \
	.type = IIO_VOLTAGE, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
	.info_mask_shared_by_all = (((ch) == 0) ? \
		BIT(IIO_CHAN_INFO_SAMP_FREQ) : 0), \
	.output = 1, \
	.indexed = 1, \
	.channel = (ch), \
	.scan_index = (ch), \
	.scan_type = { \
		.sign = 'u', \
		.realbits = 16, \
		.storagebits = 16, \
		.shift = 0, \
		.endianness = IIO_BE, \
	}, \
	.ext_info = ad3552r_axi_ext_info, \
}

static struct iio_chan_spec ad3552r_axi_channels[] = {
	AD3552R_CHANNEL(0),
	AD3552R_CHANNEL(1),
};

static const struct iio_info ad3552r_axi_info = {
	.read_raw = &ad3552r_axi_read_raw,
	.write_raw = &ad3552r_axi_write_raw,
	.debugfs_reg_access = &ad3552r_axi_reg_access,
	.update_scan_mode = ad3552r_axi_update_scan_mode,
};

static int ad3552r_axi_probe(struct platform_device *pdev)
{
	struct ad3552r_axi_state *st;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->dev = &pdev->dev;

	st->back = devm_iio_backend_get(&pdev->dev, NULL);
	if (IS_ERR(st->back))
		return PTR_ERR(st->back);

	ret = devm_iio_backend_request_buffer(&pdev->dev, st->back, indio_dev);
	if (ret)
		return ret;

	ret = devm_iio_backend_enable(&pdev->dev, st->back);
	if (ret)
		return ret;

	indio_dev->name = AD3552R_NAME;
	indio_dev->modes = INDIO_BUFFER_HARDWARE;
	indio_dev->setup_ops = &ad3552r_axi_buffer_setup_ops;
	indio_dev->channels = ad3552r_axi_channels;
	indio_dev->num_channels = ARRAY_SIZE(ad3552r_axi_channels);
	indio_dev->info = &ad3552r_axi_info;

	ret = ad3552r_axi_setup(st);
	if (ret)
		return ret;

	return devm_iio_device_register(&pdev->dev, indio_dev);
}

static const struct of_device_id ad3552r_axi_of_id[] = {
	{ .compatible = "adi,ad3552r" },
	{}
};
MODULE_DEVICE_TABLE(of, ad3552r_axi_of_id);

static struct platform_driver axi_ad3552r_driver = {
	.driver = {
		.name = "ad3552r-axi",
		.of_match_table = ad3552r_axi_of_id,
	},
	.probe = ad3552r_axi_probe,
};
module_platform_driver(axi_ad3552r_driver);

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_AUTHOR("Angelo Dureghello <adueghello@baylibre.com>");
MODULE_DESCRIPTION("AD3552R Driver - AXI IP version");
MODULE_LICENSE("GPL");
