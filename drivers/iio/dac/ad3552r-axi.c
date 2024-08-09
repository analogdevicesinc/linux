// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices AD3552R
 * Digital to Analog converter driver, AXI DAC backend version
 *
 * Copyright 2024 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/buffer.h>
#include <linux/iio/backend.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/units.h>

#include "ad3552r.h"

enum ad3552r_synchronous_mode_status {
	AD3552R_NO_SYNC,
	AD3552R_EXT_SYNC_ARM,
};

struct ad3552r_axi_state {
	struct gpio_desc *reset_gpio;
	struct device *dev;
	struct iio_backend *back;
	unsigned long active_scan_mask;
	enum ad3552r_id chip_id;
	bool single_channel;
	bool synced_transfer;
};

static int axi3552r_qspi_update_reg_bits(struct iio_backend *back,
					 u32 reg, u32 mask, u32 val,
					 size_t xfer_size)
{
	u32 rval;
	int err;

	err = iio_backend_bus_reg_read(back, reg, &rval, xfer_size);
	if (err)
		return err;

	rval &= ~mask;
	rval |= val;

	return iio_backend_bus_reg_write(back, reg, &rval, xfer_size);
}

static int ad3552r_axi_read_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int *val, int *val2, long mask)
{
	struct ad3552r_axi_state *st = iio_priv(indio_dev);
	int err, ch = chan->channel;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ: {
		int clk_rate;

		err = iio_backend_read_raw(st->back, chan, &clk_rate, 0,
					   IIO_CHAN_INFO_FREQUENCY);
		if (err != IIO_VAL_INT)
			return err;

		/*
		 * Data stream SDR/DDR (clk_in/8 or clk_in/4 update rate).
		 * Samplerate has sense in DDR only.
		 */
		if (st->single_channel)
			clk_rate = DIV_ROUND_CLOSEST(clk_rate, 4);
		else
			clk_rate = DIV_ROUND_CLOSEST(clk_rate, 8);

		*val = clk_rate;

		return IIO_VAL_INT;
	}
	case IIO_CHAN_INFO_RAW:
		err = iio_backend_bus_reg_read(st->back,
					       AD3552R_REG_ADDR_CH_DAC_16B(ch),
					       val, 2);
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
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		iio_device_claim_direct_scoped(return -EBUSY, indio_dev) {
			struct ad3552r_axi_state *st = iio_priv(indio_dev);
			int ch = chan->channel;

			return iio_backend_bus_reg_write(st->back,
				    AD3552R_REG_ADDR_CH_DAC_16B(ch), &val, 2);
		}
		unreachable();
	default:
		return -EINVAL;
	}
}

static int ad3552r_axi_update_scan_mode(struct iio_dev *indio_dev,
					const unsigned long *active_scan_mask)
{
	struct ad3552r_axi_state *st = iio_priv(indio_dev);

	st->active_scan_mask = *active_scan_mask;

	return 0;
}

static int ad3552r_axi_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ad3552r_axi_state *st = iio_priv(indio_dev);
	struct iio_backend_data_fmt fmt = {
		.type = IIO_BACKEND_DATA_UNSIGNED
	};
	int loop_len, val, err;

	/* Inform DAC chip to switch into DDR mode */
	err = axi3552r_qspi_update_reg_bits(st->back,
					    AD3552R_REG_ADDR_INTERFACE_CONFIG_D,
					    AD3552R_MASK_SPI_CONFIG_DDR,
					    AD3552R_MASK_SPI_CONFIG_DDR, 1);
	if (err)
		return err;

	/* Inform DAC IP to go for DDR mode from now on */
	err = iio_backend_ddr_enable(st->back);
	if (err)
		goto exit_err;

	switch (st->active_scan_mask) {
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

	err = iio_backend_bus_reg_write(st->back, AD3552R_REG_ADDR_STREAM_MODE,
					&loop_len, 1);
	if (err)
		goto exit_err;

	iio_backend_data_transfer_addr(st->back, val);
	if (err)
		goto exit_err;
	/*
	 * The EXT_SYNC is mandatory in the CN0585 project where 2 instances
	 * of the IP are in the design and they need to generate the signals
	 * synchronized.
	 *
	 * Note: in first IP implementations CONFIG EXT_SYNC (RO) can be 0,
	 * but EXT_SYMC is anabled anyway.
	 */

	if (st->synced_transfer == AD3552R_EXT_SYNC_ARM)
		err = iio_backend_ext_sync_enable(st->back);
	else
		err = iio_backend_ext_sync_disable(st->back);
	if (err)
		goto exit_err_sync;

	err = iio_backend_data_format_set(st->back, 0, &fmt);
	if (err)
		goto exit_err;

	err =  iio_backend_buffer_enable(st->back);
	if (!err)
		return 0;

exit_err_sync:
	iio_backend_ext_sync_disable(st->back);

exit_err:
	axi3552r_qspi_update_reg_bits(st->back,
				      AD3552R_REG_ADDR_INTERFACE_CONFIG_D,
				      AD3552R_MASK_SPI_CONFIG_DDR,
				      0, 1);

	iio_backend_ddr_disable(st->back);

	return err;
}

static int ad3552r_axi_buffer_predisable(struct iio_dev *indio_dev)
{
	struct ad3552r_axi_state *st = iio_priv(indio_dev);
	int err;

	err = iio_backend_buffer_disable(st->back);
	if (err)
		return err;

	/* Inform DAC to set in DDR mode */
	err = axi3552r_qspi_update_reg_bits(st->back,
					    AD3552R_REG_ADDR_INTERFACE_CONFIG_D,
					    AD3552R_MASK_SPI_CONFIG_DDR,
					    0, 1);
	if (err)
		return err;

	return iio_backend_ddr_disable(st->back);
}

static int ad3552r_axi_set_output_range(struct ad3552r_axi_state *st,
					unsigned int mode)
{
	int range_ch_0 = FIELD_PREP(AD3552R_MASK_CH0_RANGE, mode);
	int range_ch_1 = FIELD_PREP(AD3552R_MASK_CH1_RANGE, mode);

	return axi3552r_qspi_update_reg_bits(st->back,
				AD3552R_REG_ADDR_CH0_CH1_OUTPUT_RANGE,
				AD3552R_MASK_CH_OUTPUT_RANGE,
				range_ch_0 | range_ch_1, 1);
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
					AD3552R_MASK_SOFTWARE_RESET, 1);
		if (err)
			return err;
	}
	msleep(100);

	return 0;
}

static int ad3552r_axi_setup(struct ad3552r_axi_state *st)
{
	struct fwnode_handle *child __free(fwnode_handle) = NULL;
	u8 gs_p, gs_n;
	s16 goffs;
	u16 id, rfb, reg = 0, offset = 0;
	u32 val, range;
	int err;

	err = ad3552r_axi_reset(st);
	if (err)
		return err;

	err = iio_backend_ddr_disable(st->back);
	if (err)
		return err;

	val = AD3552R_SCRATCH_PAD_TEST_VAL1;
	err = iio_backend_bus_reg_write(st->back, AD3552R_REG_ADDR_SCRATCH_PAD,
					&val, 1);
	if (err)
		return err;

	err = iio_backend_bus_reg_read(st->back, AD3552R_REG_ADDR_SCRATCH_PAD,
				       &val, 1);
	if (err)
		return err;

	if (val != AD3552R_SCRATCH_PAD_TEST_VAL1) {
		dev_err(st->dev,
			"SCRATCH_PAD_TEST mismatch. Expected 0x%x, Read 0x%x\n",
			AD3552R_SCRATCH_PAD_TEST_VAL1, val);
		return -EIO;
	}

	val = AD3552R_SCRATCH_PAD_TEST_VAL2;
	err = iio_backend_bus_reg_write(st->back,
					AD3552R_REG_ADDR_SCRATCH_PAD,
					&val, 1);
	if (err)
		return err;

	err = iio_backend_bus_reg_read(st->back, AD3552R_REG_ADDR_SCRATCH_PAD,
				       &val, 1);
	if (err)
		return err;

	if (val != AD3552R_SCRATCH_PAD_TEST_VAL2) {
		dev_err(st->dev,
			"SCRATCH_PAD_TEST mismatch. Expected 0x%x, Read 0x%x\n",
			AD3552R_SCRATCH_PAD_TEST_VAL2, val);
		return -EIO;
	}

	err = iio_backend_bus_reg_read(st->back, AD3552R_REG_ADDR_PRODUCT_ID_L,
				       &val, 1);
	if (err)
		return err;

	id = val;
	mdelay(100);

	err = iio_backend_bus_reg_read(st->back, AD3552R_REG_ADDR_PRODUCT_ID_H,
				       &val, 1);
	if (err)
		return err;

	id |= val << 8;
	if (id != AD3552R_ID) {
		dev_err(st->dev, "Chip ID mismatch. Expected 0x%x, Read 0x%x\n",
			AD3552R_ID, id);
		return -ENODEV;
	}

	st->chip_id = id;

	val = AD3552R_REF_INIT;
	err = iio_backend_bus_reg_write(st->back,
					AD3552R_REG_ADDR_SH_REFERENCE_CONFIG,
					&val, 1);
	if (err)
		return err;

	val = AD3552R_TRANSFER_INIT;
	err = iio_backend_bus_reg_write(st->back,
					AD3552R_REG_ADDR_TRANSFER_REGISTER,
					&val, 1);
	if (err)
		return err;

	err = iio_backend_data_source_set(st->back, 0, IIO_BACKEND_EXTERNAL);
	if (err)
		return err;

	err = iio_backend_data_source_set(st->back, 1, IIO_BACKEND_EXTERNAL);
	if (err)
		return err;

	err = ad3552r_get_ref_voltage(st->dev, &val);
	if (err)
		return err;

	err = axi3552r_qspi_update_reg_bits(st->back,
				AD3552R_REG_ADDR_SH_REFERENCE_CONFIG,
				AD3552R_MASK_REFERENCE_VOLTAGE_SEL,
				val, 1);
	if (err)
		return err;

	err = ad3552r_get_drive_strength(st->dev, &val);
	if (!err) {
		err = axi3552r_qspi_update_reg_bits(st->back,
					AD3552R_REG_ADDR_INTERFACE_CONFIG_D,
					AD3552R_MASK_SDO_DRIVE_STRENGTH,
					val, 1);
		if (err)
			return err;
	}

	child = device_get_named_child_node(st->dev, "channel");
	if (!child)
		return -EINVAL;

	err = ad3552r_get_output_range(st->dev, st->chip_id, child, &range);
	if (!err)
		return ad3552r_axi_set_output_range(st, range);

	if (err != -ENOENT)
		return err;

	/* Try to get custom range */
	err = ad3552r_get_custom_gain(st->dev, child,
					&gs_p, &gs_n, &rfb, &goffs);
	if (err)
		return err;

	ad3552r_calc_custom_gain(gs_p, gs_n, goffs, &reg);

	offset = abs((s32)goffs);

	err = iio_backend_bus_reg_write(st->back,
					AD3552R_REG_ADDR_CH_OFFSET(0),
					&offset, 1);
	if (err)
		return dev_err_probe(st->dev, err,
					"Error writing register\n");

	err = iio_backend_bus_reg_write(st->back,
					AD3552R_REG_ADDR_CH_OFFSET(1),
					&offset, 1);
	if (err)
		return dev_err_probe(st->dev, err,
					"Error writing register\n");

	err = iio_backend_bus_reg_write(st->back,
					AD3552R_REG_ADDR_CH_GAIN(0),
					&reg, 1);
	if (err)
		return dev_err_probe(st->dev, err,
					"Error writing register\n");

	err = iio_backend_bus_reg_write(st->back,
					AD3552R_REG_ADDR_CH_GAIN(1),
					&reg, 1);
	if (err)
		return dev_err_probe(st->dev, err,
					"Error writing register\n");

	return 0;
}

static const struct iio_buffer_setup_ops ad3552r_axi_buffer_setup_ops = {
	.postenable = ad3552r_axi_buffer_postenable,
	.predisable = ad3552r_axi_buffer_predisable,
};

static int ad3552r_set_synchronous_mode_status(struct iio_dev *indio_dev,
					       const struct iio_chan_spec *chan,
					       unsigned int status)
{
	struct ad3552r_axi_state *st = iio_priv(indio_dev);

	st->synced_transfer = status;

	return 0;
}

static int ad3552r_get_synchronous_mode_status(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan)
{
	struct ad3552r_axi_state *st = iio_priv(indio_dev);

	return st->synced_transfer;
}

static const char *const synchronous_mode_status[] = {
	[AD3552R_NO_SYNC] = "no_sync",
	[AD3552R_EXT_SYNC_ARM] = "ext_sync_arm",
};

static const struct iio_enum ad3552r_synchronous_mode_enum = {
	.items = synchronous_mode_status,
	.num_items = ARRAY_SIZE(synchronous_mode_status),
	.get = ad3552r_get_synchronous_mode_status,
	.set = ad3552r_set_synchronous_mode_status,
};

static const struct iio_chan_spec_ext_info ad3552r_axi_ext_info[] = {
	IIO_ENUM("synchronous_mode", IIO_SHARED_BY_TYPE,
		 &ad3552r_synchronous_mode_enum),
	IIO_ENUM_AVAILABLE("synchronous_mode", IIO_SHARED_BY_TYPE,
			   &ad3552r_synchronous_mode_enum),
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

	ret = devm_iio_backend_enable(&pdev->dev, st->back);
	if (ret)
		return ret;

	indio_dev->name = "ad3552r";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->setup_ops = &ad3552r_axi_buffer_setup_ops;
	indio_dev->channels = ad3552r_axi_channels;
	indio_dev->num_channels = ARRAY_SIZE(ad3552r_axi_channels);
	indio_dev->info = &ad3552r_axi_info;

	ret = devm_iio_backend_request_buffer(&pdev->dev, st->back, indio_dev);
	if (ret)
		return ret;

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
