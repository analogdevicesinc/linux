// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices AD3552R
 * Digital to Analog converter driver, High Speed version
 *
 * Copyright 2024 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/backend.h>
#include <linux/iio/buffer.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/units.h>
#include "ad3552r.h"
#include "ad3552r-hs.h"

/*
 * Important notes for register map access:
 * ========================================
 *
 * Register address space is divided in 2 regions, primary (config) and
 * secondary (DAC). Primary region can only be accessed in simple SPI mode,
 * with exception for ad355x models where setting QSPI pin high allows QSPI
 * access to both the regions.
 *
 * Due to the fact that ad3541/2r do not implement QSPI, for proper device
 * detection, HDL keeps "QSPI" pin level low at boot (see ad3552r manual, rev B
 * table 7, pin 31, digital input). For this reason, actually the working mode
 * between SPI, DSPI and QSPI must be set via software, configuring the target
 * DAC appropriately, together with the backend API to configure the bus mode
 * accordingly.
 *
 * Also, important to note that none of the three modes allow to read in DDR.
 *
 * In non-buffering operations, mode is set to simple SPI SDR for all primary
 * and secondary region r/w accesses, to avoid to switch the mode each time DAC
 * register is accessed (raw accesses, r/w), and to be able to dump registers
 * content (possible as non DDR only).
 * In buffering mode, driver sets best possible mode, D/QSPI and DDR.
 */

struct ad3552r_hs_state {
	const struct ad3552r_model_data *model_data;
	struct gpio_desc *reset_gpio;
	struct device *dev;
	struct iio_backend *back;
	bool single_channel;
	struct ad3552r_ch_data ch_data[AD3552R_MAX_CH];
	struct ad3552r_hs_platform_data *data;
	/* INTERFACE_CONFIG_D register cache, in DDR we cannot read values. */
	u32 config_d;
	/* Protects backend I/O operations from concurrent accesses. */
	struct mutex lock;
};

enum ad3552r_sources {
	AD3552R_SRC_NORMAL,
	AD3552R_SRC_RAMP_16BIT,
};

static const char * const dbgfs_attr_source[] = {
	[AD3552R_SRC_NORMAL] = "normal",
	[AD3552R_SRC_RAMP_16BIT] = "ramp-16bit",
};

static int ad3552r_hs_set_data_source(struct ad3552r_hs_state *st,
				      enum iio_backend_data_source type)
{
	int ret;

	ret = iio_backend_data_source_set(st->back, 0, type);

	return 0;
}


static int ad3552r_hs_read_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int *val, int *val2, long mask)
{
	return 0;
}

static int ad3552r_hs_write_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int val, int val2, long mask)
{

	return 0;
}

static int ad3552r_hs_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ad3552r_hs_state *st = iio_priv(indio_dev);
	struct iio_backend_data_fmt fmt = {
		.type = IIO_BACKEND_DATA_UNSIGNED
	};
	int loop_len, val, ret;

	ret = iio_backend_data_transfer_addr(st->back, val);

	ret = iio_backend_data_format_set(st->back, 0, &fmt);

	ret = iio_backend_data_stream_enable(st->back);

	return 0;
}

static int ad3552r_hs_buffer_predisable(struct iio_dev *indio_dev)
{
	struct ad3552r_hs_state *st = iio_priv(indio_dev);
	int ret;

	ret = iio_backend_data_stream_disable(st->back);
	if (ret)
		return ret;

	return 0;
}




static ssize_t ad3552r_hs_show_data_source(struct file *f, char __user *userbuf,
					   size_t count, loff_t *ppos)
{
	struct ad3552r_hs_state *st = file_inode(f)->i_private;
	enum iio_backend_data_source type;
	int idx, ret;

	ret = iio_backend_data_source_get(st->back, 0, &type);
	if (ret)
		return ret;

//	return simple_read_from_buffer(userbuf, count, ppos,
//				       dbgfs_attr_source[idx],
//				       strlen(dbgfs_attr_source[idx]));
	return 0;
}

static ssize_t ad3552r_hs_write_data_source(struct file *f,
					    const char __user *userbuf,
					    size_t count, loff_t *ppos)
{
	struct ad3552r_hs_state *st = file_inode(f)->i_private;
	char buf[64];
	int ret, source;

	ret = simple_write_to_buffer(buf, sizeof(buf) - 1, ppos, userbuf,
				     count);
	if (ret < 0)
		return ret;

	buf[count] = '\0';

	return 0;
}

static ssize_t ad3552r_hs_show_data_source_avail(struct file *f,
						 char __user *userbuf,
						 size_t count, loff_t *ppos)
{
	ssize_t len = 0;
	char buf[128];
	int i;

	for (i = 0; i < ARRAY_SIZE(dbgfs_attr_source); i++) {
		len += scnprintf(buf + len, PAGE_SIZE - len, "%s ",
				 dbgfs_attr_source[i]);
	}
	buf[len - 1] = '\n';

	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static const struct file_operations ad3552r_hs_data_source_fops = {
	.owner = THIS_MODULE,
	.write = ad3552r_hs_write_data_source,
	.read = ad3552r_hs_show_data_source,
};

static const struct file_operations ad3552r_hs_data_source_avail_fops = {
	.owner = THIS_MODULE,
	.read = ad3552r_hs_show_data_source_avail,
};

static int ad3552r_hs_setup(struct ad3552r_hs_state *st)
{
	u16 id;
	u16 gain = 0, offset = 0;
	u32 ch, val, range;
	int ret;

	ret = ad3552r_hs_set_data_source(st, IIO_BACKEND_EXTERNAL);
	if (ret)
		return ret;

	val = ret;

	return 0;
}

static const struct iio_buffer_setup_ops ad3552r_hs_buffer_setup_ops = {
	.postenable = ad3552r_hs_buffer_postenable,
	.predisable = ad3552r_hs_buffer_predisable,
};

#define AD3552R_CHANNEL(ch) { \
	.type = IIO_VOLTAGE, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | \
			      BIT(IIO_CHAN_INFO_SAMP_FREQ) | \
			      BIT(IIO_CHAN_INFO_SCALE) | \
			      BIT(IIO_CHAN_INFO_OFFSET), \
	.output = 1, \
	.indexed = 1, \
	.channel = (ch), \
	.scan_index = (ch), \
	.scan_type = { \
		.sign = 'u', \
		.realbits = 16, \
		.storagebits = 16, \
		.endianness = IIO_BE, \
	} \
}

static const struct iio_chan_spec ad3552r_hs_channels[] = {
	AD3552R_CHANNEL(0),
};

static const struct iio_info ad3552r_hs_info = {
	.read_raw = &ad3552r_hs_read_raw,
	.write_raw = &ad3552r_hs_write_raw,
};


static int ad3552r_hs_probe(struct platform_device *pdev)
{
	struct ad3552r_hs_state *st;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->dev = &pdev->dev;

	st->data = dev_get_platdata(st->dev);
	if (!st->data)
		return dev_err_probe(st->dev, -ENODEV, "No platform data !");

	st->back = devm_iio_backend_get(&pdev->dev, NULL);
	if (IS_ERR(st->back))
		return PTR_ERR(st->back);

	ret = devm_iio_backend_enable(&pdev->dev, st->back);
	if (ret)
		return ret;

	st->model_data = device_get_match_data(&pdev->dev);
	if (!st->model_data)
		return -ENODEV;

	indio_dev->name = "ad3552r";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->setup_ops = &ad3552r_hs_buffer_setup_ops;
	indio_dev->channels = ad3552r_hs_channels;
	indio_dev->num_channels = ARRAY_SIZE(ad3552r_hs_channels);
	indio_dev->info = &ad3552r_hs_info;

	ret = devm_iio_backend_request_buffer(&pdev->dev, st->back, indio_dev);
	if (ret)
		return ret;

	ret = ad3552r_hs_setup(st);
	if (ret)
		return ret;

	ret = devm_iio_device_register(&pdev->dev, indio_dev);
	if (ret)
		return ret;

	return ret;
}

static const struct of_device_id ad3552r_hs_of_id[] = {
	{ .compatible = "adi,ad3541r", .data = &ad3541r_model_data },
	{ .compatible = "adi,ad3542r", .data = &ad3542r_model_data },
	{ .compatible = "adi,ad3551r", .data = &ad3551r_model_data },
	{ .compatible = "adi,ad3552r", .data = &ad3552r_model_data },
	{ }
};
MODULE_DEVICE_TABLE(of, ad3552r_hs_of_id);

static struct platform_driver ad3552r_hs_driver = {
	.driver = {
		.name = "ad3552r-hs",
		.of_match_table = ad3552r_hs_of_id,
	},
	.probe = ad3552r_hs_probe,
};
module_platform_driver(ad3552r_hs_driver);

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_AUTHOR("Angelo Dureghello <adueghello@baylibre.com>");
MODULE_DESCRIPTION("AD3552R Driver - High Speed version");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(IIO_BACKEND);
MODULE_IMPORT_NS(IIO_AD3552R);
