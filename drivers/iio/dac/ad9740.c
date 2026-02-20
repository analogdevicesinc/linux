// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices AD9740/AD9742/AD9744/AD9748
 * 8/10/12/14-Bit, 210 MSPS Digital-to-Analog Converters
 *
 * Copyright 2025 Analog Devices Inc.
 *
 * MSB Alignment Requirements:
 * The AD9740 family DACs expect MSB-aligned data in their internal 14-bit bus.
 * This driver handles the alignment by configuring the IIO scan_type.shift field
 * to indicate how many bits the data should be shifted left.
 *
 * Data Format per Device:
 * - AD9748 (8-bit):  Bits [15:8] of 16-bit word → DAC bits [13:6], shift=8
 * - AD9740 (10-bit): Bits [15:6] of 16-bit word → DAC bits [13:4], shift=6
 * - AD9742 (12-bit): Bits [15:4] of 16-bit word → DAC bits [13:2], shift=4
 * - AD9744 (14-bit): Bits [13:0] of 16-bit word → DAC bits [13:0], shift=0
 *
 * Note: AD9744 extracts bits[13:0] directly, not bits[15:2] as the formula
 * (16 - resolution) would suggest. Userspace should provide data in the
 * lower 14 bits for AD9744, and MSB-aligned for other variants.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/backend.h>
#include <linux/iio/buffer.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/property.h>

/*
 * AD9740 family has no configuration registers - they are simple parallel DACs.
 * All configuration is done via the AXI DAC IP core in the FPGA.
 */

struct ad9740_chip_info {
	const char *name;
	unsigned int resolution;	/* DAC resolution in bits */
	const struct iio_chan_spec *channels;
	int num_channels;
};

struct ad9740_state {
	struct device *dev;
	struct iio_backend *back;
	struct gpio_desc *reset_gpio;
	const struct ad9740_chip_info *chip_info;
	/* Data format: true = 2's complement, false = offset binary */
	bool twos_complement;
	/* Protects backend I/O operations from concurrent accesses. */
	struct mutex lock;
};

enum ad9740_sources {
	AD9740_SRC_NORMAL,
	AD9740_SRC_DDS,
	AD9740_SRC_RAMP,
};

static const char * const dbgfs_attr_source[] = {
	[AD9740_SRC_NORMAL] = "normal",
	[AD9740_SRC_DDS] = "dds",
	[AD9740_SRC_RAMP] = "ramp",
};

static int ad9740_set_data_source(struct ad9740_state *st,
				  enum iio_backend_data_source type)
{
	int ret;
	const char *type_str;

	switch (type) {
	case IIO_BACKEND_INTERNAL_CONTINUOUS_WAVE:
		type_str = "DDS";
		break;
	case IIO_BACKEND_INTERNAL_RAMP_16BIT:
		type_str = "RAMP";
		break;
	case IIO_BACKEND_EXTERNAL:
		type_str = "DMA";
		break;
	default:
		type_str = "UNKNOWN";
		break;
	}

	dev_info(st->dev, "Setting data source to %s (type=%d)\n", type_str, type);

	ret = iio_backend_data_source_set(st->back, 0, type);
	if (ret)
		dev_err(st->dev, "Failed to set data source to %s: %d\n", type_str, ret);
	else
		dev_info(st->dev, "Data source set to %s successfully\n", type_str);

	return ret;
}

static int ad9740_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	return -EINVAL;
}

static int ad9740_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long mask)
{
	return -EINVAL;
}

static int ad9740_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ad9740_state *st = iio_priv(indio_dev);
	struct iio_backend_data_fmt fmt = {
		.sign_extend = false,
		.enable = true,
	};
	int ret;

	dev_info(st->dev, "========================================\n");
	dev_info(st->dev, "Buffer enable requested - starting DMA streaming\n");
	dev_info(st->dev, "========================================\n");

	mutex_lock(&st->lock);

	/* Configure data format based on DT setting */
	if (st->twos_complement) {
		fmt.type = IIO_BACKEND_TWOS_COMPLEMENT;
		dev_info(st->dev, "Configuring data format: 2's complement\n");
	} else {
		fmt.type = IIO_BACKEND_OFFSET_BINARY;
		dev_info(st->dev, "Configuring data format: offset binary\n");
	}

	/*
	 * AD9740 DACs expect data aligned in a 14-bit internal bus:
	 * - AD9748 (8-bit):  Data from bits [15:8] → DAC [13:6], shift=8
	 * - AD9740 (10-bit): Data from bits [15:6] → DAC [13:4], shift=6
	 * - AD9742 (12-bit): Data from bits [15:4] → DAC [13:2], shift=4
	 * - AD9744 (14-bit): Data from bits [13:0] → DAC [13:0], shift=0
	 *
	 * The IIO framework handles this via the .shift field in scan_type.
	 * AD9744 is special: it extracts lower 14 bits directly (no MSB alignment).
	 */
	dev_info(st->dev, "Setting backend data format (%u-bit MSB-aligned in 16-bit container)\n",
		 st->chip_info->resolution);

	/* Configure data format: N-bit MSB-aligned in 16-bit container */
	ret = iio_backend_data_format_set(st->back, 0, &fmt);
	if (ret) {
		dev_err(st->dev, "Failed to set data format: %d\n", ret);
		goto err_unlock;
	}
	dev_info(st->dev, "Data format configured successfully (MSB-aligned)\n");

	/* Enable data streaming from DMA to DAC core */
	dev_info(st->dev, "Enabling backend data stream\n");
	ret = iio_backend_data_stream_enable(st->back);
	if (ret) {
		dev_err(st->dev, "Failed to enable data stream: %d\n", ret);
		goto err_unlock;
	}

	mutex_unlock(&st->lock);
	dev_info(st->dev, "========================================\n");
	dev_info(st->dev, "Buffer enabled - DMA streaming ACTIVE\n");
	dev_info(st->dev, "========================================\n");
	return 0;

err_unlock:
	mutex_unlock(&st->lock);
	dev_err(st->dev, "Buffer enable FAILED\n");
	return ret;
}

static int ad9740_buffer_predisable(struct iio_dev *indio_dev)
{
	struct ad9740_state *st = iio_priv(indio_dev);
	int ret;

	dev_info(st->dev, "========================================\n");
	dev_info(st->dev, "Buffer disable requested - stopping DMA streaming\n");
	dev_info(st->dev, "========================================\n");

	mutex_lock(&st->lock);

	/* Disable data streaming */
	dev_info(st->dev, "Disabling backend data stream\n");
	ret = iio_backend_data_stream_disable(st->back);
	if (ret) {
		dev_err(st->dev, "Failed to disable data stream: %d\n", ret);
		mutex_unlock(&st->lock);
		dev_err(st->dev, "Buffer disable FAILED\n");
		return ret;
	}

	mutex_unlock(&st->lock);
	dev_info(st->dev, "========================================\n");
	dev_info(st->dev, "Buffer disabled - DMA streaming STOPPED\n");
	dev_info(st->dev, "========================================\n");
	return 0;
}

/* IIO extended info for data source control */
static ssize_t ad9740_ext_info_get_data_source(struct iio_dev *indio_dev,
						uintptr_t private,
						const struct iio_chan_spec *chan,
						char *buf)
{
	struct ad9740_state *st = iio_priv(indio_dev);
	enum iio_backend_data_source type;
	const char *src_str;
	int ret;

	dev_dbg(st->dev, "Reading data_source attribute\n");

	ret = iio_backend_data_source_get(st->back, 0, &type);
	if (ret) {
		dev_err(st->dev, "Failed to get data source: %d\n", ret);
		return ret;
	}

	switch (type) {
	case IIO_BACKEND_INTERNAL_CONTINUOUS_WAVE:
		src_str = dbgfs_attr_source[AD9740_SRC_DDS];
		break;
	case IIO_BACKEND_INTERNAL_RAMP_16BIT:
		src_str = dbgfs_attr_source[AD9740_SRC_RAMP];
		break;
	case IIO_BACKEND_EXTERNAL:
	default:
		src_str = dbgfs_attr_source[AD9740_SRC_NORMAL];
		break;
	}

	dev_dbg(st->dev, "Current data source: %s (type=%d)\n", src_str, type);

	return sysfs_emit(buf, "%s\n", src_str);
}

static ssize_t ad9740_ext_info_set_data_source(struct iio_dev *indio_dev,
						uintptr_t private,
						const struct iio_chan_spec *chan,
						const char *buf, size_t len)
{
	struct ad9740_state *st = iio_priv(indio_dev);
	enum iio_backend_data_source type;
	int ret;

	dev_info(st->dev, "User requesting data source change to: '%s'\n", buf);

	if (sysfs_streq(buf, "normal"))
		type = IIO_BACKEND_EXTERNAL;
	else if (sysfs_streq(buf, "dds"))
		type = IIO_BACKEND_INTERNAL_CONTINUOUS_WAVE;
	else if (sysfs_streq(buf, "ramp"))
		type = IIO_BACKEND_INTERNAL_RAMP_16BIT;
	else {
		dev_err(st->dev, "Invalid data source '%s'. Valid: normal, dds, ramp\n", buf);
		return -EINVAL;
	}

	ret = ad9740_set_data_source(st, type);
	if (ret)
		return ret;

	return len;
}

static ssize_t ad9740_ext_info_get_data_source_available(struct iio_dev *indio_dev,
							  uintptr_t private,
							  const struct iio_chan_spec *chan,
							  char *buf)
{
	ssize_t len = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(dbgfs_attr_source); i++)
		len += sysfs_emit_at(buf, len, "%s ", dbgfs_attr_source[i]);

	buf[len - 1] = '\n';

	return len;
}

static const struct iio_chan_spec_ext_info ad9740_ext_info[] = {
	{
		.name = "data_source",
		.read = ad9740_ext_info_get_data_source,
		.write = ad9740_ext_info_set_data_source,
		.shared = IIO_SEPARATE,
	},
	{
		.name = "data_source_available",
		.read = ad9740_ext_info_get_data_source_available,
		.shared = IIO_SHARED_BY_ALL,
	},
	{ }
};

static int ad9740_setup(struct ad9740_state *st)
{
	struct iio_backend_data_fmt fmt = {
		.sign_extend = false,
		.enable = true,
	};
	int ret;

	dev_info(st->dev, "Starting %s setup\n", st->chip_info->name);

	/*
	 * Configure data format based on DT setting.
	 * This must be done at probe time so both DMA and DDS modes work.
	 * The DDS (CORDIC) always outputs signed (2's complement) data,
	 * so the backend needs to know whether to convert it to unsigned
	 * (offset binary) for the DAC.
	 */
	if (st->twos_complement) {
		fmt.type = IIO_BACKEND_TWOS_COMPLEMENT;
		dev_info(st->dev, "Configuring data format: 2's complement (signed DAC)\n");
	} else {
		fmt.type = IIO_BACKEND_OFFSET_BINARY;
		dev_info(st->dev, "Configuring data format: offset binary (unsigned DAC)\n");
	}

	ret = iio_backend_data_format_set(st->back, 0, &fmt);
	if (ret) {
		dev_err(st->dev, "Failed to set data format: %d\n", ret);
		return ret;
	}

	/* Set data source to external (from DMA) */
	dev_info(st->dev, "Initializing data source to DMA mode\n");
	ret = ad9740_set_data_source(st, IIO_BACKEND_EXTERNAL);
	if (ret) {
		dev_err(st->dev, "Failed to set initial data source: %d\n", ret);
		return ret;
	}

	/*
	 * AD9740 family has no software-configurable registers.
	 * Hardware configuration (full-scale current, references, etc.)
	 * is done via external analog components on the board.
	 */

	dev_info(st->dev, "%s setup completed successfully\n", st->chip_info->name);
	return 0;
}

static const struct iio_buffer_setup_ops ad9740_buffer_setup_ops = {
	.postenable = ad9740_buffer_postenable,
	.predisable = ad9740_buffer_predisable,
};

#define AD9740_CHANNEL(ch, bits, shft) { \
	.type = IIO_ALTVOLTAGE, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
	.output = 1, \
	.indexed = 1, \
	.channel = (ch), \
	.scan_index = (ch), \
	.scan_type = { \
		.sign = 'u', \
		.realbits = (bits), \
		.storagebits = 16, \
		.shift = (shft), \
		.endianness = IIO_BE, \
	} \
}

static const struct iio_chan_spec ad9748_channels[] = {
	AD9740_CHANNEL(0, 8, 8),   /* 8-bit: shift=8, data in bits[15:8] */
};

static const struct iio_chan_spec ad9740_channels[] = {
	AD9740_CHANNEL(0, 10, 6),  /* 10-bit: shift=6, data in bits[15:6] */
};

static const struct iio_chan_spec ad9742_channels[] = {
	AD9740_CHANNEL(0, 12, 4),  /* 12-bit: shift=4, data in bits[15:4] */
};

static const struct iio_chan_spec ad9744_channels[] = {
	AD9740_CHANNEL(0, 14, 0),  /* 14-bit: shift=0, data in bits[13:0] */
};

static const struct iio_info ad9740_info = {
	.read_raw = &ad9740_read_raw,
	.write_raw = &ad9740_write_raw,
};

static const struct ad9740_chip_info ad9748_chip_info = {
	.name = "ad9748",
	.resolution = 8,
	.channels = ad9748_channels,
	.num_channels = ARRAY_SIZE(ad9748_channels),
};

static const struct ad9740_chip_info ad9740_chip_info = {
	.name = "ad9740",
	.resolution = 10,
	.channels = ad9740_channels,
	.num_channels = ARRAY_SIZE(ad9740_channels),
};

static const struct ad9740_chip_info ad9742_chip_info = {
	.name = "ad9742",
	.resolution = 12,
	.channels = ad9742_channels,
	.num_channels = ARRAY_SIZE(ad9742_channels),
};

static const struct ad9740_chip_info ad9744_chip_info = {
	.name = "ad9744",
	.resolution = 14,
	.channels = ad9744_channels,
	.num_channels = ARRAY_SIZE(ad9744_channels),
};

static int ad9740_probe(struct platform_device *pdev)
{
	struct ad9740_state *st;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (!indio_dev) {
		dev_err(&pdev->dev, "Failed to allocate IIO device\n");
		return -ENOMEM;
	}

	st = iio_priv(indio_dev);
	st->dev = &pdev->dev;

	/* Get chip info from device match data */
	st->chip_info = device_get_match_data(&pdev->dev);
	if (!st->chip_info) {
		dev_err(&pdev->dev, "Failed to get chip info\n");
		return -ENODEV;
	}

	dev_info(&pdev->dev, "%s probe starting (%u-bit DAC)\n",
		 st->chip_info->name, st->chip_info->resolution);

	mutex_init(&st->lock);
	dev_dbg(&pdev->dev, "Mutex initialized\n");

	/* Parse device tree properties */
	st->twos_complement = device_property_read_bool(&pdev->dev,
							"adi,twos-complement");
	dev_info(&pdev->dev, "Data format: %s\n",
		 st->twos_complement ? "2's complement" : "offset binary");

	/* Get IIO backend (AXI_AD9740 DAC core) */
	dev_info(&pdev->dev, "Getting IIO backend (AXI_AD9740)\n");
	st->back = devm_iio_backend_get(&pdev->dev, NULL);
	if (IS_ERR(st->back))
		return dev_err_probe(&pdev->dev, PTR_ERR(st->back),
				     "Failed to get backend\n");
	dev_info(&pdev->dev, "IIO backend acquired successfully\n");

	dev_info(&pdev->dev, "Enabling IIO backend\n");
	ret = devm_iio_backend_enable(&pdev->dev, st->back);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "Failed to enable backend\n");
	dev_info(&pdev->dev, "IIO backend enabled successfully\n");

	/* Get reset GPIO if present */
	dev_dbg(&pdev->dev, "Checking for reset GPIO\n");
	st->reset_gpio = devm_gpiod_get_optional(&pdev->dev, "reset",
						 GPIOD_OUT_HIGH);
	if (IS_ERR(st->reset_gpio)) {
		dev_err(&pdev->dev, "Failed to get reset GPIO: %ld\n",
			PTR_ERR(st->reset_gpio));
		return PTR_ERR(st->reset_gpio);
	}

	if (st->reset_gpio) {
		dev_info(&pdev->dev, "Reset GPIO found, performing hardware reset\n");
		/* Reset the device */
		gpiod_set_value_cansleep(st->reset_gpio, 1);
		msleep(10);
		gpiod_set_value_cansleep(st->reset_gpio, 0);
		msleep(10);
		dev_info(&pdev->dev, "Hardware reset completed\n");
	} else {
		dev_info(&pdev->dev, "No reset GPIO specified\n");
	}

	/* Make a modifiable copy of the channel spec for backend extension */
	struct iio_chan_spec *channels;
	const struct iio_chan_spec_ext_info *backend_ext_info;
	struct iio_chan_spec_ext_info *merged_ext_info;
	int num_our_ext_info, num_backend_ext_info, i, j;

	dev_info(&pdev->dev, "Creating modifiable channel spec\n");
	channels = devm_kmemdup(&pdev->dev, st->chip_info->channels,
				sizeof(struct iio_chan_spec) * st->chip_info->num_channels,
				GFP_KERNEL);
	if (!channels) {
		dev_err(&pdev->dev, "Failed to allocate channel spec\n");
		return -ENOMEM;
	}

	/* Extend channel with DDS controls from backend */
	dev_info(&pdev->dev, "Extending channel spec with backend DDS controls\n");
	ret = iio_backend_extend_chan_spec(st->back, &channels[0]);
	if (ret) {
		dev_err(&pdev->dev, "Failed to extend channel spec: %d\n", ret);
		return ret;
	}

	/* Merge our ext_info (data_source) with backend's ext_info (DDS) */
	backend_ext_info = channels[0].ext_info;

	/* Count entries in both arrays */
	num_our_ext_info = ARRAY_SIZE(ad9740_ext_info) - 1; /* exclude terminator */
	num_backend_ext_info = 0;
	if (backend_ext_info) {
		while (backend_ext_info[num_backend_ext_info].name)
			num_backend_ext_info++;
	}

	dev_info(&pdev->dev, "Merging ext_info: %d our attrs + %d backend attrs\n",
		 num_our_ext_info, num_backend_ext_info);

	/* Allocate merged array: our entries + backend entries + terminator */
	merged_ext_info = devm_kcalloc(&pdev->dev,
				       num_our_ext_info + num_backend_ext_info + 1,
				       sizeof(*merged_ext_info), GFP_KERNEL);
	if (!merged_ext_info) {
		dev_err(&pdev->dev, "Failed to allocate merged ext_info\n");
		return -ENOMEM;
	}

	/* Copy our ext_info first */
	for (i = 0; i < num_our_ext_info; i++)
		merged_ext_info[i] = ad9740_ext_info[i];

	/* Then append backend ext_info */
	for (j = 0; j < num_backend_ext_info; j++)
		merged_ext_info[i + j] = backend_ext_info[j];

	/* Terminator is already zero from kcalloc */
	channels[0].ext_info = merged_ext_info;
	dev_info(&pdev->dev, "Extended attributes merged successfully\n");

	dev_info(&pdev->dev, "Configuring IIO device structure\n");
	indio_dev->name = st->chip_info->name;
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_HARDWARE;
	indio_dev->setup_ops = &ad9740_buffer_setup_ops;
	indio_dev->channels = channels;
	indio_dev->num_channels = st->chip_info->num_channels;
	indio_dev->info = &ad9740_info;
	dev_info(&pdev->dev, "IIO device configured: %d channel(s), modes=0x%x\n",
		 indio_dev->num_channels, indio_dev->modes);

	/* Request DMA buffer from backend */
	dev_info(&pdev->dev, "Requesting DMA buffer from backend\n");
	ret = devm_iio_backend_request_buffer(&pdev->dev, st->back, indio_dev);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "Failed to request backend buffer\n");
	dev_info(&pdev->dev, "DMA buffer allocated successfully\n");

	/* Setup DAC and backend */
	ret = ad9740_setup(st);
	if (ret) {
		dev_err(&pdev->dev, "AD9740 setup failed: %d\n", ret);
		return ret;
	}

	/* Register IIO device */
	dev_info(&pdev->dev, "Registering IIO device\n");
	ret = devm_iio_device_register(&pdev->dev, indio_dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register IIO device: %d\n", ret);
		return ret;
	}

	dev_info(&pdev->dev, "========================================\n");
	dev_info(&pdev->dev, "%s %u-bit DAC registered successfully!\n",
		 st->chip_info->name, st->chip_info->resolution);
	dev_info(&pdev->dev, "========================================\n");
	dev_info(&pdev->dev, "Features:\n");
	dev_info(&pdev->dev, "  - Resolution: %u-bit\n", st->chip_info->resolution);
	dev_info(&pdev->dev, "  - DDS dual-tone generator\n");
	dev_info(&pdev->dev, "  - DMA streaming mode\n");
	dev_info(&pdev->dev, "  - Internal ramp pattern\n");
	dev_info(&pdev->dev, "Data source control:\n");
	dev_info(&pdev->dev, "  - out_voltage0_data_source (normal/dds/ramp)\n");
	dev_info(&pdev->dev, "  - out_voltage0_data_source_available\n");
	dev_info(&pdev->dev, "DDS controls:\n");
	dev_info(&pdev->dev, "  - out_voltage0_frequency0/1 (tone frequencies)\n");
	dev_info(&pdev->dev, "  - out_voltage0_scale0/1 (tone amplitudes)\n");
	dev_info(&pdev->dev, "  - out_voltage0_phase0/1 (tone phases)\n");
	dev_info(&pdev->dev, "========================================\n");

	return 0;
}

static const struct of_device_id ad9740_of_id[] = {
	{ .compatible = "adi,ad9748", .data = &ad9748_chip_info },
	{ .compatible = "adi,ad9740", .data = &ad9740_chip_info },
	{ .compatible = "adi,ad9742", .data = &ad9742_chip_info },
	{ .compatible = "adi,ad9744", .data = &ad9744_chip_info },
	{ }
};
MODULE_DEVICE_TABLE(of, ad9740_of_id);

static struct platform_driver ad9740_driver = {
	.driver = {
		.name = "ad9740",
		.of_match_table = ad9740_of_id,
	},
	.probe = ad9740_probe,
};
module_platform_driver(ad9740_driver);

MODULE_AUTHOR("Analog Devices Inc.");
MODULE_DESCRIPTION("Analog Devices AD9740/AD9742/AD9744/AD9748 DAC Driver with MSB Alignment");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(IIO_BACKEND);
