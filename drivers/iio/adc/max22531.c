#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>

enum max22531_id {
	max22531,
};

struct max22531 {
	struct spi_device *spi;
	struct regulator *vref;
};

#define MAX22531_CHANNEL(ch)						\
	{								\
		.type = IIO_VOLTAGE,					\
		.indexed = 1,						\
		.channel = (ch),					\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
		.scan_index = ch,					\
		.output = 0,						\
	}

static const struct iio_chan_spec max22531_channels[] = {
	MAX22531_CHANNEL(0),
	MAX22531_CHANNEL(1),
	MAX22531_CHANNEL(2),
	MAX22531_CHANNEL(3),
	IIO_CHAN_SOFT_TIMESTAMP(2),
};

static const struct iio_info max22531_info = {
};

static int max22531_probe(struct spi_device *spi)
{
	pr_err("max22531: probe on\n");

	struct max22531 *adc;
	struct iio_dev *indio_dev;
	const struct spi_device_id *id = spi_get_device_id(spi);

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*adc));
	if (!indio_dev) {
		pr_err("max22531: Failed to allocate memory for IIO device.\n");
		return -ENOMEM;
	}

	adc = iio_priv(indio_dev);
	adc->spi = spi;
	
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->info = &max22531_info;
	indio_dev->channels = max22531_channels;
	indio_dev->num_channels = ARRAY_SIZE(max22531_channels);

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct spi_device_id max22531_id[] = {
	{ "max22531", max22531 },
	{}
};
MODULE_DEVICE_TABLE(spi, max22531_id);

static const struct of_device_id max22531_dt_ids[] = {
	{ .compatible = "maxim,max22531" },
	{},
};
MODULE_DEVICE_TABLE(of, max22531_dt_ids);

static struct spi_driver max22531_driver = {
	.driver = {
		.name = "max22531",
		.of_match_table = max22531_dt_ids,
	},
	.probe = max22531_probe,
	.id_table = max22531_id,
};
module_spi_driver(max22531_driver);

MODULE_AUTHOR("Abhinav Jain <jain.abhinav177@gmail.com>");
MODULE_DESCRIPTION("MAX22531 ADC");
MODULE_LICENSE("GPL v2");
