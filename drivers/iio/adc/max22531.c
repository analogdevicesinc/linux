#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#define MAX22531_REG_PROD_ID		0x00
#define MAX22531_REG_ADC1		0x01
#define MAX22531_REG_ADC2		0x02
#define MAX22531_REG_ADC3		0x03
#define MAX22531_REG_ADC4		0x04
#define MAX22531_REG_FADC1		0x05
#define MAX22531_REG_FADC2		0x06
#define MAX22531_REG_FADC3		0x07
#define MAX22531_REG_FADC4		0x08
#define MAX22531_REG_COUTHI1		0x09
#define MAX22531_REG_COUTHI2		0x0a
#define MAX22531_REG_COUTHI3		0x0b
#define MAX22531_REG_COUTHI4		0x0c
#define MAX22531_REG_COUTLO1		0x0d
#define MAX22531_REG_COUTLO2		0x0e
#define MAX22531_REG_COUTLO3		0x0f
#define MAX22531_REG_COUTLO4		0x10
#define MAX22531_REG_COUT_STATUS	0x11
#define MAX22531_REG_INTERRUPT_STATUS	0x12
#define MAX22531_REG_INTERRUPT_ENABLE	0x13
#define MAX22531_REG_CONTROL		0x14

enum max22531_id {
	max22531,
};

struct max22531 {
	struct spi_device *spi_dev;
	struct regulator *vref;
	struct regulator *vddl;
	struct regulator *vddf;
	struct regmap *regmap;
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

static const struct regmap_config regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.max_register = 0x14,
};

static int max22531_read_raw(struct iio_dev *indio_dev,
			 struct iio_chan_spec const *chan,
			 int *val, int *val2, long mask)
{
	struct regmap **regmap = iio_priv(indio_dev);
	int ret; 

	/* mock for now */
	switch(mask) {
	case IIO_CHAN_INFO_RAW:
		ret = regmap_read(*regmap, chan->address, val);
		if (ret) 
			return ret;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static const struct iio_info max22531_info = {
	.read_raw = max22531_read_raw,
};

static void max22531_regulator_disable(void *reg)
{
	regulator_disable(reg);
}

static int max22531_probe(struct spi_device *spi)
{
	dev_info(&spi->dev, "MAX22531: probing ADC\n");

	unsigned int ret, prod_id;
	struct max22531 *adc;
	struct iio_dev *indio_dev;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*adc));
	if (!indio_dev) {
		dev_err(&spi->dev, "MAX22531: Failed to allocate memory"
			       "for IIO device.\n");
		return -ENOMEM;
	}

	adc = iio_priv(indio_dev);
	adc->spi_dev = spi;
	
	indio_dev->name = "max22531";
	indio_dev->info = &max22531_info;
	indio_dev->channels = max22531_channels;
	indio_dev->num_channels = ARRAY_SIZE(max22531_channels);

	adc->regmap = devm_regmap_init_spi(spi, &regmap_config);
	if (IS_ERR(adc->regmap))
		dev_err(&spi->dev, "regmap init failure\n");

	ret = regmap_read(adc->regmap, MAX22531_REG_PROD_ID, &prod_id);
	if (ret)
		dev_err(&spi->dev, "Failed to read PROD_ID\n");
	else
		dev_info(&spi->dev, "MAX22531: Successfully read PROD_ID"
				": %d from the driver.\n", ret);

	adc->vref = devm_regulator_get(&spi->dev, "vref");
	if (IS_ERR(adc->vref))
		dev_err(&spi->dev, "Failed to retrieve vref\n");

	ret = regulator_enable(adc->vref);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, max22531_regulator_disable,
					adc->vref);
	if (ret)
		return ret;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct spi_device_id max22531_id[] = {
	{ "max22531" },
	{ }
};
MODULE_DEVICE_TABLE(spi, max22531_id);

static const struct of_device_id max22531_spi_of_id[] = {
	{ .compatible = "adi,max22531" },
	{ }
};
MODULE_DEVICE_TABLE(of, max22531_spi_of_id);

static struct spi_driver max22531_driver = {
	.driver = {
		.name = "max22531",
		.of_match_table = max22531_spi_of_id,
	},
	.probe		= max22531_probe,
	.id_table	= max22531_id,
};
module_spi_driver(max22531_driver);

MODULE_AUTHOR("Abhinav Jain <jain.abhinav177@gmail.com>");
MODULE_DESCRIPTION("MAX22531 ADC");
MODULE_LICENSE("GPL v2");
