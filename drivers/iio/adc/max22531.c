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

#define MAX22531_VREF_MV		1800 


enum max22531_id {
	max22530,
	max22531,
	max22532,
};

struct max22531_chip_info {
	const char* name;
};

static struct max22531_chip_info max22531_chip_info_tbl[] = {
	[max22530] = {
		.name = "max22530",
	},
	[max22531] = {
		.name = "max22531",
	},
	[max22532] = {
		.name = "max22532",
	}.
};

struct max22531 {
	struct spi_device *spi_dev;
	const struct max22531_chip_info *chip_info;
	struct regulator *vddl;
	struct regulator *vddpl;
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
	}

static const struct iio_chan_spec max22531_channels[] = {
	MAX22531_CHANNEL(0),
	MAX22531_CHANNEL(1),
	MAX22531_CHANNEL(2),
	MAX22531_CHANNEL(3),
};

static const struct regmap_config regmap_config = {
	.reg_bits = 6,
	.val_bits = 6,
	.write_flag_mask = BIT(1),
	.pad_bits = 1,
	.max_register = MAX22531_REG_CONTROL,
};

static int max22531_read_raw(struct iio_dev *indio_dev,
			 struct iio_chan_spec const *chan,
			 int *val, int *val2, long mask)
{
	struct max22531 *adc = iio_priv(indio_dev);
	int ret; 

	switch(mask) {
	case IIO_CHAN_INFO_RAW:
		switch(chan->channel) {
		case 0:
			reg = MAX22531_REG_ADC1;
			break;
		case 1:
			reg = MAX22531_REG_ADC2;
			break;
		case 2:
			reg = MAX22531_REG_ADC3;
			break;
		case 3:
			reg = MAX22531_REG_ADC3;
			break;
		default:
			return -EINVAL;
	}
	ret = regmap_read(adc->regmap, reg, val);
	if (ret) 
		return ret;
	return IIO_VAL_INT;
	
	case IIO_CHAN_INFO_SCALE:
		switch(chan->channel) {
		case 0:
			reg = MAX22531_REG_FADC1;
			break;
		case 1:
			reg = MAX22531_REG_FADC2;
			break;
		case 2:
			reg = MAX22531_REG_FADC3;
			break;
		case 3:
			reg = MAX22531_REG_FADC4;
			break;
		default:
			return -EINVAL;
	}
	
	reg = regmap_read(adc->regmap, reg, val);
	if (ret)
		return ret;
	return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = MAX22531_VREF_MV;
		*val2 = 12;

		return IIO_VAL_FRACTIONAL_LOG2;
	
	default:
		return -EINVAL;
	
	}
}

static const struct iio_info max22531_info = {
	.read_raw = max22531_read_raw,
};

static int max22531_probe(struct spi_device *spi)
{
	dev_info(&spi->dev, "MAX22531: probing ADC\n");

	unsigned int ret, prod_id;
	const struct max22531_chip_info *info;
	struct max22531 *adc;
	struct iio_dev *indio_dev;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*adc));
	if (!indio_dev)
		dev_err_probe(&spi->dev, -ENODEV,
				"MAX22531: Failed to allocate memory or IIO device.\n");

	adc = iio_priv(indio_dev);
	adc->spi_dev = spi;
	adc->chip_info = info;

	indio_dev->name = adc->chip_info->name;
	indio_dev->info = &max22531_info;
	indio_dev->channels = max22531_channels;
	indio_dev->num_channels = ARRAY_SIZE(max22531_channels);

	adc->regmap = devm_regmap_init_spi(spi, &regmap_config);
	if (IS_ERR(adc->regmap))
		return dev_err_probe(&spi->dev, PTR_ERR(adc->regmap),
				"regmap init failure\n");

	ret = regmap_read(adc->regmap, MAX22531_REG_PROD_ID, &prod_id);
	if (ret)
		return dev_err_probe(&spi->dev, PTR_ERR(adc->regmap),
				"Failed to read PROD_ID\n");
	else
		dev_info(&spi->dev, "MAX22531: Successfully read PROD_ID"
				": %d from the driver.\n", ret);

	adc->vddl = devm_regulator_get_enable(&spi->dev, "vddl");
	if (IS_ERR(adc->vddl))
		return dev_err_probe(&spi->dev, PTR_ERR(adc->vddl),
				"Failed to retrieve power logic supply\n");

	adc->vddpl = devm_regulator_get_enable(&spi->dev, "vddpl");
	if (IS_ERR(adc->vddpl))
		return dev_err_probe(&spi->dev, PTR_ERR(adc->vddpl),
		       "Failed to retrieve isolated DC-DC supply\n");

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct spi_device_id max22531_id[] = {
	{ "max22530", (kernel_ulong_t)&max22531_chip_info_tbl[max22530] },
	{ "max22531", (kernel_ulong_t)&max22531_chip_info_tbl[max22531] },
	{ "max22532", (kernel_ulong_t)&max22531_chip_info_tbl[max22532] },
	{ }
};
MODULE_DEVICE_TABLE(spi, max22531_id);

static const struct of_device_id max22531_spi_of_id[] = {
	{ .compatible = "adi,max22530",
		.data = &max22531_chip_info_tbl[max22530], },
	{ .compatible = "adi,max22531",
		.data = &max22531_chip_info_tbl[max22531], },
	{ .compatible = "adi,max22532",
		.data = &max22531_chip_info_tbl[max22532], },
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
