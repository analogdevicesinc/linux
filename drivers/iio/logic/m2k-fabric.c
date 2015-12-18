#include <linux/module.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

enum {
	M2K_FABRIC_GPIO_EN_SC1_LG,
	M2K_FABRIC_GPIO_EN_SC1_HG,
	M2K_FABRIC_GPIO_EN_SC2_LG,
	M2K_FABRIC_GPIO_EN_SC2_HG,
	M2K_FABRIC_GPIO_EN_SC_CAL1,
	M2K_FABRIC_GPIO_EN_SC_CAL2,
	M2K_FABRIC_GPIO_SC_CAL_MUX0,
	M2K_FABRIC_GPIO_SC_CAL_MUX1,
	M2K_FABRIC_GPIO_EN_AWG_CAL,
	M2K_FABRIC_GPIO_EN_AWG_50OHM,
	M2K_FABRIC_GPIO_MAX
};

enum m2k_fabric_calibration_mode {
	M2K_FABRIC_CALIBRATION_MODE_NONE,
	M2K_FABRIC_CALIBRATION_MODE_ADC_VREF,
	M2K_FABRIC_CALIBRATION_MODE_ADC_GND,
	M2K_FABRIC_CALIBRATION_MODE_DAC,
};

enum m2k_fabric_output_impedeance {
	M2K_FABRIC_IMPEDANCE_0OHM,
	M2K_FABRIC_IMPEDANCE_50OHM,
};

enum m2k_fabric_adc_gain {
	M2K_FABRIC_ADC_GAIN_LOW,
	M2K_FABRIC_ADC_GAIN_HIGH,
};

struct m2k_fabric {
	struct mutex lock;

	enum m2k_fabric_calibration_mode calibration_mode;
	enum m2k_fabric_output_impedeance output_impedance;
	enum m2k_fabric_adc_gain adc_gain[2];

	struct gpio_desc *switch_gpios[M2K_FABRIC_GPIO_MAX];
	struct gpio_desc *usr_pow_gpio;
};

static int m2k_fabric_switch_values_open[] = {
	[M2K_FABRIC_GPIO_EN_SC1_LG] = 0,
	[M2K_FABRIC_GPIO_EN_SC1_HG] = 0,
	[M2K_FABRIC_GPIO_EN_SC2_LG] = 0,
	[M2K_FABRIC_GPIO_EN_SC2_HG] = 0,
	[M2K_FABRIC_GPIO_EN_SC_CAL1] = 1,
	[M2K_FABRIC_GPIO_EN_SC_CAL2] = 1,
	[M2K_FABRIC_GPIO_SC_CAL_MUX0] = 0,
	[M2K_FABRIC_GPIO_SC_CAL_MUX1] = 0,
	[M2K_FABRIC_GPIO_EN_AWG_CAL] = 1,
	[M2K_FABRIC_GPIO_EN_AWG_50OHM] = 1,
};

static void m2k_fabric_update_switch_settings(struct m2k_fabric *m2k_fabric)
{
	int values[M2K_FABRIC_GPIO_MAX];

	switch (m2k_fabric->calibration_mode) {
	case M2K_FABRIC_CALIBRATION_MODE_NONE:
		values[M2K_FABRIC_GPIO_SC_CAL_MUX0] = 1;
		values[M2K_FABRIC_GPIO_SC_CAL_MUX1] = 1;
		break;
	case M2K_FABRIC_CALIBRATION_MODE_ADC_VREF:
		values[M2K_FABRIC_GPIO_SC_CAL_MUX0] = 1;
		values[M2K_FABRIC_GPIO_SC_CAL_MUX1] = 0;
		break;
	case M2K_FABRIC_CALIBRATION_MODE_ADC_GND:
		values[M2K_FABRIC_GPIO_SC_CAL_MUX0] = 0;
		values[M2K_FABRIC_GPIO_SC_CAL_MUX1] = 1;
		break;
	case M2K_FABRIC_CALIBRATION_MODE_DAC:
		values[M2K_FABRIC_GPIO_SC_CAL_MUX0] = 0;
		values[M2K_FABRIC_GPIO_SC_CAL_MUX1] = 0;
		break;
	}

	switch (m2k_fabric->calibration_mode) {
	case M2K_FABRIC_CALIBRATION_MODE_NONE:
		if (m2k_fabric->adc_gain[0] == M2K_FABRIC_ADC_GAIN_LOW)
			values[M2K_FABRIC_GPIO_EN_SC1_LG] = 1;
		else
			values[M2K_FABRIC_GPIO_EN_SC1_LG] = 0;
		if (m2k_fabric->adc_gain[1] == M2K_FABRIC_ADC_GAIN_LOW)
			values[M2K_FABRIC_GPIO_EN_SC2_LG] = 1;
		else
			values[M2K_FABRIC_GPIO_EN_SC2_LG] = 0;
		values[M2K_FABRIC_GPIO_EN_SC1_HG] = !values[M2K_FABRIC_GPIO_EN_SC1_LG];
		values[M2K_FABRIC_GPIO_EN_SC2_HG] = !values[M2K_FABRIC_GPIO_EN_SC2_LG];
		values[M2K_FABRIC_GPIO_EN_SC_CAL1] = 1;
		values[M2K_FABRIC_GPIO_EN_SC_CAL2] = 1;
		if (m2k_fabric->output_impedance == M2K_FABRIC_IMPEDANCE_0OHM)
			values[M2K_FABRIC_GPIO_EN_AWG_CAL] = 0;
		else
			values[M2K_FABRIC_GPIO_EN_AWG_CAL] = 1;
		values[M2K_FABRIC_GPIO_EN_AWG_50OHM] = !values[M2K_FABRIC_GPIO_EN_AWG_CAL];
		break;
	default:
		values[M2K_FABRIC_GPIO_EN_SC1_LG] = 0;
		values[M2K_FABRIC_GPIO_EN_SC1_HG] = 0;
		values[M2K_FABRIC_GPIO_EN_SC2_LG] = 0;
		values[M2K_FABRIC_GPIO_EN_SC2_HG] = 0;
		values[M2K_FABRIC_GPIO_EN_SC_CAL1] = 0;
		values[M2K_FABRIC_GPIO_EN_SC_CAL2] = 0;
		values[M2K_FABRIC_GPIO_EN_AWG_CAL] = 1;
		values[M2K_FABRIC_GPIO_EN_AWG_50OHM] = 1;
		break;
	}

	/* Open up all first to avoid shorts */
	gpiod_set_array_value_cansleep(M2K_FABRIC_GPIO_MAX,
		m2k_fabric->switch_gpios, m2k_fabric_switch_values_open);

	gpiod_set_array_value_cansleep(M2K_FABRIC_GPIO_MAX,
		m2k_fabric->switch_gpios, values);
}

static int m2k_fabric_set_calibration_mode(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int val)
{
	struct m2k_fabric *m2k_fabric = iio_priv(indio_dev);

	mutex_lock(&m2k_fabric->lock);
	m2k_fabric->calibration_mode = val;
	m2k_fabric_update_switch_settings(m2k_fabric);
	mutex_unlock(&m2k_fabric->lock);

	return 0;
}

static int m2k_fabric_get_calibration_mode(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct m2k_fabric *m2k_fabric = iio_priv(indio_dev);

	return m2k_fabric->calibration_mode;
}

static const char * const m2k_fabric_calibration_mode_items[] = {
	[M2K_FABRIC_CALIBRATION_MODE_NONE] = "none",
	[M2K_FABRIC_CALIBRATION_MODE_ADC_VREF] = "adc_vref",
	[M2K_FABRIC_CALIBRATION_MODE_ADC_GND] = "adc_gnd",
	[M2K_FABRIC_CALIBRATION_MODE_DAC] = "dac",
};

static const struct iio_enum m2k_fabric_calibration_mode_enum = {
	.items = m2k_fabric_calibration_mode_items,
	.num_items = ARRAY_SIZE(m2k_fabric_calibration_mode_items),
	.set = m2k_fabric_set_calibration_mode,
	.get = m2k_fabric_get_calibration_mode,
};

static int m2k_fabric_set_adc_gain(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int val)
{
	struct m2k_fabric *m2k_fabric = iio_priv(indio_dev);

	mutex_lock(&m2k_fabric->lock);
	m2k_fabric->adc_gain[chan->address] = val;
	m2k_fabric_update_switch_settings(m2k_fabric);
	mutex_unlock(&m2k_fabric->lock);

	return 0;
}

static int m2k_fabric_get_adc_gain(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct m2k_fabric *m2k_fabric = iio_priv(indio_dev);

	return m2k_fabric->adc_gain[chan->address];
}

static const char * const m2k_fabric_adc_gain_items[] = {
	[M2K_FABRIC_ADC_GAIN_LOW] = "low",
	[M2K_FABRIC_ADC_GAIN_HIGH] = "high",
};

static const struct iio_enum m2k_fabric_adc_gain_enum = {
	.items = m2k_fabric_adc_gain_items,
	.num_items = ARRAY_SIZE(m2k_fabric_adc_gain_items),
	.set = m2k_fabric_set_adc_gain,
	.get = m2k_fabric_get_adc_gain,
};

static int m2k_fabric_set_output_impedance(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int val)
{
	struct m2k_fabric *m2k_fabric = iio_priv(indio_dev);

	mutex_lock(&m2k_fabric->lock);
	m2k_fabric->output_impedance = val;
	m2k_fabric_update_switch_settings(m2k_fabric);
	mutex_unlock(&m2k_fabric->lock);

	return 0;
}

static int m2k_fabric_get_output_impedance(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct m2k_fabric *m2k_fabric = iio_priv(indio_dev);

	return m2k_fabric->output_impedance;
}

static const char * const m2k_fabric_output_impedance_items[] = {
	[M2K_FABRIC_IMPEDANCE_0OHM] = "0",
	[M2K_FABRIC_IMPEDANCE_50OHM] = "50",
};

static const struct iio_enum m2k_fabric_output_impedance_enum = {
	.items = m2k_fabric_output_impedance_items,
	.num_items = ARRAY_SIZE(m2k_fabric_output_impedance_items),
	.set = m2k_fabric_set_output_impedance,
	.get = m2k_fabric_get_output_impedance,
};

static const struct iio_chan_spec_ext_info m2k_fabric_rx_ext_info[] = {
	IIO_ENUM("calibration_mode", IIO_SHARED_BY_ALL,
		&m2k_fabric_calibration_mode_enum),
	IIO_ENUM_AVAILABLE_SHARED("calibration_mode", IIO_SHARED_BY_ALL,
		&m2k_fabric_calibration_mode_enum),
	IIO_ENUM("gain", IIO_SEPARATE, &m2k_fabric_adc_gain_enum),
	IIO_ENUM_AVAILABLE("gain", &m2k_fabric_adc_gain_enum),
	{}
};

static const struct iio_chan_spec_ext_info m2k_fabric_tx_ext_info[] = {
	IIO_ENUM("calibration_mode", IIO_SHARED_BY_ALL,
		&m2k_fabric_calibration_mode_enum),
	IIO_ENUM_AVAILABLE_SHARED("calibration_mode", IIO_SHARED_BY_ALL,
		&m2k_fabric_calibration_mode_enum),
	IIO_ENUM("impedance", IIO_SHARED_BY_TYPE, &m2k_fabric_output_impedance_enum),
	IIO_ENUM_AVAILABLE("impedance", &m2k_fabric_output_impedance_enum),
	{}
};

#define M2K_FABRIC_TX_CHAN(x) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.channel = (x), \
	.address = (x), \
	.output = 1, \
	.scan_index = 0, \
	.ext_info = m2k_fabric_tx_ext_info, \
}

#define M2K_FABRIC_RX_CHAN(x) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.channel = (x), \
	.address = (x), \
	.scan_index = 0, \
	.ext_info = m2k_fabric_rx_ext_info, \
}

static const struct iio_chan_spec m2k_fabric_chan_spec[] = {
	M2K_FABRIC_RX_CHAN(0),
	M2K_FABRIC_RX_CHAN(1),
	M2K_FABRIC_TX_CHAN(0),
	M2K_FABRIC_TX_CHAN(1),
};

static const struct iio_info m2k_fabric_iio_info = {
};

static const char * const m2k_fabric_gpio_names[] = {
	[M2K_FABRIC_GPIO_EN_SC1_LG] = "en-sc1-lg",
	[M2K_FABRIC_GPIO_EN_SC1_HG] = "en-sc1-hg",
	[M2K_FABRIC_GPIO_EN_SC2_LG] = "en-sc2-lg",
	[M2K_FABRIC_GPIO_EN_SC2_HG] = "en-sc2-hg",
	[M2K_FABRIC_GPIO_EN_SC_CAL1] = "en-sc-cal1",
	[M2K_FABRIC_GPIO_EN_SC_CAL2] = "en-sc-cal2",
	[M2K_FABRIC_GPIO_SC_CAL_MUX0] = "sc-cal-mux0",
	[M2K_FABRIC_GPIO_SC_CAL_MUX1] = "sc-cal-mux1",
	[M2K_FABRIC_GPIO_EN_AWG_CAL] = "en-awg-cal",
	[M2K_FABRIC_GPIO_EN_AWG_50OHM] = "en-awg-50ohm",
};

static int m2k_fabric_probe(struct platform_device *pdev)
{
	struct m2k_fabric *m2k_fabric;
	struct iio_dev *indio_dev;
	unsigned int i;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*m2k_fabric));
	if (!indio_dev)
		return -ENOMEM;

	m2k_fabric = iio_priv(indio_dev);

	for (i = 0; i < ARRAY_SIZE(m2k_fabric_gpio_names); i++) {
		m2k_fabric->switch_gpios[i] = devm_gpiod_get(&pdev->dev,
				m2k_fabric_gpio_names[i], GPIOD_OUT_LOW);
		if (IS_ERR(m2k_fabric->switch_gpios[i]))
			return PTR_ERR(m2k_fabric->switch_gpios[i]);
	}

	m2k_fabric->usr_pow_gpio = devm_gpiod_get(&pdev->dev, "en-usr-pow",
			GPIOD_OUT_HIGH);
	if (IS_ERR(m2k_fabric->usr_pow_gpio))
		return PTR_ERR(m2k_fabric->usr_pow_gpio);

	mutex_init(&m2k_fabric->lock);

	m2k_fabric_update_switch_settings(m2k_fabric);

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = "m2k-fabric";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &m2k_fabric_iio_info;
	indio_dev->channels = m2k_fabric_chan_spec,
	indio_dev->num_channels = ARRAY_SIZE(m2k_fabric_chan_spec);

	platform_set_drvdata(pdev, indio_dev);

	return devm_iio_device_register(&pdev->dev, indio_dev);
}

static const struct of_device_id m2k_fabric_of_match[] = {
	{ .compatible = "adi,m2k-fabric" },
	{},
};

static struct platform_driver m2k_fabric_driver = {
	.driver = {
		.name = "m2k_fabric",
		.of_match_table = m2k_fabric_of_match,
	},
	.probe = m2k_fabric_probe,
};
module_platform_driver(m2k_fabric_driver);

MODULE_AUTHOR("");
MODULE_DESCRIPTION("");
MODULE_LICENSE("GPL v2");
