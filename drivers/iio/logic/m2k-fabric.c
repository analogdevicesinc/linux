#include <linux/module.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/clk.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

enum {
	/* Output */
	M2K_FABRIC_GPIO_EN_AWG1,
	M2K_FABRIC_GPIO_EN_AWG2,
	M2K_FABRIC_GPIO_OUTPUT_MAX,

	/* Input */
	M2K_FABRIC_GPIO_EN_SC1_LG = M2K_FABRIC_GPIO_OUTPUT_MAX,
	M2K_FABRIC_GPIO_EN_SC1_HG,
	M2K_FABRIC_GPIO_EN_SC2_LG,
	M2K_FABRIC_GPIO_EN_SC2_HG,
	M2K_FABRIC_GPIO_EN_SC_CAL1,
	M2K_FABRIC_GPIO_EN_SC1_CAL2,
	M2K_FABRIC_GPIO_EN_SC2_CAL2,
	M2K_FABRIC_GPIO_SC_CAL_MUX0,
	M2K_FABRIC_GPIO_SC_CAL_MUX1,
	M2K_FABRIC_GPIO_EN_SC1,
	M2K_FABRIC_GPIO_EN_SC2,

	M2K_FABRIC_GPIO_MAX,
};

enum m2k_fabric_calibration_mode {
	M2K_FABRIC_CALIBRATION_MODE_NONE,
	M2K_FABRIC_CALIBRATION_MODE_ADC_VREF1,
	M2K_FABRIC_CALIBRATION_MODE_ADC_VREF2,
	M2K_FABRIC_CALIBRATION_MODE_ADC_GND,
	M2K_FABRIC_CALIBRATION_MODE_DAC,
};

enum m2k_fabric_adc_gain {
	M2K_FABRIC_ADC_GAIN_LOW,
	M2K_FABRIC_ADC_GAIN_HIGH,
};

struct m2k_fabric {
	struct mutex lock;
	struct clk *clk;

	enum m2k_fabric_calibration_mode calibration_mode;
	enum m2k_fabric_adc_gain adc_gain[2];

	struct gpio_desc *switch_gpios[M2K_FABRIC_GPIO_MAX];
	struct gpio_desc *usr_pow_gpio[2];
	struct gpio_desc *done_led_overwrite_gpio;

	bool done_led_overwrite;
	bool user_supply_powerdown[2];

	bool sc_powerdown[2];
	bool awg_powerdown[2];

	bool revc;
	bool revd;
	bool reve;
};

static int m2k_fabric_switch_values_open[] = {
	/* Output */
	[M2K_FABRIC_GPIO_EN_AWG1] = 1,
	[M2K_FABRIC_GPIO_EN_AWG2] = 1,

	/* Input */
	[M2K_FABRIC_GPIO_EN_SC1_LG] = 0,
	[M2K_FABRIC_GPIO_EN_SC1_HG] = 0,
	[M2K_FABRIC_GPIO_EN_SC2_LG] = 0,
	[M2K_FABRIC_GPIO_EN_SC2_HG] = 0,
	[M2K_FABRIC_GPIO_EN_SC_CAL1] = 1,
	[M2K_FABRIC_GPIO_EN_SC1_CAL2] = 1,
	[M2K_FABRIC_GPIO_EN_SC2_CAL2] = 1,
	[M2K_FABRIC_GPIO_SC_CAL_MUX0] = 0,
	[M2K_FABRIC_GPIO_SC_CAL_MUX1] = 0,
	[M2K_FABRIC_GPIO_EN_SC1] = 1,
	[M2K_FABRIC_GPIO_EN_SC2] = 1,
};

static void m2k_fabric_update_switch_settings(struct m2k_fabric *m2k_fabric,
	bool update_input, bool update_output)
{
	int values[M2K_FABRIC_GPIO_MAX];
	unsigned int ngpios;
	unsigned int gpio_base;

	switch (m2k_fabric->calibration_mode) {
	case M2K_FABRIC_CALIBRATION_MODE_ADC_VREF1:
		values[M2K_FABRIC_GPIO_SC_CAL_MUX0] = 1;
		values[M2K_FABRIC_GPIO_SC_CAL_MUX1] = 0;
		break;
	case M2K_FABRIC_CALIBRATION_MODE_ADC_VREF2:
		values[M2K_FABRIC_GPIO_SC_CAL_MUX0] = 1;
		values[M2K_FABRIC_GPIO_SC_CAL_MUX1] = 1;
		break;
	case M2K_FABRIC_CALIBRATION_MODE_NONE:
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
		values[M2K_FABRIC_GPIO_EN_SC1_HG] =
			!values[M2K_FABRIC_GPIO_EN_SC1_LG];
		values[M2K_FABRIC_GPIO_EN_SC2_HG] =
			!values[M2K_FABRIC_GPIO_EN_SC2_LG];
		values[M2K_FABRIC_GPIO_EN_SC_CAL1] = 1;
		values[M2K_FABRIC_GPIO_EN_SC2_CAL2] = 1;
		values[M2K_FABRIC_GPIO_EN_SC2_CAL2] = 1;

		values[M2K_FABRIC_GPIO_EN_SC1] = m2k_fabric->sc_powerdown[0];
		values[M2K_FABRIC_GPIO_EN_SC2] = m2k_fabric->sc_powerdown[1];
		values[M2K_FABRIC_GPIO_EN_AWG1] = m2k_fabric->awg_powerdown[0];
		values[M2K_FABRIC_GPIO_EN_AWG2] = m2k_fabric->awg_powerdown[1];
		break;
	default:
		values[M2K_FABRIC_GPIO_EN_SC1_LG] = 0;
		values[M2K_FABRIC_GPIO_EN_SC1_HG] = 0;
		values[M2K_FABRIC_GPIO_EN_SC2_LG] = 0;
		values[M2K_FABRIC_GPIO_EN_SC2_HG] = 0;
		values[M2K_FABRIC_GPIO_EN_SC_CAL1] = 0;
		values[M2K_FABRIC_GPIO_EN_SC1_CAL2] = 0;
		values[M2K_FABRIC_GPIO_EN_SC2_CAL2] = 0;
		values[M2K_FABRIC_GPIO_EN_SC1] = 0;
		values[M2K_FABRIC_GPIO_EN_SC2] = 0;
		values[M2K_FABRIC_GPIO_EN_AWG1] = 0;
		values[M2K_FABRIC_GPIO_EN_AWG2] = 0;
		break;
	}

	if (update_output) {
		gpio_base = 0;
		ngpios = M2K_FABRIC_GPIO_OUTPUT_MAX;
	} else {
		gpio_base = M2K_FABRIC_GPIO_OUTPUT_MAX;
		ngpios = 0;
	}

	if (update_input) {
		ngpios += M2K_FABRIC_GPIO_MAX - M2K_FABRIC_GPIO_OUTPUT_MAX;

		if (m2k_fabric->revd || m2k_fabric->reve)
			ngpios--; /* skip M2K_FABRIC_GPIO_EN_SC2 */
	}

	/* Open up all first to avoid shorts */
	gpiod_set_array_value_cansleep(ngpios -
		(update_output ? M2K_FABRIC_GPIO_OUTPUT_MAX : 0),
		&m2k_fabric->switch_gpios[M2K_FABRIC_GPIO_OUTPUT_MAX],
		&m2k_fabric_switch_values_open[M2K_FABRIC_GPIO_OUTPUT_MAX]);

	gpiod_set_array_value_cansleep(ngpios,
		&m2k_fabric->switch_gpios[gpio_base],
		&values[gpio_base]);
}

static int m2k_fabric_set_calibration_mode(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int val)
{
	struct m2k_fabric *m2k_fabric = iio_priv(indio_dev);

	mutex_lock(&m2k_fabric->lock);
	if (m2k_fabric->calibration_mode != val) {
		m2k_fabric->calibration_mode = val;
		m2k_fabric_update_switch_settings(m2k_fabric, true, true);
	}
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
	[M2K_FABRIC_CALIBRATION_MODE_ADC_VREF1] = "adc_ref1",
	[M2K_FABRIC_CALIBRATION_MODE_ADC_VREF2] = "adc_ref2",
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
	if (m2k_fabric->adc_gain[chan->address] != val) {
		m2k_fabric->adc_gain[chan->address] = val;
		m2k_fabric_update_switch_settings(m2k_fabric, true, false);
	}
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

static ssize_t m2k_fabric_user_supply_read(struct iio_dev *indio_dev,
	uintptr_t private, const struct iio_chan_spec *chan, char *buf)
{
	struct m2k_fabric *m2k_fabric = iio_priv(indio_dev);

	if (chan->channel == 4)
		return sprintf(buf, "%d\n", m2k_fabric->done_led_overwrite);

	return sprintf(buf, "%d\n",
		       m2k_fabric->user_supply_powerdown[chan->channel - 2]);
}

static ssize_t m2k_fabric_user_supply_write(struct iio_dev *indio_dev,
	 uintptr_t private, const struct iio_chan_spec *chan,
	 const char *buf, size_t len)
{
	struct m2k_fabric *m2k_fabric = iio_priv(indio_dev);
	bool state;
	int ret;

	ret = strtobool(buf, &state);
	if (ret)
		return ret;

	if (chan->channel == 4) {
		gpiod_set_value_cansleep(m2k_fabric->done_led_overwrite_gpio,
					 !state);
		m2k_fabric->done_led_overwrite = state;
		return len;
	}

	gpiod_set_value_cansleep(m2k_fabric->usr_pow_gpio[chan->channel - 2],
				 !state);
	m2k_fabric->user_supply_powerdown[chan->channel - 2] = state;

	return len;
}

static ssize_t m2k_fabric_powerdown_read(struct iio_dev *indio_dev,
	uintptr_t private, const struct iio_chan_spec *chan, char *buf)
{
	struct m2k_fabric *m2k_fabric = iio_priv(indio_dev);
	bool state;


	if (private == 1) {
		state = !!clk_get_phase(m2k_fabric->clk);
	} else {
		if (chan->output)
			state = m2k_fabric->awg_powerdown[chan->channel];
		else
			state = m2k_fabric->sc_powerdown[chan->channel];
	}

	return sprintf(buf, "%d\n", state);
}

static ssize_t m2k_fabric_powerdown_write(struct iio_dev *indio_dev,
	 uintptr_t private, const struct iio_chan_spec *chan,
	 const char *buf, size_t len)
{
	struct m2k_fabric *m2k_fabric = iio_priv(indio_dev);
	bool state;
	int ret;

	ret = strtobool(buf, &state);
	if (ret)
		return ret;

	mutex_lock(&m2k_fabric->lock);
	if (private == 1) {
		/* REVISIT: Workaorund for PowerDown */
		clk_set_phase(m2k_fabric->clk, state ? 42 : 0);
	} else {
		if (chan->output) {
			if (m2k_fabric->awg_powerdown[chan->channel] != state) {
				m2k_fabric->awg_powerdown[chan->channel] = state;
				m2k_fabric_update_switch_settings(m2k_fabric,
								  false, true);
			}
		} else {
			if (m2k_fabric->sc_powerdown[chan->channel] != state) {
				m2k_fabric->sc_powerdown[chan->channel] = state;
				m2k_fabric_update_switch_settings(m2k_fabric,
								  true, false);
			}
		}
	}
	mutex_unlock(&m2k_fabric->lock);

	return len;
}


static const struct iio_chan_spec_ext_info m2k_fabric_user_supply_ext_info[] = {
	{
		.name = "powerdown",
		.read = m2k_fabric_user_supply_read,
		.write = m2k_fabric_user_supply_write,
		.shared = IIO_SEPARATE,
	},
	{}
};

static const struct iio_chan_spec_ext_info m2k_fabric_rx_ext_info_revc[] = {
	IIO_ENUM("calibration_mode", IIO_SHARED_BY_ALL,
		&m2k_fabric_calibration_mode_enum),
	IIO_ENUM_AVAILABLE_SHARED("calibration_mode", IIO_SHARED_BY_ALL,
		&m2k_fabric_calibration_mode_enum),
	IIO_ENUM("gain", IIO_SEPARATE, &m2k_fabric_adc_gain_enum),
	IIO_ENUM_AVAILABLE("gain", &m2k_fabric_adc_gain_enum),
	{
		.name = "powerdown",
		.read = m2k_fabric_powerdown_read,
		.write = m2k_fabric_powerdown_write,
		.shared = IIO_SEPARATE,
	},
	{}
};

static const struct iio_chan_spec_ext_info m2k_fabric_tx_ext_info[] = {
	IIO_ENUM("calibration_mode", IIO_SHARED_BY_ALL,
		&m2k_fabric_calibration_mode_enum),
	IIO_ENUM_AVAILABLE_SHARED("calibration_mode", IIO_SHARED_BY_ALL,
		&m2k_fabric_calibration_mode_enum),
	{
		.name = "powerdown",
		.read = m2k_fabric_powerdown_read,
		.write = m2k_fabric_powerdown_write,
		.shared = IIO_SEPARATE,
	},
	{}
};

#define M2K_FABRIC_RX_CHAN(x) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.channel = (x), \
	.address = (x), \
	.scan_index = 0, \
	.ext_info = m2k_fabric_rx_ext_info_revc, \
}

#define M2K_FABRIC_TX_CHAN(x) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.channel = (x), \
	.address = (x), \
	.output = 1, \
	.scan_index = 0, \
	.ext_info = m2k_fabric_tx_ext_info, \
}

static const struct iio_chan_spec m2k_fabric_chan_spec_revc[] = {
	M2K_FABRIC_RX_CHAN(0),
	M2K_FABRIC_RX_CHAN(1),
	M2K_FABRIC_TX_CHAN(0),
	M2K_FABRIC_TX_CHAN(1),
	{
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = 2,
		.extend_name = "user_supply",
		.output = 1,
		.scan_index = -1,
		.ext_info = m2k_fabric_user_supply_ext_info,
	}
};

static const struct iio_chan_spec_ext_info m2k_fabric_rx_ext_info_revd[] = {
	IIO_ENUM("calibration_mode", IIO_SHARED_BY_ALL,
		&m2k_fabric_calibration_mode_enum),
	IIO_ENUM_AVAILABLE_SHARED("calibration_mode", IIO_SHARED_BY_ALL,
		&m2k_fabric_calibration_mode_enum),
	IIO_ENUM("gain", IIO_SEPARATE, &m2k_fabric_adc_gain_enum),
	IIO_ENUM_AVAILABLE("gain", &m2k_fabric_adc_gain_enum),
	{
		.name = "powerdown",
		.read = m2k_fabric_powerdown_read,
		.write = m2k_fabric_powerdown_write,
		.shared = IIO_SHARED_BY_TYPE,
	},
	{
		.name = "clk_powerdown",
		.read = m2k_fabric_powerdown_read,
		.write = m2k_fabric_powerdown_write,
		.private = 1,
		.shared = IIO_SHARED_BY_ALL,
	},
	{}
};

#define M2K_FABRIC_RX_CHAN_REVD(x) { \
.type = IIO_VOLTAGE, \
.indexed = 1, \
.channel = (x), \
.address = (x), \
.scan_index = 0, \
.ext_info = m2k_fabric_rx_ext_info_revd, \
}

static const struct iio_chan_spec m2k_fabric_chan_spec_revd[] = {
	M2K_FABRIC_RX_CHAN_REVD(0),
	M2K_FABRIC_RX_CHAN_REVD(1),
	M2K_FABRIC_TX_CHAN(0),
	M2K_FABRIC_TX_CHAN(1),
	{
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = 2,
		.extend_name = "user_supply",
		.output = 1,
		.scan_index = -1,
		.ext_info = m2k_fabric_user_supply_ext_info,
	}
};

static const struct iio_chan_spec m2k_fabric_chan_spec_reve[] = {
	M2K_FABRIC_RX_CHAN_REVD(0),
	M2K_FABRIC_RX_CHAN_REVD(1),
	M2K_FABRIC_TX_CHAN(0),
	M2K_FABRIC_TX_CHAN(1),
	{
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = 2,
		.extend_name = "user_supply",
		.output = 1,
		.scan_index = -1,
		.ext_info = m2k_fabric_user_supply_ext_info,
	},
	{
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = 3,
		.extend_name = "user_supply",
		.output = 1,
		.scan_index = -1,
		.ext_info = m2k_fabric_user_supply_ext_info,
	},
	{
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = 4,
		.extend_name = "done_led_overwrite",
		.output = 1,
		.scan_index = -1,
		.ext_info = m2k_fabric_user_supply_ext_info,
	}
};

static const struct iio_info m2k_fabric_iio_info = {
};

static const char * const m2k_fabric_gpio_names_revc[] = {
	[M2K_FABRIC_GPIO_EN_SC1_LG] = "en-sc1-lg",
	[M2K_FABRIC_GPIO_EN_SC1_HG] = "en-sc1-hg",
	[M2K_FABRIC_GPIO_EN_SC2_LG] = "en-sc2-lg",
	[M2K_FABRIC_GPIO_EN_SC2_HG] = "en-sc2-hg",
	[M2K_FABRIC_GPIO_EN_SC_CAL1] = "en-sc-cal1",
	[M2K_FABRIC_GPIO_EN_SC1_CAL2] = "en-sc1-cal2",
	[M2K_FABRIC_GPIO_EN_SC2_CAL2] = "en-sc2-cal2",
	[M2K_FABRIC_GPIO_SC_CAL_MUX0] = "sc-cal-mux0",
	[M2K_FABRIC_GPIO_SC_CAL_MUX1] = "sc-cal-mux1",
	[M2K_FABRIC_GPIO_EN_AWG1] = "en-awg1",
	[M2K_FABRIC_GPIO_EN_AWG2] = "en-awg2",
	[M2K_FABRIC_GPIO_EN_SC1] = "en-sc1",
	[M2K_FABRIC_GPIO_EN_SC2] = "en-sc2",
};

static const char * const m2k_fabric_gpio_names_revd[] = {
	[M2K_FABRIC_GPIO_EN_SC1_LG] = "en-sc1-lg",
	[M2K_FABRIC_GPIO_EN_SC1_HG] = "en-sc1-hg",
	[M2K_FABRIC_GPIO_EN_SC2_LG] = "en-sc2-lg",
	[M2K_FABRIC_GPIO_EN_SC2_HG] = "en-sc2-hg",
	[M2K_FABRIC_GPIO_EN_SC_CAL1] = "en-sc-cal1",
	[M2K_FABRIC_GPIO_EN_SC1_CAL2] = "en-sc1-cal2",
	[M2K_FABRIC_GPIO_EN_SC2_CAL2] = "en-sc2-cal2",
	[M2K_FABRIC_GPIO_SC_CAL_MUX0] = "sc-cal-mux0",
	[M2K_FABRIC_GPIO_SC_CAL_MUX1] = "sc-cal-mux1",
	[M2K_FABRIC_GPIO_EN_AWG1] = "en-awg1",
	[M2K_FABRIC_GPIO_EN_AWG2] = "en-awg2",
	[M2K_FABRIC_GPIO_EN_SC1] = "en-sc",
};

static int m2k_fabric_gpios_init(struct device *dev,
				 struct m2k_fabric *m2k_fabric)
{
	const char * const *gpio_names = NULL;
	unsigned int num_gpio_names;
	int i;

	if (m2k_fabric->revc) {
		gpio_names = m2k_fabric_gpio_names_revc;
		num_gpio_names = ARRAY_SIZE(m2k_fabric_gpio_names_revc);
	} else if (m2k_fabric->revd || m2k_fabric->reve) {
		gpio_names = m2k_fabric_gpio_names_revd;
		num_gpio_names = ARRAY_SIZE(m2k_fabric_gpio_names_revd);
	}

	if (!gpio_names)
		return 0;

	for (i = 0; i < num_gpio_names; i++) {
		if (!gpio_names[i])
			continue;
		m2k_fabric->switch_gpios[i] = devm_gpiod_get(dev,
				gpio_names[i], GPIOD_OUT_LOW);
		if (IS_ERR(m2k_fabric->switch_gpios[i]))
			return PTR_ERR(m2k_fabric->switch_gpios[i]);
	}

	return 0;
}

static int m2k_fabric_probe(struct platform_device *pdev)
{
	struct m2k_fabric *m2k_fabric;
	struct iio_dev *indio_dev;
	bool revc, revd, reve, remain_powerdown;
	int ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*m2k_fabric));
	if (!indio_dev)
		return -ENOMEM;

	m2k_fabric = iio_priv(indio_dev);

	revc = of_property_read_bool(pdev->dev.of_node, "adi,revc");
	revd = of_property_read_bool(pdev->dev.of_node, "adi,revd");
	reve = of_property_read_bool(pdev->dev.of_node, "adi,reve");

	if (!revc && !revd  && !reve) {
		dev_err(&pdev->dev, "Unsupported revision\n");
		return -EINVAL;
	}

	m2k_fabric->revc = revc;
	m2k_fabric->revd = revd;
	m2k_fabric->reve = reve;

	m2k_fabric->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(m2k_fabric->clk))
		return PTR_ERR(m2k_fabric->clk);

	if (clk_prepare_enable(m2k_fabric->clk) < 0)
		return -EINVAL;

	ret = m2k_fabric_gpios_init(&pdev->dev, m2k_fabric);
	if (ret)
		return ret;

	m2k_fabric->usr_pow_gpio[0] = devm_gpiod_get(&pdev->dev, "en-usr-pow",
			GPIOD_OUT_HIGH);
	if (IS_ERR(m2k_fabric->usr_pow_gpio[0]))
		return PTR_ERR(m2k_fabric->usr_pow_gpio[0]);


	if (reve) {
		m2k_fabric->usr_pow_gpio[1] = devm_gpiod_get(&pdev->dev,
							     "en-usr-pow-neg",
							     GPIOD_OUT_HIGH);
		if (IS_ERR(m2k_fabric->usr_pow_gpio[1]))
			return PTR_ERR(m2k_fabric->usr_pow_gpio[1]);

		m2k_fabric->done_led_overwrite_gpio = devm_gpiod_get(&pdev->dev,
							"en-done-led-overwrite",
							GPIOD_OUT_HIGH);
		if (IS_ERR(m2k_fabric->done_led_overwrite_gpio))
			return PTR_ERR(m2k_fabric->done_led_overwrite_gpio);
	}

	remain_powerdown = of_property_read_bool(pdev->dev.of_node,
						 "adi,powerdown-enable");

	m2k_fabric->user_supply_powerdown[0] = remain_powerdown;
	m2k_fabric->user_supply_powerdown[1] = remain_powerdown;
	m2k_fabric->sc_powerdown[0] = remain_powerdown;
	m2k_fabric->sc_powerdown[1] = remain_powerdown;
	m2k_fabric->awg_powerdown[0] = remain_powerdown;
	m2k_fabric->awg_powerdown[1] = remain_powerdown;

	mutex_init(&m2k_fabric->lock);

	m2k_fabric_update_switch_settings(m2k_fabric, true, true);

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = "m2k-fabric";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &m2k_fabric_iio_info;
	if (m2k_fabric->revc) {
		indio_dev->channels = m2k_fabric_chan_spec_revc;
		indio_dev->num_channels = ARRAY_SIZE(m2k_fabric_chan_spec_revc);
	} else if (m2k_fabric->revd) {
		indio_dev->channels = m2k_fabric_chan_spec_revd;
		indio_dev->num_channels = ARRAY_SIZE(m2k_fabric_chan_spec_revd);
	} else if (m2k_fabric->reve) {
		indio_dev->channels = m2k_fabric_chan_spec_reve;
		indio_dev->num_channels = ARRAY_SIZE(m2k_fabric_chan_spec_reve);
	}

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
