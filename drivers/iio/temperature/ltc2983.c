// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Devices LTC2983 Multi-Sensor Digital Temperature Measurement System
 * driver
 *
 * Copyright 2019 Analog Devices Inc.
 */
#include <linux/bitfield.h>
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/iio/iio.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>

/* register map */
#define LTC2983_STATUS_REG			0x0000
#define LTC2983_TEMP_RES_START_REG		0x0010
#define LTC2983_TEMP_RES_END_REG		0x005F
#define LTC2983_GLOBAL_CONFIG_REG		0x00F0
#define LTC2983_MULT_CHANNEL_START_REG		0x00F4
#define LTC2983_MULT_CHANNEL_END_REG		0x00F7
#define LTC2983_MUX_CONFIG_REG			0x00FF
#define LTC2983_CHAN_ASSIGN_START_REG		0x0200
#define LTC2983_CHAN_ASSIGN_END_REG		0x024F
#define LTC2983_CUST_SENS_TBL_START_REG		0x0250
#define LTC2983_CUST_SENS_TBL_END_REG		0x03CF

#define LTC2983_DIFFERENTIAL_CHAN_MIN		2
#define LTC2983_MAX_CHANNELS_NR			20
#define LTC2983_MIN_CHANNELS_NR			1
#define LTC2983_SLEEP				0x97
#define LTC2983_CUSTOM_STEINHART_SIZE		24
#define LTC2983_CUSTOM_SENSOR_ENTRY_SZ		6
#define LTC2983_CUSTOM_STEINHART_ENTRY_SZ	4

#define LTC2983_CHAN_START_ADDR(chan) \
			(((chan - 1) * 4) + LTC2983_CHAN_ASSIGN_START_REG)
#define LTC2983_CHAN_RES_ADDR(chan) \
			(((chan - 1) * 4) + LTC2983_TEMP_RES_START_REG)
#define LTC2983_THERMOCOUPLE_DIFF_MASK		BIT(3)
#define LTC2983_THERMISTOR_DIFF_MASK		BIT(2)
#define LTC2983_DIODE_DIFF_MASK			BIT(2)
#define LTC2983_RTD_4_WIRE_MASK			BIT(3)
#define LTC2983_RTD_ROTATION_MASK		BIT(1)
#define LTC2983_RTD_KELVIN_R_SENSE_MASK		GENMASK(3, 2)

#define LTC2983_COMMON_HARD_FAULT_MASK	GENMASK(31, 30)
#define LTC2983_COMMON_SOFT_FAULT_MASK	GENMASK(27, 25)

#define	LTC2983_STATUS_START_MASK	BIT(7)
#define	LTC2983_STATUS_START(x)		FIELD_PREP(LTC2983_STATUS_START_MASK, x)

#define	LTC2983_STATUS_CHAN_SEL_MASK	GENMASK(4, 0)
#define	LTC2983_STATUS_CHAN_SEL(x) \
				FIELD_PREP(LTC2983_STATUS_CHAN_SEL_MASK, x)

#define LTC2983_TEMP_UNITS_MASK		BIT(2)
#define LTC2983_TEMP_UNITS(x)		FIELD_PREP(LTC2983_TEMP_UNITS_MASK, x)

#define LTC2983_NOTCH_FREQ_MASK		GENMASK(1, 0)
#define LTC2983_NOTCH_FREQ(x)		FIELD_PREP(LTC2983_NOTCH_FREQ_MASK, x)

#define LTC2983_RES_VALID_MASK		BIT(24)
#define LTC2983_DATA_MASK		GENMASK(23, 0)
#define LTC2983_DATA_SIGN_BIT		23

#define LTC2983_CHAN_TYPE_MASK		GENMASK(31, 27)
#define LTC2983_CHAN_TYPE(x)		FIELD_PREP(LTC2983_CHAN_TYPE_MASK, x)

/* cold junction for thermocouples and rsense for rtd's and thermistor's */
#define LTC2983_CHAN_ASSIGN_MASK	GENMASK(26, 22)
#define LTC2983_CHAN_ASSIGN(x)		FIELD_PREP(LTC2983_CHAN_ASSIGN_MASK, x)

#define LTC2983_CUSTOM_LEN_MASK		GENMASK(5, 0)
#define LTC2983_CUSTOM_LEN(x)		FIELD_PREP(LTC2983_CUSTOM_LEN_MASK, x)

#define LTC2983_CUSTOM_ADDR_MASK	GENMASK(11, 6)
#define LTC2983_CUSTOM_ADDR(x)		FIELD_PREP(LTC2983_CUSTOM_ADDR_MASK, x)

#define LTC2983_THERMOCOUPLE_CFG_MASK	GENMASK(21, 18)
#define LTC2983_THERMOCOUPLE_CFG(x) \
				FIELD_PREP(LTC2983_THERMOCOUPLE_CFG_MASK, x)
#define LTC2983_THERMOCOUPLE_HARD_FAULT_MASK	GENMASK(31, 29)
#define LTC2983_THERMOCOUPLE_SOFT_FAULT_MASK	GENMASK(28, 25)

#define LTC2983_RTD_CFG_MASK		GENMASK(21, 18)
#define LTC2983_RTD_CFG(x)		FIELD_PREP(LTC2983_RTD_CFG_MASK, x)
#define LTC2983_RTD_EXC_CURRENT_MASK	GENMASK(17, 14)
#define LTC2983_RTD_EXC_CURRENT(x) \
				FIELD_PREP(LTC2983_RTD_EXC_CURRENT_MASK, x)
#define LTC2983_RTD_CURVE_MASK		GENMASK(13, 12)
#define LTC2983_RTD_CURVE(x)		FIELD_PREP(LTC2983_RTD_CURVE_MASK, x)

#define LTC2983_THERMISTOR_CFG_MASK	GENMASK(21, 19)
#define LTC2983_THERMISTOR_CFG(x) \
				FIELD_PREP(LTC2983_THERMISTOR_CFG_MASK, x)
#define LTC2983_THERMISTOR_EXC_CURRENT_MASK	GENMASK(18, 15)
#define LTC2983_THERMISTOR_EXC_CURRENT(x) \
			FIELD_PREP(LTC2983_THERMISTOR_EXC_CURRENT_MASK, x)

#define LTC2983_DIODE_CFG_MASK		GENMASK(26, 24)
#define LTC2983_DIODE_CFG(x)		FIELD_PREP(LTC2983_DIODE_CFG_MASK, x)
#define LTC2983_DIODE_EXC_CURRENT_MASK	GENMASK(23, 22)
#define LTC2983_DIODE_EXC_CURRENT(x) \
				FIELD_PREP(LTC2983_DIODE_EXC_CURRENT_MASK, x)
#define LTC2983_DIODE_IDEAL_FACTOR_MASK	GENMASK(21, 0)
#define LTC2983_DIODE_IDEAL_FACTOR(x) \
				FIELD_PREP(LTC2983_DIODE_IDEAL_FACTOR_MASK, x)

#define LTC2983_R_SENSE_VAL_MASK	GENMASK(26, 0)
#define LTC2983_R_SENSE_VAL(x)		FIELD_PREP(LTC2983_R_SENSE_VAL_MASK, x)

#define LTC2983_ADC_SINGLE_ENDED_MASK	BIT(26)
#define LTC2983_ADC_SINGLE_ENDED(x) \
				FIELD_PREP(LTC2983_ADC_SINGLE_ENDED_MASK, x)

enum {
	LTC2983_SENSOR_THERMOCOUPLE = 1,
	LTC2983_SENSOR_THERMOCOUPLE_CUSTOM = 9,
	LTC2983_SENSOR_RTD = 10,
	LTC2983_SENSOR_RTD_CUSTOM = 18,
	LTC2983_SENSOR_THERMISTOR = 19,
	LTC2983_SENSOR_THERMISTOR_STEINHART = 26,
	LTC2983_SENSOR_THERMISTOR_CUSTOM = 27,
	LTC2983_SENSOR_DIODE = 28,
	LTC2983_SENSOR_SENSE_RESISTOR = 29,
	LTC2983_SENSOR_DIRECT_ADC = 30,
};

#define to_thermocouple(_sensor) \
		container_of(_sensor, struct ltc2983_thermocouple, sensor)

#define to_rtd(_sensor) \
		container_of(_sensor, struct ltc2983_rtd, sensor)

#define to_thermistor(_sensor) \
		container_of(_sensor, struct ltc2983_thermistor, sensor)

#define to_diode(_sensor) \
		container_of(_sensor, struct ltc2983_diode, sensor)

#define to_rsense(_sensor) \
		container_of(_sensor, struct ltc2983_rsense, sensor)

#define to_adc(_sensor) \
		container_of(_sensor, struct ltc2983_adc, sensor)

struct ltc2983_data {
	struct regmap *regmap;
	struct spi_device *spi;
	struct mutex lock;
	struct completion completion;
	struct iio_chan_spec *iio_chan;
	struct ltc2983_sensor **sensors;
	u32 mux_delay_config;
	u32 filter_notch_freq;
	u16 custom_table_size;
	u8 num_channels;
	u8 iio_channels;
	bool temp_farenheit;
	bool reset;
};

struct ltc2983_sensor {
	int (*fault_handler)(const struct ltc2983_data *st, const u32 result);
	int (*assign_chan)(struct ltc2983_data *st,
			   const struct ltc2983_sensor *sensor);
	const char *name;
	/* specifies the sensor channel */
	u32 chan;
	/* sensor type */
	u32 type;
};

struct ltc2983_custom_sensor {
	/* raw table sensor data */
	u8 *table;
	size_t size;
	/* address offset */
	s8 offset;
	bool is_steinhart;
};

struct ltc2983_thermocouple {
	struct ltc2983_sensor sensor;
	struct ltc2983_custom_sensor *custom;
	u32 sensor_config;
	u32 cold_junction_chan;
};

struct ltc2983_rtd {
	struct ltc2983_sensor sensor;
	struct ltc2983_custom_sensor *custom;
	u32 sensor_config;
	u32 r_sense_chan;
	u32 excitation_current;
	u32 rtd_curve;
};

struct ltc2983_thermistor {
	struct ltc2983_sensor sensor;
	struct ltc2983_custom_sensor *custom;
	u32 sensor_config;
	u32 r_sense_chan;
	u32 excitation_current;
};

struct ltc2983_diode {
	struct ltc2983_sensor sensor;
	u32 sensor_config;
	u32 excitation_current;
	u32 ideal_factor_value;
};

struct ltc2983_rsense {
	struct ltc2983_sensor sensor;
	u32 r_sense_val;
};

struct ltc2983_adc {
	struct ltc2983_sensor sensor;
	bool single_ended;
};

static int __ltc2983_fault_handler(const struct ltc2983_data *st,
				   const u32 result, const u32 hard_mask,
				   const u32 soft_mask)
{
	const struct device *dev = &st->spi->dev;

	if (result & hard_mask) {
		dev_err(dev, "Invalid conversion: Sensor HARD fault\n");
		return -EIO;
	} else if (result & soft_mask) {
		/* just print a warning */
		dev_warn(dev, "Suspicious conversion: Sensor SOFT fault\n");
	}

	return 0;
}

static int __ltc2983_chan_assign_common(const struct ltc2983_data *st,
					const struct ltc2983_sensor *sensor,
					u32 chan_val)
{
	u32 reg = LTC2983_CHAN_START_ADDR(sensor->chan);
	__be32 __chan_val;

	chan_val |= LTC2983_CHAN_TYPE(sensor->type);
	dev_dbg(&st->spi->dev, "Assign %s, reg:0x%04X, val:0x%08X\n",
							sensor->name,
							reg, chan_val);
	__chan_val = cpu_to_be32(chan_val);
	return regmap_bulk_write(st->regmap, reg, &__chan_val,
				 sizeof(__chan_val));
}

static int __ltc2983_chan_custom_sensor_assign(struct ltc2983_data *st,
					  struct ltc2983_custom_sensor *custom,
					  u32 *chan_val)
{
	u32 reg;
	u8 mult = custom->is_steinhart ? LTC2983_CUSTOM_STEINHART_ENTRY_SZ :
		LTC2983_CUSTOM_SENSOR_ENTRY_SZ;
	const struct device *dev = &st->spi->dev;
	/*
	 * custom->size holds the raw size of the table. However, when
	 * configuring the sensor channel, we must write the number of
	 * entries of the table minus 1. For steinhart sensors 0 is written
	 * since the size is constant!
	 */
	const u8 len = custom->is_steinhart ? 0 :
		(custom->size / LTC2983_CUSTOM_SENSOR_ENTRY_SZ) - 1;
	/*
	 * Check if the offset was assigned already. It should be for steinhart
	 * sensors. When coming from sleep, it should be assigned for all.
	 */
	if (custom->offset < 0) {
		/*
		 * This needs to be done again here because, from the moment
		 * when this test was done (successfully) for this custom
		 * sensor, a steinhart sensor might have been added changing
		 * custom_table_size...
		 */
		if (st->custom_table_size + custom->size >
		    (LTC2983_CUST_SENS_TBL_END_REG -
		     LTC2983_CUST_SENS_TBL_START_REG) + 1) {
			dev_err(dev,
				"Not space left(%d) for new custom sensor(%d)",
							st->custom_table_size,
							custom->size);
			return -EINVAL;
		}

		custom->offset = st->custom_table_size /
					LTC2983_CUSTOM_SENSOR_ENTRY_SZ;
		st->custom_table_size += custom->size;
	}

	reg = (custom->offset * mult) + LTC2983_CUST_SENS_TBL_START_REG;

	*chan_val |= LTC2983_CUSTOM_LEN(len);
	*chan_val |= LTC2983_CUSTOM_ADDR(custom->offset);
	dev_dbg(dev, "Assign custom sensor, reg:0x%04X, off:%d, sz:%d",
							reg, custom->offset,
							custom->size);
	/* write custom sensor table */
	return regmap_bulk_write(st->regmap, reg, custom->table, custom->size);
}

static struct ltc2983_custom_sensor *__ltc2983_custom_sensor_new(
						struct ltc2983_data *st,
						const struct device_node *np,
						const bool is_steinhart)
{
	struct ltc2983_custom_sensor *new_custom;
	u8 index, n_entries, tbl = 0;
	struct device *dev = &st->spi->dev;
	/*
	 * For custom steinhart, the full u32 is taken. For all the others
	 * the MSB is discarded.
	 */
	const u8 n_size = (is_steinhart == true) ? 4 : 3;

	n_entries = of_property_count_elems_of_size(np, "adi,custom-sensor",
						sizeof(u32));
	/* n_entries must be an even number */
	if (!n_entries || (n_entries % 2) != 0) {
		dev_err(dev, "Number of entries either 0 or not even\n");
		return ERR_PTR(-EINVAL);
	}

	new_custom = devm_kzalloc(dev, sizeof(*new_custom), GFP_KERNEL);
	if (!new_custom)
		return ERR_PTR(-ENOMEM);

	new_custom->size = n_entries * n_size;
	/* check Steinhart size */
	if (is_steinhart && new_custom->size != LTC2983_CUSTOM_STEINHART_SIZE) {
		dev_err(dev, "Steinhart sensors size(%d) must be 24",
							new_custom->size);
		return ERR_PTR(-EINVAL);
	}
	/* Check space on the table. */
	if (st->custom_table_size + new_custom->size >
	    (LTC2983_CUST_SENS_TBL_END_REG -
	     LTC2983_CUST_SENS_TBL_START_REG) + 1) {
		dev_err(dev, "No space left(%d) for new custom sensor(%d)",
				st->custom_table_size, new_custom->size);
		return ERR_PTR(-EINVAL);
	}

	/* allocate the table */
	new_custom->table = devm_kzalloc(dev, new_custom->size, GFP_KERNEL);
	if (!new_custom->table)
		return ERR_PTR(-ENOMEM);

	for (index = 0; index < n_entries; index++) {
		u32 temp = 0, j;

		of_property_read_u32_index(np, "adi,custom-sensor", index,
					   &temp);
		for (j = 0; j < n_size; j++)
			new_custom->table[tbl++] =
				temp >> (8 * (n_size - j - 1));
	}

	new_custom->is_steinhart = is_steinhart;
	/*
	 * This is done to first add all the steinhart sensors to the table,
	 * in order to maximize the table usage. If we mix adding steinhart
	 * with the other sensors, we might have to do some roundup to make
	 * sure that sensor_addr - 0x250(start address) is a multiple of 4
	 * (for steinhart), and a multiple of 6 for all the other sensors.
	 * Since we have const 24 bytes for steinhart sensors and 24 is
	 * also a multiple of 6, we guarantee that the first non-steinhart
	 * sensor will sit in a correct address without the need of filling
	 * addresses.
	 */
	if (is_steinhart) {
		new_custom->offset = st->custom_table_size /
					LTC2983_CUSTOM_STEINHART_ENTRY_SZ;
		st->custom_table_size += new_custom->size;
	} else {
		/* mark as unset. This is checked later on the assign phase */
		new_custom->offset = -1;
	}

	return new_custom;
}

static int ltc2983_thermocouple_fault_handler(const struct ltc2983_data *st,
					      const u32 result)
{
	return __ltc2983_fault_handler(st, result,
				       LTC2983_THERMOCOUPLE_HARD_FAULT_MASK,
				       LTC2983_THERMOCOUPLE_SOFT_FAULT_MASK);
}

static int ltc2983_common_fault_handler(const struct ltc2983_data *st,
					const u32 result)
{
	return __ltc2983_fault_handler(st, result,
				       LTC2983_COMMON_HARD_FAULT_MASK,
				       LTC2983_COMMON_SOFT_FAULT_MASK);
}

static int ltc2983_thermocouple_assign_chan(struct ltc2983_data *st,
				const struct ltc2983_sensor *sensor)
{
	struct ltc2983_thermocouple *thermo = to_thermocouple(sensor);
	u32 chan_val;

	chan_val = LTC2983_CHAN_ASSIGN(thermo->cold_junction_chan);
	chan_val |= LTC2983_THERMOCOUPLE_CFG(thermo->sensor_config);

	if (thermo->custom) {
		int ret;

		ret = __ltc2983_chan_custom_sensor_assign(st, thermo->custom,
							  &chan_val);
		if (ret)
			return ret;
	}
	return __ltc2983_chan_assign_common(st, sensor, chan_val);
}

static int ltc2983_rtd_assign_chan(struct ltc2983_data *st,
				   const struct ltc2983_sensor *sensor)
{
	struct ltc2983_rtd *rtd = to_rtd(sensor);
	u32 chan_val;

	chan_val = LTC2983_CHAN_ASSIGN(rtd->r_sense_chan);
	chan_val |= LTC2983_RTD_CFG(rtd->sensor_config);
	chan_val |= LTC2983_RTD_EXC_CURRENT(rtd->excitation_current);
	chan_val |= LTC2983_RTD_CURVE(rtd->rtd_curve);

	if (rtd->custom) {
		int ret;

		ret = __ltc2983_chan_custom_sensor_assign(st, rtd->custom,
							  &chan_val);
		if (ret)
			return ret;
	}
	return __ltc2983_chan_assign_common(st, sensor, chan_val);
}

static int ltc2983_thermistor_assign_chan(struct ltc2983_data *st,
					  const struct ltc2983_sensor *sensor)
{
	struct ltc2983_thermistor *thermistor = to_thermistor(sensor);
	u32 chan_val;

	chan_val = LTC2983_CHAN_ASSIGN(thermistor->r_sense_chan);
	chan_val |= LTC2983_THERMISTOR_CFG(thermistor->sensor_config);
	chan_val |=
		LTC2983_THERMISTOR_EXC_CURRENT(thermistor->excitation_current);

	if (thermistor->custom) {
		int ret;

		ret = __ltc2983_chan_custom_sensor_assign(st,
							  thermistor->custom,
							  &chan_val);
		if (ret)
			return ret;
	}
	return __ltc2983_chan_assign_common(st, sensor, chan_val);
}

static int ltc2983_diode_assign_chan(struct ltc2983_data *st,
				     const struct ltc2983_sensor *sensor)
{
	struct ltc2983_diode *diode = to_diode(sensor);
	u32 chan_val;

	chan_val = LTC2983_DIODE_CFG(diode->sensor_config);
	chan_val |= LTC2983_DIODE_EXC_CURRENT(diode->excitation_current);
	chan_val |= LTC2983_DIODE_IDEAL_FACTOR(diode->ideal_factor_value);

	return __ltc2983_chan_assign_common(st, sensor, chan_val);
}

static int ltc2983_r_sense_assign_chan(struct ltc2983_data *st,
				       const struct ltc2983_sensor *sensor)
{
	struct ltc2983_rsense *rsense = to_rsense(sensor);
	u32 chan_val;

	chan_val = LTC2983_R_SENSE_VAL(rsense->r_sense_val);

	return __ltc2983_chan_assign_common(st, sensor, chan_val);
}

static int ltc2983_adc_assign_chan(struct ltc2983_data *st,
				   const struct ltc2983_sensor *sensor)
{
	struct ltc2983_adc *adc = to_adc(sensor);
	u32 chan_val;

	chan_val = LTC2983_ADC_SINGLE_ENDED(adc->single_ended);

	return __ltc2983_chan_assign_common(st, sensor, chan_val);
}

static struct ltc2983_sensor *ltc2983_thermocouple_new(
					const struct device_node *child,
					struct ltc2983_data *st,
					const struct ltc2983_sensor *sensor)
{
	struct ltc2983_thermocouple *thermo;
	struct device_node *phandle;

	thermo = devm_kzalloc(&st->spi->dev, sizeof(*thermo), GFP_KERNEL);
	if (!thermo)
		return ERR_PTR(-ENOMEM);

	of_property_read_u32(child, "adi,sensor-config",
				 &thermo->sensor_config);
	/* validate channel index */
	if (!(thermo->sensor_config & LTC2983_THERMOCOUPLE_DIFF_MASK) &&
	    sensor->chan < LTC2983_DIFFERENTIAL_CHAN_MIN) {
		dev_err(&st->spi->dev,
			"Invalid chann:%d for differential thermocouple",
								sensor->chan);
		return ERR_PTR(-EINVAL);
	}

	phandle = of_parse_phandle(child, "adi,cold-junction-handle", 0);
	if (phandle) {
		int ret;

		ret = of_property_read_u32(phandle, "reg",
					   &thermo->cold_junction_chan);
		if (ret) {
			/*
			 * This would be catched later but we can just return
			 * the error right away.
			 */
			dev_err(&st->spi->dev, "Property reg must be given\n");
			of_node_put(phandle);
			return ERR_PTR(-EINVAL);
		}
	}

	/* check custom sensor */
	if (sensor->type == LTC2983_SENSOR_THERMOCOUPLE_CUSTOM) {
		thermo->custom = __ltc2983_custom_sensor_new(st, child, false);
		if (IS_ERR(thermo->custom)) {
			of_node_put(phandle);
			return ERR_CAST(thermo->custom);
		}
	}

	/* set common parameters */
	thermo->sensor.name = "thermocouple";
	thermo->sensor.fault_handler = ltc2983_thermocouple_fault_handler;
	thermo->sensor.assign_chan = ltc2983_thermocouple_assign_chan;

	of_node_put(phandle);
	return &thermo->sensor;
}

static struct ltc2983_sensor *ltc2983_rtd_new(const struct device_node *child,
					  struct ltc2983_data *st,
					  const struct ltc2983_sensor *sensor)
{
	struct ltc2983_rtd *rtd;
	int ret = 0;
	struct device *dev = &st->spi->dev;
	struct device_node *phandle;

	rtd = devm_kzalloc(dev, sizeof(*rtd), GFP_KERNEL);
	if (!rtd)
		return ERR_PTR(-ENOMEM);

	phandle = of_parse_phandle(child, "adi,rsense-handle", 0);
	if (!phandle) {
		dev_err(dev, "Property adi,rsense-handle missing or invalid");
		return ERR_PTR(-EINVAL);
	}

	ret = of_property_read_u32(phandle, "reg",
				&rtd->r_sense_chan);
	if (ret) {
		dev_err(dev, "Property reg must be given\n");
		goto fail;
	}

	of_property_read_u32(child, "adi,sensor-config", &rtd->sensor_config);
	/*
	 * rtd channel indexes are a bit more complicated to validate.
	 * For 4wire RTD with rotation, the channel selection cannot be
	 * >=19 since the chann + 1 is used in this configuration.
	 * For 4wire RTDs with kelvin rsense, the rsense channel cannot be
	 * <=1 since chanel - 1 and channel - 2 are used.
	 */
	if (rtd->sensor_config & LTC2983_RTD_4_WIRE_MASK) {
		/* 4-wire */
		u8 min = LTC2983_DIFFERENTIAL_CHAN_MIN,
			max = LTC2983_MAX_CHANNELS_NR;

		if (rtd->sensor_config & LTC2983_RTD_ROTATION_MASK)
			max = LTC2983_MAX_CHANNELS_NR - 1;

		if (((rtd->sensor_config & LTC2983_RTD_KELVIN_R_SENSE_MASK)
		     == LTC2983_RTD_KELVIN_R_SENSE_MASK) &&
		    (rtd->r_sense_chan <=  min)) {
			/* kelvin rsense*/
			dev_err(dev,
				"Invalid rsense chann:%d to use in kelvin rsense",
							rtd->r_sense_chan);

			ret = -EINVAL;
			goto fail;
		}

		if (sensor->chan < min || sensor->chan > max) {
			dev_err(dev, "Invalid chann:%d for the rtd config",
								sensor->chan);

			ret = -EINVAL;
			goto fail;
		}
	} else {
		/* same as differential case */
		if (sensor->chan < LTC2983_DIFFERENTIAL_CHAN_MIN) {
			dev_err(&st->spi->dev,
				"Invalid chann:%d for RTD", sensor->chan);

			ret = -EINVAL;
			goto fail;
		}
	}

	/* check custom sensor */
	if (sensor->type == LTC2983_SENSOR_RTD_CUSTOM) {
		rtd->custom = __ltc2983_custom_sensor_new(st, child, false);
		if (IS_ERR(rtd->custom)) {
			of_node_put(phandle);
			return ERR_CAST(rtd->custom);
		}
	}

	/* set common parameters */
	rtd->sensor.name = "rtd";
	rtd->sensor.fault_handler = ltc2983_common_fault_handler;
	rtd->sensor.assign_chan = ltc2983_rtd_assign_chan;

	ret = of_property_read_u32(child, "adi,excitation-current",
				   &rtd->excitation_current);
	if (ret)
		/* default to 1uA */
		rtd->excitation_current = 1;

	of_property_read_u32(child, "adi,rtd-curve", &rtd->rtd_curve);

	of_node_put(phandle);
	return &rtd->sensor;
fail:
	of_node_put(phandle);
	return ERR_PTR(ret);
}

static struct ltc2983_sensor *ltc2983_thermistor_new(
					const struct device_node *child,
					struct ltc2983_data *st,
					const struct ltc2983_sensor *sensor)
{
	struct ltc2983_thermistor *thermistor;
	struct device *dev = &st->spi->dev;
	struct device_node *phandle;
	int ret = 0;

	thermistor = devm_kzalloc(dev, sizeof(*thermistor), GFP_KERNEL);
	if (!thermistor)
		return ERR_PTR(-ENOMEM);

	phandle = of_parse_phandle(child, "adi,rsense-handle", 0);
	if (!phandle) {
		dev_err(dev, "Property adi,rsense-handle missing or invalid");
		return ERR_PTR(-EINVAL);
	}

	ret = of_property_read_u32(phandle, "reg",
				   &thermistor->r_sense_chan);
	if (ret) {
		dev_err(dev, "rsense channel must be configured...\n");
		goto fail;
	}

	of_property_read_u32(child, "adi,sensor-config",
			     &thermistor->sensor_config);
	/* validate channel index */
	if (!(thermistor->sensor_config & LTC2983_THERMISTOR_DIFF_MASK) &&
	    sensor->chan < LTC2983_DIFFERENTIAL_CHAN_MIN) {
		dev_err(&st->spi->dev,
			"Invalid chann:%d for differential thermistor",
								sensor->chan);
		ret = -EINVAL;
		goto fail;
	}

	/* check custom sensor */
	if (sensor->type >= LTC2983_SENSOR_THERMISTOR_STEINHART) {
		thermistor->custom = __ltc2983_custom_sensor_new(st, child,
			sensor->type == LTC2983_SENSOR_THERMISTOR_STEINHART ?
							  true : false);
		if (IS_ERR(thermistor->custom)) {
			of_node_put(phandle);
			return ERR_CAST(thermistor->custom);
		}
	}
	/* set common parameters */
	thermistor->sensor.name = "thermistor";
	thermistor->sensor.fault_handler = ltc2983_common_fault_handler;
	thermistor->sensor.assign_chan = ltc2983_thermistor_assign_chan;

	ret = of_property_read_u32(child, "adi,excitation-current",
				   &thermistor->excitation_current);
	if (ret)
		/* default to auto-range */
		thermistor->excitation_current = 0x0c;

	of_node_put(phandle);
	return &thermistor->sensor;
fail:
	of_node_put(phandle);
	return ERR_PTR(ret);
}

static struct ltc2983_sensor *ltc2983_diode_new(
					const struct device_node *child,
					const struct ltc2983_data *st,
					const struct ltc2983_sensor *sensor)
{
	struct ltc2983_diode *diode;

	diode = devm_kzalloc(&st->spi->dev, sizeof(*diode), GFP_KERNEL);
	if (!diode)
		return ERR_PTR(-ENOMEM);

	of_property_read_u32(child, "adi,sensor-config", &diode->sensor_config);
	/* validate channel index */
	if (!(diode->sensor_config & LTC2983_DIODE_DIFF_MASK) &&
	    sensor->chan < LTC2983_DIFFERENTIAL_CHAN_MIN) {
		dev_err(&st->spi->dev,
			"Invalid chann:%d for differential thermistor",
								sensor->chan);
		return ERR_PTR(-EINVAL);
	}
	/* set common parameters */
	diode->sensor.name = "diode";
	diode->sensor.fault_handler = ltc2983_common_fault_handler;
	diode->sensor.assign_chan = ltc2983_diode_assign_chan;

	of_property_read_u32(child, "adi,excitation-current",
			     &diode->excitation_current);

	of_property_read_u32(child, "adi,ideal-factor-value",
			     &diode->ideal_factor_value);

	return &diode->sensor;
}

static struct ltc2983_sensor *ltc2983_r_sense_new(struct device_node *child,
					struct ltc2983_data *st,
					const struct ltc2983_sensor *sensor)
{
	struct ltc2983_rsense *rsense;
	int ret;

	rsense = devm_kzalloc(&st->spi->dev, sizeof(*rsense), GFP_KERNEL);
	if (!rsense)
		return ERR_PTR(-ENOMEM);

	/* validate channel index */
	if (sensor->chan < LTC2983_DIFFERENTIAL_CHAN_MIN) {
		dev_err(&st->spi->dev, "Invalid chann:%d for r_sense",
							sensor->chan);
		return ERR_PTR(-EINVAL);
	}
	/* get raw value */
	ret = of_property_read_u32(child, "adi,rsense-val",
				   &rsense->r_sense_val);
	if (ret) {
		dev_err(&st->spi->dev, "Property adi,rsense-val missing\n");
		return ERR_PTR(-EINVAL);
	}

	/* set common parameters */
	rsense->sensor.name = "r_sense";
	rsense->sensor.assign_chan = ltc2983_r_sense_assign_chan;

	return &rsense->sensor;
}

static struct ltc2983_sensor *ltc2983_adc_new(struct device_node *child,
					 struct ltc2983_data *st,
					 const struct ltc2983_sensor *sensor)
{
	struct ltc2983_adc *adc;

	adc = devm_kzalloc(&st->spi->dev, sizeof(*adc), GFP_KERNEL);
	if (!adc)
		return ERR_PTR(-ENOMEM);

	if (of_property_read_bool(child, "adi,single-ended"))
		adc->single_ended = true;

	if (!adc->single_ended &&
	    sensor->chan < LTC2983_DIFFERENTIAL_CHAN_MIN) {
		dev_err(&st->spi->dev, "Invalid chan:%d for differential adc\n",
								sensor->chan);
		return ERR_PTR(-EINVAL);
	}
	/* set common parameters */
	adc->sensor.name = "direct_adc";
	adc->sensor.assign_chan = ltc2983_adc_assign_chan;
	adc->sensor.fault_handler = ltc2983_common_fault_handler;

	return &adc->sensor;
}

static int ltc2983_chan_read(struct ltc2983_data *st,
			const struct ltc2983_sensor *sensor, int *val)
{
	u32 start_conversion = 0;
	int ret;
	unsigned long time;
	__be32 temp;

	/*
	 * Do not allow channel readings if device is in sleep state.
	 * A read/write on the spi bus would bring the device prematurely
	 * out of sleep.
	 */
	if (st->reset)
		return -EPERM;

	start_conversion = LTC2983_STATUS_START(true);
	start_conversion |= LTC2983_STATUS_CHAN_SEL(sensor->chan);
	dev_dbg(&st->spi->dev, "Start conversion on chan:%d, status:%02X\n",
		sensor->chan, start_conversion);
	/* start conversion */
	ret = regmap_write(st->regmap, LTC2983_STATUS_REG, start_conversion);
	if (ret)
		return ret;

	reinit_completion(&st->completion);
	/*
	 * wait for conversion to complete.
	 * 300 ms should be more than enough to complete the conversion.
	 * Depending on the sensor configuration, there are 2/3 conversions
	 * cycles of 82ms.
	 */
	time = wait_for_completion_timeout(&st->completion,
					   msecs_to_jiffies(300));
	if (!time) {
		dev_warn(&st->spi->dev, "Conversion timed out\n");
		return -ETIMEDOUT;
	}

	/* read the converted data */
	ret = regmap_bulk_read(st->regmap, LTC2983_CHAN_RES_ADDR(sensor->chan),
			       &temp, sizeof(temp));
	if (ret)
		return ret;

	*val = __be32_to_cpu(temp);
	dev_dbg(&st->spi->dev, "Converted data: 0x%08X (addr:0x%04X)\n", *val,
					LTC2983_CHAN_RES_ADDR(sensor->chan));

	if (!(LTC2983_RES_VALID_MASK & *val)) {
		dev_err(&st->spi->dev, "Invalid conversion detected\n");
		return -EIO;
	}

	ret = sensor->fault_handler(st, *val);
	if (ret)
		return ret;

	*val = sign_extend32((*val) & LTC2983_DATA_MASK, LTC2983_DATA_SIGN_BIT);
	return 0;
}

static int ltc2983_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct ltc2983_data *st = iio_priv(indio_dev);
	int ret;

	/* sanity check */
	if (chan->address >= st->num_channels) {
		dev_err(&st->spi->dev, "Invalid chan address:%ld",
							chan->address);
		return -EINVAL;
	}
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&st->lock);
		ret = ltc2983_chan_read(st, st->sensors[chan->address], val);
		mutex_unlock(&st->lock);
		return ret ?: IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_TEMP:
			*val = 1;
			/* 2^10 */
			*val2 = 1024;
			return IIO_VAL_FRACTIONAL;
		case IIO_VOLTAGE:
			*val = 1;
			/* 2^21 */
			*val2 = 2097152;
			return IIO_VAL_FRACTIONAL;
		default:
			return -EINVAL;
		}
	}

	return -EINVAL;
}

static int ltc2983_reg_access(struct iio_dev *indio_dev,
			      unsigned int reg,
			      unsigned int writeval,
			      unsigned int *readval)
{
	struct ltc2983_data *st = iio_priv(indio_dev);

	/* check comment in ltc2983_chan_read() */
	if (st->reset)
		return -EPERM;

	if (readval)
		return regmap_read(st->regmap, reg, readval);
	else
		return regmap_write(st->regmap, reg, writeval);
}

static irqreturn_t ltc2983_irq_handler(int irq, void *data)
{
	struct ltc2983_data *st = data;

	complete(&st->completion);
	return IRQ_HANDLED;
}

#define LTC2983_CHAN(__type, index, __address, ext_name) ({ \
	struct iio_chan_spec __chan = { \
		.type = __type, \
		.indexed = 1, \
		.channel = index, \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), \
		.address = __address, \
		.extend_name = ext_name, \
	}; \
	__chan; \
})

static int ltc2983_parse_dt(struct ltc2983_data *st)
{
	struct device_node *child;
	struct device *dev = &st->spi->dev;
	int ret = 0, chan = 0, channel_avail_mask = 0;

	if (!of_property_read_bool(dev->of_node, "adi,temperature-celcius"))
		st->temp_farenheit = true;

	of_property_read_u32(dev->of_node, "adi,mux-delay-config",
			     &st->mux_delay_config);

	of_property_read_u32(dev->of_node, "adi,filter-notch-freq",
			     &st->filter_notch_freq);

	st->num_channels = of_get_available_child_count(dev->of_node);
	st->sensors = devm_kcalloc(dev, st->num_channels, sizeof(*st->sensors),
				   GFP_KERNEL);
	if (!st->sensors)
		return -ENOMEM;

	st->iio_channels = st->num_channels;
	for_each_available_child_of_node(dev->of_node, child) {
		struct ltc2983_sensor sensor;

		ret = of_property_read_u32(child, "reg", &sensor.chan);
		if (ret) {
			dev_err(dev, "reg property must given for child nodes\n");
			return ret;
		}

		/* check if we have a valid channel */
		if (sensor.chan < LTC2983_MIN_CHANNELS_NR ||
		    sensor.chan > LTC2983_MAX_CHANNELS_NR) {
			dev_err(dev,
				"chan:%d must be from 1 to 20\n", sensor.chan);
			return -EINVAL;
		} else if (channel_avail_mask & BIT(sensor.chan)) {
			dev_err(dev, "chan:%d already in use\n", sensor.chan);
			return -EINVAL;
		}

		ret = of_property_read_u32(child, "adi,sensor-type",
					       &sensor.type);
		if (ret) {
			dev_err(dev,
				"adi,sensor-type property must given for child nodes\n");
			return ret;
		}

		dev_dbg(dev, "Create new sensor, type %u, chann %u",
								sensor.type,
								sensor.chan);

		if (sensor.type >= LTC2983_SENSOR_THERMOCOUPLE &&
		    sensor.type <= LTC2983_SENSOR_THERMOCOUPLE_CUSTOM) {
			st->sensors[chan] = ltc2983_thermocouple_new(child, st,
								     &sensor);
		} else if (sensor.type >= LTC2983_SENSOR_RTD &&
			   sensor.type <= LTC2983_SENSOR_RTD_CUSTOM) {
			st->sensors[chan] = ltc2983_rtd_new(child, st, &sensor);
		} else if (sensor.type >= LTC2983_SENSOR_THERMISTOR &&
			   sensor.type <= LTC2983_SENSOR_THERMISTOR_CUSTOM) {
			st->sensors[chan] = ltc2983_thermistor_new(child, st,
								   &sensor);
		} else if (sensor.type == LTC2983_SENSOR_DIODE) {
			st->sensors[chan] = ltc2983_diode_new(child, st,
							      &sensor);
		} else if (sensor.type == LTC2983_SENSOR_SENSE_RESISTOR) {
			st->sensors[chan] = ltc2983_r_sense_new(child, st,
								&sensor);
			/* don't add rsense to iio */
			st->iio_channels--;
		} else if (sensor.type == LTC2983_SENSOR_DIRECT_ADC) {
			st->sensors[chan] = ltc2983_adc_new(child, st, &sensor);
		} else {
			dev_err(dev, "Unknown sensor type %d\n", sensor.type);
			return -EINVAL;
		}

		if (IS_ERR(st->sensors[chan])) {
			dev_err(dev, "Failed to create sensor %ld",
						PTR_ERR(st->sensors[chan]));
			return PTR_ERR(st->sensors[chan]);
		}
		/* set generic sensor parameters */
		st->sensors[chan]->chan = sensor.chan;
		st->sensors[chan]->type = sensor.type;

		channel_avail_mask |= BIT(sensor.chan);
		chan++;
	}
	/* allocate iio_chan array */
	st->iio_chan = devm_kzalloc(dev,
				    st->iio_channels * sizeof(*st->iio_chan),
				    GFP_KERNEL);
	if (!st->iio_chan)
		return -ENOMEM;

	return 0;
}

static int ltc2983_setup(struct ltc2983_data *st, bool assign_iio)
{
	u32 iio_chan_t = 0, iio_chan_v = 0, chan, iio_idx = 0;
	u32 global_cfg = 0;
	int ret;
	unsigned long time;

	/* make sure the device is up */
	time = wait_for_completion_timeout(&st->completion,
					    msecs_to_jiffies(250));
	if (!time) {
		dev_err(&st->spi->dev, "Device startup timed out\n");
		return -ETIMEDOUT;
	}
	/* set generic global cfg and mux delay */
	global_cfg = LTC2983_NOTCH_FREQ(st->filter_notch_freq);
	global_cfg |= LTC2983_TEMP_UNITS(st->temp_farenheit);
	regmap_write(st->regmap, LTC2983_GLOBAL_CONFIG_REG, global_cfg);
	regmap_write(st->regmap, LTC2983_MUX_CONFIG_REG, st->mux_delay_config);
	/* setup channels */
	for (chan = 0; chan < st->num_channels; chan++) {
		u32 chan_type = 0, *iio_chan;

		ret = st->sensors[chan]->assign_chan(st, st->sensors[chan]);
		if (ret)
			return ret;
		/*
		 * The assign_iio flag is necessary for when the device is
		 * coming out of sleep. In that case, we just need to
		 * re-configure the device channels.
		 * We also don't assign iio channels for rsense.
		 */
		if (st->sensors[chan]->type == LTC2983_SENSOR_SENSE_RESISTOR ||
		    !assign_iio)
			continue;

		/* assign iio channel */
		if (st->sensors[chan]->type != LTC2983_SENSOR_DIRECT_ADC) {
			chan_type = IIO_TEMP;
			iio_chan = &iio_chan_t;
		} else {
			chan_type = IIO_VOLTAGE;
			iio_chan = &iio_chan_v;
		}

		/*
		 * add chan as the iio .address so that, we can directly
		 * reference the sensor given the iio_chan_spec
		 */
		st->iio_chan[iio_idx++] = LTC2983_CHAN(chan_type, (*iio_chan)++,
						       chan,
						       st->sensors[chan]->name);
	}

	return 0;
}

static const struct regmap_range ltc2983_reg_ranges[] = {
	regmap_reg_range(LTC2983_STATUS_REG, LTC2983_STATUS_REG),
	regmap_reg_range(LTC2983_TEMP_RES_START_REG, LTC2983_TEMP_RES_END_REG),
	regmap_reg_range(LTC2983_GLOBAL_CONFIG_REG, LTC2983_GLOBAL_CONFIG_REG),
	regmap_reg_range(LTC2983_MULT_CHANNEL_START_REG,
			 LTC2983_MULT_CHANNEL_END_REG),
	regmap_reg_range(LTC2983_MUX_CONFIG_REG, LTC2983_MUX_CONFIG_REG),
	regmap_reg_range(LTC2983_CHAN_ASSIGN_START_REG,
			 LTC2983_CHAN_ASSIGN_END_REG),
	regmap_reg_range(LTC2983_CUST_SENS_TBL_START_REG,
			 LTC2983_CUST_SENS_TBL_END_REG),
};

static const struct regmap_access_table ltc2983_reg_table = {
	.yes_ranges = ltc2983_reg_ranges,
	.n_yes_ranges = ARRAY_SIZE(ltc2983_reg_ranges),
};
/*
 *  The reg_bits are actually 12 but the device needs the first *complete*
 *  byte for the command (R/W).
 */
static const struct regmap_config ltc2983_regmap_config = {
	.reg_bits = 24,
	.val_bits = 8,
	.wr_table = &ltc2983_reg_table,
	.rd_table = &ltc2983_reg_table,
	.read_flag_mask = GENMASK(1, 0),
	.write_flag_mask = BIT(1),
};

static const struct  iio_info ltc2983_iio_info = {
	.read_raw = ltc2983_read_raw,
	.debugfs_reg_access = ltc2983_reg_access,
};

static int ltc2983_probe(struct spi_device *spi)
{
	struct ltc2983_data *st;
	struct iio_dev *indio_dev;
	const char *name = spi_get_device_id(spi)->name;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	st->regmap = devm_regmap_init_spi(spi, &ltc2983_regmap_config);
	if (IS_ERR(st->regmap)) {
		dev_err(&spi->dev, "Failed to initialize regmap\n");
		return PTR_ERR(st->regmap);
	}

	mutex_init(&st->lock);
	init_completion(&st->completion);
	st->spi = spi;
	spi_set_drvdata(spi, st);

	ret = ltc2983_parse_dt(st);
	if (ret)
		return ret;
	/*
	 * let's request the irq now so it is used to sync the device
	 * startup in ltc2983_setup()
	 */
	ret = devm_request_threaded_irq(&spi->dev, spi->irq, NULL,
					ltc2983_irq_handler, IRQF_ONESHOT |
					IRQF_TRIGGER_RISING, name, st);
	if (ret) {
		dev_err(&spi->dev, "failed to request an irq, %d", ret);
		return ret;
	}

	ret = ltc2983_setup(st, true);
	if (ret)
		return ret;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = name;
	indio_dev->num_channels = st->iio_channels;
	indio_dev->channels = st->iio_chan;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ltc2983_iio_info;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static int __maybe_unused ltc2983_resume(struct device *dev)
{
	struct ltc2983_data *st = spi_get_drvdata(to_spi_device(dev));
	int ret;

	mutex_lock(&st->lock);
	/* dummy read to bring the device out of sleep */
	regmap_read(st->regmap, LTC2983_STATUS_REG, &ret);
	/* we need to re-assign the channels */
	ret = ltc2983_setup(st, false);
	st->reset = false;
	mutex_unlock(&st->lock);

	return ret;
}

static int __maybe_unused ltc2983_suspend(struct device *dev)
{
	struct ltc2983_data *st = spi_get_drvdata(to_spi_device(dev));
	int ret;

	mutex_lock(&st->lock);
	ret = regmap_write(st->regmap, LTC2983_STATUS_REG, LTC2983_SLEEP);
	st->reset = true;
	mutex_unlock(&st->lock);

	return ret;
}

static SIMPLE_DEV_PM_OPS(ltc2983_pm_ops, ltc2983_suspend, ltc2983_resume);

static const struct spi_device_id ltc2983_id_table[] = {
	{ "ltc2983" },
	{},
};
MODULE_DEVICE_TABLE(spi, ltc2983_id_table);

static const struct of_device_id ltc2983_of_match[] = {
	{ .compatible = "adi,ltc2983" },
	{},
};
MODULE_DEVICE_TABLE(of, ltc2983_id_table);

static struct spi_driver ltc2983_driver = {
	.driver = {
		.name = "ltc2983",
		.of_match_table = ltc2983_of_match,
		.pm = &ltc2983_pm_ops,
	},
	.probe = ltc2983_probe,
	.id_table = ltc2983_id_table,
};

module_spi_driver(ltc2983_driver);

MODULE_AUTHOR("Nuno Sa <nuno.sa@analog.com>");
MODULE_DESCRIPTION("Analog Devices LTC2983 SPI Temperature sensors");
MODULE_LICENSE("GPL");
