# Linux IIO Temperature Sensor Driver Template

This template covers IIO temperature sensor drivers under `drivers/iio/temperature/`.
It builds on the consolidated IIO driver template and adds patterns specific to
temperature measurement: single-channel digital sensors, thermocouple front-ends
with cold-junction compensation, RTD-to-digital converters, and multi-sensor
measurement systems like the LTC2983.

---

## 1. Purpose & Subsystem Mapping

Temperature sensors use the `IIO_TEMP` channel type. All temperature values
reported through IIO must be in **millidegrees Celsius**.

| Aspect                | Value / Convention                                           |
|-----------------------|--------------------------------------------------------------|
| IIO channel type      | `IIO_TEMP`                                                   |
| Subdirectory          | `drivers/iio/temperature/`                                   |
| Unit                  | millidegrees Celsius (raw * scale + offset = millidegrees C) |
| Typical `info_mask`   | `RAW`, `SCALE`, `OFFSET`, `PROCESSED`                        |

### info_mask Bits for Temperature Channels

| Bit                             | When to Use                                                                  |
|---------------------------------|------------------------------------------------------------------------------|
| `IIO_CHAN_INFO_RAW`             | Always. Returns the raw register reading from the sensor.                    |
| `IIO_CHAN_INFO_SCALE`           | Always alongside RAW. Converts raw to millidegrees C: `temp_mC = raw * scale`. Typically returned as `IIO_VAL_INT_PLUS_MICRO` (e.g. `*val = 7, *val2 = 812500` for 7.8125 mC/LSB). |
| `IIO_CHAN_INFO_OFFSET`          | When the raw value needs an additive correction before scaling (e.g. bipolar sensors, Kelvin-to-Celsius conversion). |
| `IIO_CHAN_INFO_PROCESSED`       | When the hardware provides a direct temperature reading that requires no userspace math. Return the value in millidegrees C as `IIO_VAL_INT`. Use PROCESSED **instead of** RAW+SCALE, not alongside them. |
| `IIO_CHAN_INFO_CALIBBIAS`       | When the sensor has a programmable offset calibration register (e.g. TMP117). |
| `IIO_CHAN_INFO_OVERSAMPLING_RATIO` | When the sensor supports hardware averaging / oversampling.               |
| `IIO_CHAN_INFO_THERMOCOUPLE_TYPE`  | For thermocouple sensors that support runtime type selection (B/E/J/K/N/R/S/T). Returns `IIO_VAL_CHAR`. |

### Choosing Between RAW+SCALE and PROCESSED

- Use **RAW + SCALE** when the sensor provides a register value that must be
  multiplied by a constant to get millidegrees C. This is the common case and
  lets userspace tools (e.g. `iio_readdev`) apply the conversion.
- Use **PROCESSED** when the sensor hardware outputs a value that is already
  in millidegrees C (or trivially derived). In this case do **not** also
  expose RAW and SCALE; pick one approach or the other.

---

## 2. File Checklist

| File                                                                  | Action   | Required |
|-----------------------------------------------------------------------|----------|----------|
| `drivers/iio/temperature/<devname>.c`                                 | Create   | Yes      |
| `drivers/iio/temperature/Kconfig`                                     | Modify   | Yes      |
| `drivers/iio/temperature/Makefile`                                    | Modify   | Yes      |
| `Documentation/devicetree/bindings/iio/temperature/adi,<devname>.yaml`| Create   | Yes      |
| `include/dt-bindings/iio/temperature/thermocouple.h`                  | Modify   | If thermocouple type constants are needed |

---

## 3. Devicetree Binding

### Simple I2C Temperature Sensor

```yaml
# SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)
%YAML 1.2
---
$id: http://devicetree.org/schemas/iio/temperature/adi,adt7420.yaml#
$schema: http://devicetree.org/meta-schemas/core.yaml#

title: Analog Devices ADT7420 Digital Temperature Sensor

maintainers:
  - First Last <first.last@analog.com>

description: |
  The ADT7420 is a high accuracy digital temperature sensor with a
  16-bit ADC and I2C interface. Accuracy is +/-0.25 degC from -40 to
  +125 degC.

properties:
  compatible:
    enum:
      - adi,adt7410
      - adi,adt7420

  reg:
    maxItems: 1

  interrupts:
    maxItems: 1
    description: Active-low overtemperature interrupt.

  vdd-supply: true

required:
  - compatible
  - reg

additionalProperties: false

examples:
  - |
    i2c {
        #address-cells = <1>;
        #size-cells = <0>;

        temperature-sensor@48 {
            compatible = "adi,adt7420";
            reg = <0x48>;
            interrupts = <5 2>;
            interrupt-parent = <&gpio>;
            vdd-supply = <&vdd_3v3>;
        };
    };
```

### SPI Thermocouple Sensor

For thermocouple and RTD sensors, add properties for the sensor element type
and any excitation or wiring configuration:

```yaml
# SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)
%YAML 1.2
---
$id: http://devicetree.org/schemas/iio/temperature/adi,adxxxx-tc.yaml#
$schema: http://devicetree.org/meta-schemas/core.yaml#

title: Analog Devices ADXXXX Thermocouple-to-Digital Converter

maintainers:
  - First Last <first.last@analog.com>

description: |
  The ADXXXX is a thermocouple-to-digital converter with integrated
  cold-junction compensation and SPI interface.

properties:
  compatible:
    const: adi,adxxxx-tc

  reg:
    maxItems: 1

  spi-max-frequency:
    maximum: 5000000

  interrupts:
    maxItems: 1

  thermocouple-type:
    $ref: /schemas/types.yaml#/definitions/uint32
    description: |
      Thermocouple type. Use constants from
      include/dt-bindings/iio/temperature/thermocouple.h.
      Supported values: 0 (B), 1 (E), 2 (J), 3 (K),
      4 (N), 5 (R), 6 (S), 7 (T).
    default: 3

  vdd-supply: true

required:
  - compatible
  - reg

additionalProperties: false

examples:
  - |
    #include <dt-bindings/iio/temperature/thermocouple.h>

    spi {
        #address-cells = <1>;
        #size-cells = <0>;

        temperature-sensor@0 {
            compatible = "adi,adxxxx-tc";
            reg = <0>;
            spi-max-frequency = <5000000>;
            thermocouple-type = <THERMOCOUPLE_TYPE_K>;
        };
    };
```

### Multi-Sensor System (LTC2983 Pattern)

For devices like the LTC2983 that support multiple sensor types
(thermocouples, RTDs, thermistors, diodes) on different channels, use
child nodes to describe each sensor:

```yaml
properties:
  '#address-cells':
    const: 1

  '#size-cells':
    const: 0

patternProperties:
  "^thermocouple@[0-9a-f]+$":
    type: object
    additionalProperties: false
    description: Thermocouple sensor on a given channel.

    properties:
      reg:
        minimum: 1
        maximum: 20
        description: Channel number.

      adi,sensor-type:
        $ref: /schemas/types.yaml#/definitions/uint32
        description: Sensor type enumeration.

      adi,cold-junction-handle:
        $ref: /schemas/types.yaml#/definitions/phandle
        description: Phandle to the cold-junction sensor channel.

      adi,sensor-config:
        $ref: /schemas/types.yaml#/definitions/uint32
        description: Sensor-specific configuration bits.

    required:
      - reg
      - adi,sensor-type

  "^rtd@[0-9a-f]+$":
    type: object
    additionalProperties: false
    description: RTD sensor on a given channel.

    properties:
      reg:
        minimum: 1
        maximum: 20

      adi,sensor-type:
        $ref: /schemas/types.yaml#/definitions/uint32

      adi,rsense-handle:
        $ref: /schemas/types.yaml#/definitions/phandle
        description: Phandle to the sense resistor channel.

      adi,excitation-current-microamp:
        $ref: /schemas/types.yaml#/definitions/uint32
        description: Excitation current for RTD measurement.

      adi,number-of-wires:
        $ref: /schemas/types.yaml#/definitions/uint32
        enum: [2, 3, 4]
        description: RTD wiring configuration.

      adi,rtd-curve:
        $ref: /schemas/types.yaml#/definitions/uint32
        description: RTD curve type.

    required:
      - reg
      - adi,sensor-type

  "^thermistor@[0-9a-f]+$":
    type: object
    additionalProperties: false
    description: Thermistor sensor on a given channel.

    properties:
      reg:
        minimum: 1
        maximum: 20

      adi,sensor-type:
        $ref: /schemas/types.yaml#/definitions/uint32

      adi,rsense-handle:
        $ref: /schemas/types.yaml#/definitions/phandle

      adi,excitation-current-microamp:
        $ref: /schemas/types.yaml#/definitions/uint32

    required:
      - reg
      - adi,sensor-type
```

---

## 4. Kconfig

Add the entry to `drivers/iio/temperature/Kconfig` in alphabetical order.

### Simple I2C Temperature Sensor

```kconfig
config ADT7420
	tristate "Analog Devices ADT7420 temperature sensor driver"
	depends on I2C
	select REGMAP_I2C
	help
	  Say yes here to build support for Analog Devices ADT7420 and
	  ADT7410 high accuracy digital temperature sensors.

	  To compile this driver as a module, choose M here: the module will be
	  called adt7420.
```

### SPI Thermocouple / RTD Front-End

```kconfig
config ADXXXX_TC
	tristate "Analog Devices ADXXXX thermocouple converter driver"
	depends on SPI
	help
	  Say yes here to build support for the Analog Devices ADXXXX
	  thermocouple-to-digital converter.

	  To compile this driver as a module, choose M here: the module will be
	  called adxxxx-tc.
```

### Multi-Sensor System (with regmap)

```kconfig
config LTC2983
	tristate "Analog Devices Multi-Sensor Digital Temperature Measurement System"
	depends on SPI
	select REGMAP_SPI
	help
	  Say yes here to build support for the LTC2983 Multi-Sensor
	  high accuracy digital temperature measurement system.

	  To compile this driver as a module, choose M here: the module
	  will be called ltc2983.
```

---

## 5. Makefile

Add the entry to `drivers/iio/temperature/Makefile` in alphabetical order:

```makefile
obj-$(CONFIG_ADT7420) += adt7420.o
obj-$(CONFIG_LTC2983) += ltc2983.o
```

---

## 6. Driver Source

### Single IIO_TEMP Channel (I2C Digital Sensor)

This is the simplest temperature driver pattern. The sensor has a single
temperature output register and a fixed resolution (scale).

```c
// SPDX-License-Identifier: GPL-2.0+
/*
 * Analog Devices ADXXXX Digital Temperature Sensor driver
 *
 * Copyright 2024 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>

#include <linux/iio/iio.h>

/* ------------------------------------------------------------------ */
/* Register Definitions                                                */
/* ------------------------------------------------------------------ */

#define ADXXXX_REG_TEMP		0x00
#define ADXXXX_REG_CONFIG	0x01
#define ADXXXX_REG_ID		0x0F

#define ADXXXX_DEVICE_ID	0x0XXX

/*
 * Resolution: each LSB = 7.8125 millidegrees C
 * Express as IIO_VAL_INT_PLUS_MICRO: 7 + 812500/1000000
 */
#define ADXXXX_SCALE_INT	7
#define ADXXXX_SCALE_MICRO	812500

/* ------------------------------------------------------------------ */
/* Device State                                                        */
/* ------------------------------------------------------------------ */

struct adxxxx_data {
	struct i2c_client *client;
};

/* ------------------------------------------------------------------ */
/* IIO Channel Specification                                           */
/* ------------------------------------------------------------------ */

/*
 * A single, non-indexed IIO_TEMP channel.
 * For a single-sensor device the channel does not need .indexed = 1
 * or .channel = N; the sysfs entry will be in_temp_raw / in_temp_scale.
 */
static const struct iio_chan_spec adxxxx_channels[] = {
	{
		.type = IIO_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
	},
};

/* ------------------------------------------------------------------ */
/* read_raw Callback                                                   */
/* ------------------------------------------------------------------ */

static int adxxxx_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	struct adxxxx_data *data = iio_priv(indio_dev);
	s32 ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = i2c_smbus_read_word_swapped(data->client,
						  ADXXXX_REG_TEMP);
		if (ret < 0)
			return ret;

		*val = sign_extend32(ret, 15);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		/*
		 * Convert raw to millidegrees C.
		 * temp_mC = raw * 7.8125
		 * Expressed as IIO_VAL_INT_PLUS_MICRO: 7 + 812500/1e6
		 */
		*val = ADXXXX_SCALE_INT;
		*val2 = ADXXXX_SCALE_MICRO;
		return IIO_VAL_INT_PLUS_MICRO;

	default:
		return -EINVAL;
	}
}

static const struct iio_info adxxxx_info = {
	.read_raw = adxxxx_read_raw,
};

/* ------------------------------------------------------------------ */
/* Probe                                                               */
/* ------------------------------------------------------------------ */

static int adxxxx_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct adxxxx_data *data;
	struct iio_dev *indio_dev;
	int ret;

	ret = devm_regulator_get_enable(dev, "vdd");
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to enable vdd supply\n");

	indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	data->client = client;

	indio_dev->name = "adxxxx";
	indio_dev->info = &adxxxx_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = adxxxx_channels;
	indio_dev->num_channels = ARRAY_SIZE(adxxxx_channels);

	return devm_iio_device_register(dev, indio_dev);
}

/* ------------------------------------------------------------------ */
/* Match Tables                                                        */
/* ------------------------------------------------------------------ */

static const struct of_device_id adxxxx_of_match[] = {
	{ .compatible = "adi,adxxxx" },
	{ }
};
MODULE_DEVICE_TABLE(of, adxxxx_of_match);

static const struct i2c_device_id adxxxx_ids[] = {
	{ "adxxxx" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, adxxxx_ids);

static struct i2c_driver adxxxx_driver = {
	.driver = {
		.name = "adxxxx",
		.of_match_table = adxxxx_of_match,
	},
	.probe = adxxxx_probe,
	.id_table = adxxxx_ids,
};
module_i2c_driver(adxxxx_driver);

MODULE_AUTHOR("First Last <first.last@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADXXXX temperature sensor driver");
MODULE_LICENSE("GPL");
```

### Scale/Offset Conversion to Millidegrees C

The fundamental equation for userspace is:

```
temperature_mC = (raw + offset) * scale
```

Common patterns for returning `scale`:

```c
case IIO_CHAN_INFO_SCALE:
	/*
	 * Pattern 1: Fixed-point resolution.
	 * Example: 7.8125 mC per LSB (TMP117-like)
	 *   IIO_VAL_INT_PLUS_MICRO: val=7, val2=812500
	 */
	*val = 7;
	*val2 = 812500;
	return IIO_VAL_INT_PLUS_MICRO;

	/*
	 * Pattern 2: Fractional with power-of-2 denominator.
	 * Example: 1000/1024 mC per LSB (LTC2983)
	 *   IIO_VAL_FRACTIONAL: val=1000, val2=1024
	 */
	*val = 1000;
	*val2 = 1024;
	return IIO_VAL_FRACTIONAL;

	/*
	 * Pattern 3: Simple integer scale (uncommon).
	 * Example: 62.5 mC per LSB
	 *   IIO_VAL_INT_PLUS_MICRO: val=62, val2=500000
	 */
	*val = 62;
	*val2 = 500000;
	return IIO_VAL_INT_PLUS_MICRO;
```

Common patterns for returning `offset` (when raw is not zero-referenced to Celsius):

```c
case IIO_CHAN_INFO_OFFSET:
	/*
	 * Example: Sensor reads in Kelvin, offset to convert to Celsius.
	 * offset = -273150 (in millidegrees, already scaled)
	 * or as a raw offset if applied before scaling.
	 */
	*val = -273;
	*val2 = 150000;
	return IIO_VAL_INT_PLUS_MICRO;
```

### PROCESSED: Direct Temperature Reading

When the sensor hardware provides temperature directly and no conversion is
needed, use `IIO_CHAN_INFO_PROCESSED` instead of RAW+SCALE:

```c
static const struct iio_chan_spec adxxxx_channels[] = {
	{
		.type = IIO_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
	},
};

static int adxxxx_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_PROCESSED:
		/* Read hardware register that directly gives temperature */
		ret = adxxxx_read_temp_reg(data, &raw_temp);
		if (ret)
			return ret;

		/*
		 * Convert to millidegrees C in the driver.
		 * Example: register value is in 1/16 degree C.
		 */
		*val = (raw_temp * 1000) / 16;
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}
```

### Thermocouple + Cold Junction (Dual Channel)

Thermocouple sensors typically expose two temperature channels: the
thermocouple (hot junction) measurement and the cold-junction (ambient)
compensation temperature. Use the `IIO_MOD_TEMP_AMBIENT` modifier on the
cold-junction channel:

```c
static const struct iio_chan_spec adxxxx_tc_channels[] = {
	{	/* Thermocouple Temperature (hot junction) */
		.type = IIO_TEMP,
		.info_mask_separate =
			BIT(IIO_CHAN_INFO_RAW) |
			BIT(IIO_CHAN_INFO_SCALE) |
			BIT(IIO_CHAN_INFO_THERMOCOUPLE_TYPE),
		.info_mask_shared_by_type =
			BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),
	},
	{	/* Cold Junction Temperature (ambient) */
		.type = IIO_TEMP,
		.channel2 = IIO_MOD_TEMP_AMBIENT,
		.modified = 1,
		.info_mask_separate =
			BIT(IIO_CHAN_INFO_RAW) |
			BIT(IIO_CHAN_INFO_SCALE),
		.info_mask_shared_by_type =
			BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),
	},
};
```

In the `read_raw` callback, differentiate between channels using `chan->channel2`:

```c
case IIO_CHAN_INFO_RAW:
	ret = adxxxx_thermocouple_read(data, chan, val);
	if (ret)
		return ret;
	return IIO_VAL_INT;

case IIO_CHAN_INFO_SCALE:
	switch (chan->channel2) {
	case IIO_MOD_TEMP_AMBIENT:
		/* Cold junction: 0.015625 degC per LSB = 15.625 mC */
		*val = 15;
		*val2 = 625000;
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		/* Thermocouple: 0.0078125 degC per LSB = 7.8125 mC */
		*val = 7;
		*val2 = 812500;
		return IIO_VAL_INT_PLUS_MICRO;
	}

case IIO_CHAN_INFO_THERMOCOUPLE_TYPE:
	*val = tc_type_chars[data->thermocouple_type];
	return IIO_VAL_CHAR;
```

The `IIO_CHAN_INFO_THERMOCOUPLE_TYPE` attribute uses `IIO_VAL_CHAR` and returns
a single character (B, E, J, K, N, R, S, T). The `write_raw_get_fmt` callback
must also return `IIO_VAL_CHAR` for this mask:

```c
static int adxxxx_write_raw_get_fmt(struct iio_dev *indio_dev,
				    struct iio_chan_spec const *chan,
				    long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_THERMOCOUPLE_TYPE:
		return IIO_VAL_CHAR;
	default:
		return IIO_VAL_INT;
	}
}
```

---

## 7. Multi-Sensor Support

For devices like the LTC2983 that accept multiple sensor types on
configurable input channels, the driver architecture uses:

1. **A base sensor struct** with function pointers for channel assignment
   and fault handling, plus type-specific sub-structs accessed via
   `container_of()`.
2. **Dynamic IIO channel allocation** based on devicetree child nodes.
3. **An indexed channel** per physical sensor input.

### Sensor Hierarchy

```c
struct ltc2983_sensor {
	int (*fault_handler)(const struct ltc2983_data *st, const u32 result);
	int (*assign_chan)(struct ltc2983_data *st,
			  const struct ltc2983_sensor *sensor);
	u32 chan;     /* physical channel number */
	u32 type;    /* sensor type enum */
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

#define to_thermocouple(_sensor) \
		container_of(_sensor, struct ltc2983_thermocouple, sensor)
#define to_rtd(_sensor) \
		container_of(_sensor, struct ltc2983_rtd, sensor)
#define to_thermistor(_sensor) \
		container_of(_sensor, struct ltc2983_thermistor, sensor)
```

### Dynamic Channel Construction

```c
#define LTC2983_CHAN(__type, index, __address) ({ \
	struct iio_chan_spec __chan = { \
		.type = __type, \
		.indexed = 1, \
		.channel = index, \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), \
		.address = __address, \
	}; \
	__chan; \
})
```

Channels are allocated in `probe()` based on the number of child nodes,
and each sensor's physical channel number is stored in `.address` so the
`read_raw` callback can look up the correct sensor:

```c
st->iio_chan = devm_kcalloc(dev, st->iio_channels,
			    sizeof(*st->iio_chan), GFP_KERNEL);

/* For each sensor that is not a sense resistor: */
st->iio_chan[iio_idx++] = LTC2983_CHAN(chan_type, (*iio_chan)++, chan);
```

### read_raw for Multi-Sensor

The `read_raw` callback uses `chan->address` to index into the sensor array.
All sensor types share the same scale factor (the device does the conversion
internally):

```c
static int ltc2983_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct ltc2983_data *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&st->lock);
		ret = ltc2983_chan_read(st, st->sensors[chan->address], val);
		mutex_unlock(&st->lock);
		return ret ?: IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		/* value in millidegrees; raw is Q-format 2^10 */
		*val = 1000;
		*val2 = 1024;
		return IIO_VAL_FRACTIONAL;
	}

	return -EINVAL;
}
```

---

## 8. Devicetree Parsing

### Simple Sensor (Thermocouple Type from DT)

```c
static int adxxxx_probe(struct spi_device *spi)
{
	struct adxxxx_data *data;
	int ret;

	/* ... alloc, init ... */

	ret = device_property_read_u32(&spi->dev, "thermocouple-type",
				       &data->thermocouple_type);
	if (ret) {
		dev_info(&spi->dev,
			 "No thermocouple type specified, defaulting to K\n");
		data->thermocouple_type = THERMOCOUPLE_TYPE_K;
	}

	/* Validate the type */
	switch (data->thermocouple_type) {
	case THERMOCOUPLE_TYPE_B:
	case THERMOCOUPLE_TYPE_E:
	case THERMOCOUPLE_TYPE_J:
	case THERMOCOUPLE_TYPE_K:
	case THERMOCOUPLE_TYPE_N:
	case THERMOCOUPLE_TYPE_R:
	case THERMOCOUPLE_TYPE_S:
	case THERMOCOUPLE_TYPE_T:
		break;
	default:
		return dev_err_probe(&spi->dev, -EINVAL,
				     "Unsupported thermocouple type %u\n",
				     data->thermocouple_type);
	}

	/* ... */
}
```

### Multi-Sensor Child Node Parsing (LTC2983 Pattern)

```c
static int ltc2983_parse_fw(struct ltc2983_data *st)
{
	struct device *dev = &st->spi->dev;
	int ret, chan = 0;

	device_property_read_u32(dev, "adi,mux-delay-config-us",
				 &st->mux_delay_config);
	device_property_read_u32(dev, "adi,filter-notch-freq",
				 &st->filter_notch_freq);

	st->num_channels = device_get_child_node_count(dev);
	if (!st->num_channels)
		return dev_err_probe(dev, -EINVAL,
				     "At least one channel must be given\n");

	st->sensors = devm_kcalloc(dev, st->num_channels,
				   sizeof(*st->sensors), GFP_KERNEL);
	if (!st->sensors)
		return -ENOMEM;

	device_for_each_child_node_scoped(dev, child) {
		struct ltc2983_sensor sensor;

		ret = fwnode_property_read_u32(child, "reg", &sensor.chan);
		if (ret)
			return dev_err_probe(dev, ret,
				"reg property must be given for child nodes\n");

		ret = fwnode_property_read_u32(child, "adi,sensor-type",
					       &sensor.type);
		if (ret)
			return dev_err_probe(dev, ret,
				"adi,sensor-type must be given\n");

		/* Dispatch based on sensor type */
		if (sensor.type >= LTC2983_SENSOR_THERMOCOUPLE &&
		    sensor.type <= LTC2983_SENSOR_THERMOCOUPLE_CUSTOM) {
			st->sensors[chan] = ltc2983_thermocouple_new(
							child, st, &sensor);
		} else if (sensor.type >= LTC2983_SENSOR_RTD &&
			   sensor.type <= LTC2983_SENSOR_RTD_CUSTOM) {
			st->sensors[chan] = ltc2983_rtd_new(
							child, st, &sensor);
		} else if (sensor.type >= LTC2983_SENSOR_THERMISTOR &&
			   sensor.type <= LTC2983_SENSOR_THERMISTOR_CUSTOM) {
			st->sensors[chan] = ltc2983_thermistor_new(
							child, st, &sensor);
		}
		/* ... */

		if (IS_ERR(st->sensors[chan]))
			return dev_err_probe(dev,
				PTR_ERR(st->sensors[chan]),
				"Failed to create sensor %u\n", sensor.chan);
		chan++;
	}

	return 0;
}
```

### Sensor-Specific Child Node Properties

When parsing RTD or thermistor child nodes, read excitation current and
wiring configuration:

```c
static struct ltc2983_sensor *
ltc2983_rtd_new(const struct fwnode_handle *child,
		struct ltc2983_data *st,
		const struct ltc2983_sensor *sensor)
{
	struct ltc2983_rtd *rtd;
	struct device *dev = &st->spi->dev;

	rtd = devm_kzalloc(dev, sizeof(*rtd), GFP_KERNEL);
	if (!rtd)
		return ERR_PTR(-ENOMEM);

	/* Read excitation current (microamps) */
	fwnode_property_read_u32(child, "adi,excitation-current-microamp",
				 &rtd->excitation_current);

	/* Read number of wires (2, 3, or 4) */
	fwnode_property_read_u32(child, "adi,number-of-wires",
				 &rtd->n_wires);

	/* Read sense resistor channel reference */
	fwnode_property_read_u32(child, "adi,rsense-handle",
				 &rtd->r_sense_chan);

	/* Read RTD curve type */
	fwnode_property_read_u32(child, "adi,rtd-curve", &rtd->rtd_curve);

	/* Read sensor config bits */
	fwnode_property_read_u32(child, "adi,sensor-config",
				 &rtd->sensor_config);

	rtd->sensor = *sensor;
	rtd->sensor.assign_chan = ltc2983_rtd_assign_chan;
	rtd->sensor.fault_handler = ltc2983_rtd_fault_handler;

	return &rtd->sensor;
}
```

---

## 9. Test & Debug

### sysfs Interface

Temperature channels appear in sysfs under `/sys/bus/iio/devices/iio:deviceN/`.
The naming depends on the channel configuration:

```
# Single temperature sensor (non-indexed):
in_temp_raw                    # Raw register value
in_temp_scale                  # Scale factor (millidegrees C per LSB)
in_temp_offset                 # Offset (if exposed)

# Indexed temperature channels (multi-sensor):
in_temp0_raw                   # Raw for channel 0
in_temp0_scale                 # Scale for channel 0
in_temp1_raw                   # Raw for channel 1

# Thermocouple + cold junction (modified channel):
in_temp_raw                    # Thermocouple (hot junction) raw
in_temp_scale                  # Thermocouple scale
in_temp_ambient_raw            # Cold junction raw
in_temp_ambient_scale          # Cold junction scale

# Thermocouple type (if supported):
in_temp_thermocouple_type      # Reads/writes: B, E, J, K, N, R, S, T

# Oversampling (if supported):
in_temp_oversampling_ratio     # Number of samples averaged
```

### Computing Temperature from sysfs

```sh
# Read raw and scale
RAW=$(cat /sys/bus/iio/devices/iio:device0/in_temp_raw)
SCALE=$(cat /sys/bus/iio/devices/iio:device0/in_temp_scale)

# Temperature in millidegrees C:
#   temp_mC = RAW * SCALE
# Example: RAW=3200, SCALE=7.812500
#   temp_mC = 3200 * 7.812500 = 25000 mC = 25.000 degC

# With offset:
OFFSET=$(cat /sys/bus/iio/devices/iio:device0/in_temp_offset)
# temp_mC = (RAW + OFFSET) * SCALE

# For PROCESSED channels (no math needed):
TEMP_MC=$(cat /sys/bus/iio/devices/iio:device0/in_temp_input)
# Value is already in millidegrees C
```

### Using iio_info / iio_readdev

```sh
# List all IIO devices and their channels
iio_info

# Read a single attribute
iio_attr -d iio:device0 -c in_temp_raw

# For devices with triggered buffer support:
iio_readdev -b 256 -s 100 iio:device0
```

### debugfs Register Access

If the driver implements `debugfs_reg_access` in `iio_info`:

```sh
# Read register 0x00 (temperature register)
echo 0x00 > /sys/kernel/debug/iio/iio:device0/direct_reg_access
cat /sys/kernel/debug/iio/iio:device0/direct_reg_access

# Write 0x01 to configuration register 0x01
echo 0x01 0x01 > /sys/kernel/debug/iio/iio:device0/direct_reg_access
```

### Common Validation Checks

- Verify that `in_temp_raw * in_temp_scale` produces a reasonable temperature
  in millidegrees C (e.g. ~25000 for room temperature).
- For thermocouple sensors, verify both `in_temp_raw` (hot junction) and
  `in_temp_ambient_raw` (cold junction) produce sensible readings.
- Check that `in_temp_thermocouple_type` reads back the expected type letter.
- For multi-sensor systems (LTC2983), verify each indexed channel
  (`in_temp0_raw`, `in_temp1_raw`, etc.) corresponds to the correct
  physical sensor input.

---

## 10. Key Conventions

### License

All new IIO drivers must use GPL-2.0 or GPL-2.0+:

```c
// SPDX-License-Identifier: GPL-2.0+
```

The SPDX tag is the very first line. `MODULE_LICENSE("GPL")` at the bottom
must match.

### Temperature Unit: Millidegrees Celsius

The IIO subsystem requires all `IIO_TEMP` values to be reported in
**millidegrees Celsius**. This means:

- `raw * scale` (plus offset if applicable) must yield millidegrees C.
- The scale factor is typically returned as `IIO_VAL_INT_PLUS_MICRO`.
  For example, a sensor with 7.8125 mC/LSB returns `val=7, val2=812500`.
- For `PROCESSED` channels, return the value directly in millidegrees C as
  `IIO_VAL_INT`.

### Return Value Format: IIO_VAL_INT_PLUS_MICRO for Scale

The preferred return format for temperature scale is `IIO_VAL_INT_PLUS_MICRO`:

| Return Value              | Usage                                            | Example                                    |
|---------------------------|--------------------------------------------------|--------------------------------------------|
| `IIO_VAL_INT`             | Raw reading or processed temperature             | raw = 3200                                 |
| `IIO_VAL_INT_PLUS_MICRO`  | Scale with sub-integer precision (preferred)     | 7 + 812500/1e6 = 7.8125 mC/LSB            |
| `IIO_VAL_FRACTIONAL`      | Scale as exact fraction                          | 1000/1024 mC/LSB                           |
| `IIO_VAL_FRACTIONAL_LOG2` | Scale as value / 2^N                             | 1000 / 2^10 = 0.9766 mC/LSB               |
| `IIO_VAL_CHAR`            | Thermocouple type character                      | 'K'                                        |

### Memory Management

Use `devm_*` functions exclusively. Common ones for temperature drivers:

| Function                          | Purpose                                  |
|-----------------------------------|------------------------------------------|
| `devm_iio_device_alloc()`         | Allocate IIO device + private data       |
| `devm_iio_device_register()`      | Register IIO device (auto-unregister)    |
| `devm_regulator_get_enable()`     | Get and enable a regulator               |
| `devm_regmap_init_i2c()`          | Initialize I2C regmap                    |
| `devm_regmap_init_spi()`          | Initialize SPI regmap                    |
| `devm_mutex_init()`               | Initialize a mutex with devm cleanup     |
| `devm_kcalloc()`                  | Allocate zeroed array (dynamic channels) |
| `devm_request_irq()`              | Request interrupt with auto-free         |

### Locking

- Use `struct mutex` to protect device state and bus transactions.
- Initialize with `devm_mutex_init()` in `probe()`.
- Use `guard(mutex)(&st->lock)` for scoped locking where possible.
- Use `iio_device_claim_direct()` / `iio_device_release_direct()` to prevent
  raw reads while the buffer is active.

### Coding Style

- Follow the kernel coding style (`Documentation/process/coding-style.rst`).
- Run `scripts/checkpatch.pl` before submitting.
- Use `FIELD_GET()` / `FIELD_PREP()` with `GENMASK()` for register bitfields.
- Use `BIT()` for single-bit definitions.
- Use `dev_err_probe()` for all error returns in `probe()`.
- Include headers in alphabetical order within each group.

---

## 11. Commit Message Format

### Subject Line Prefix

```
iio: temperature: <devname>: <brief description>
```

### Examples

```
iio: temperature: adt7420: add support for ADT7420 and ADT7410
iio: temperature: ltc2983: add support for ADT7604
iio: temperature: max31856: fix thermocouple type validation
iio: temperature: tmp117: use devm_iio_device_register
```

### DT Binding Commit

```
dt-bindings: iio: temperature: add adi,adt7420.yaml
```

### Patch Series for a New Temperature Driver

1. `dt-bindings: iio: temperature: add adi,<devname>.yaml` -- DT binding
2. `iio: temperature: <devname>: add support for <DEVNAME>` -- Driver source
3. `MAINTAINERS: add entry for <DEVNAME> IIO driver` -- Maintainer entry

### Full Example

```
iio: temperature: adt7420: add support for ADT7420 and ADT7410

The ADT7420 is a high accuracy 16-bit digital temperature sensor with
an I2C interface. It achieves +/-0.25 degC accuracy from -40 to +125
degC without requiring calibration.

This driver supports:
  - Single temperature reading via IIO_TEMP channel
  - 13-bit and 16-bit resolution modes
  - Programmable overtemperature interrupt thresholds

Signed-off-by: First Last <first.last@analog.com>
```
