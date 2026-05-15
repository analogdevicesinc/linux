# Linux IIO Light / Photo-Electronic Driver Template

This template covers IIO drivers in the `iio/light/` subdirectory: ambient light
sensors (ALS), infrared intensity sensors, proximity sensors, and distance
sensors. It builds on the consolidated IIO driver template with light-specific
channel types, event handling, and DT conventions.

---

## 1. Purpose

The `iio/light/` subdirectory hosts drivers for devices that measure optical or
photo-electronic quantities:

- **Ambient light** -- visible-spectrum illuminance (lux).
- **Intensity** -- raw photodiode readings for individual spectral bands
  (visible, IR, clear/white, RGB).
- **Proximity** -- reflected-IR distance indication (unitless count or mm).
- **Distance** -- calibrated range measurement.

### IIO Channel Types

| Measurement         | IIO Channel Type  | Typical Modifier(s)                                         |
|---------------------|-------------------|-------------------------------------------------------------|
| Illuminance (lux)   | `IIO_LIGHT`       | None (unmodified)                                           |
| Raw intensity       | `IIO_INTENSITY`   | `IIO_MOD_LIGHT_CLEAR`, `IIO_MOD_LIGHT_RED`, `IIO_MOD_LIGHT_GREEN`, `IIO_MOD_LIGHT_BLUE`, `IIO_MOD_LIGHT_BOTH`, `IIO_MOD_LIGHT_IR` |
| Proximity           | `IIO_PROXIMITY`   | None (unmodified)                                           |
| Distance            | `IIO_DISTANCE`    | None (unmodified)                                           |

### Typical `info_mask` Bits

| `info_mask` Bit                | Purpose                                           |
|--------------------------------|---------------------------------------------------|
| `IIO_CHAN_INFO_RAW`            | Raw ADC / photodiode count                        |
| `IIO_CHAN_INFO_PROCESSED`      | Computed lux value (resolution x raw)             |
| `IIO_CHAN_INFO_SCALE`          | Gain setting (ALS gain, proximity gain)           |
| `IIO_CHAN_INFO_INT_TIME`       | Integration time in seconds (fractional)          |
| `IIO_CHAN_INFO_CALIBSCALE`     | Factory or user calibration multiplier            |
| `IIO_CHAN_INFO_CALIBBIAS`      | Proximity offset calibration                      |
| `IIO_CHAN_INFO_SAMP_FREQ`     | Measurement repetition rate                       |

---

## 2. File Checklist

| File                                                                  | Action   | Required |
|-----------------------------------------------------------------------|----------|----------|
| `drivers/iio/light/<devname>.c`                                       | Create   | Yes      |
| `drivers/iio/light/Kconfig`                                           | Modify   | Yes      |
| `drivers/iio/light/Makefile`                                          | Modify   | Yes      |
| `Documentation/devicetree/bindings/iio/light/adi,<devname>.yaml`      | Create   | Yes      |
| `include/dt-bindings/iio/light/<devname>.h`                           | Create   | Optional |

### Notes

- Most light sensors are I2C devices. A header file is only needed when DT
  binding constants (e.g. LED current enums, channel configuration enums)
  must be shared between the binding YAML and the driver source.
- If the device has both an I2C and SPI interface, split into
  `<devname>-core.c`, `<devname>-i2c.c`, `<devname>-spi.c` with a shared
  header (see `st_uvis25` for an example).

---

## 3. Devicetree Binding (.yaml)

Light sensors are almost always I2C devices. Key properties beyond the standard
`compatible`, `reg`, and `interrupts`:

- **`led-current-microamp`** -- Proximity LED drive current in microamps.
- **`proximity-near-level`** -- Proximity threshold for "near" detection.
- **`vishay,window-factor`** / **`adi,window-factor`** -- Lens / window
  transmittance correction factor.

### Light Sensor Binding Example

```yaml
# SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)
%YAML 1.2
---
$id: http://devicetree.org/schemas/iio/light/adi,adxxxx.yaml#
$schema: http://devicetree.org/meta-schemas/core.yaml#

title: Analog Devices ADxxxx Ambient Light / Proximity Sensor

maintainers:
  - First Last <first.last@analog.com>

description: |
  The ADxxxx is a combined ambient light and proximity sensor with I2C
  interface. It provides visible light, IR, and proximity channels with
  programmable gain and integration time.

properties:
  compatible:
    enum:
      - adi,adxxxx

  reg:
    maxItems: 1

  interrupts:
    maxItems: 1
    description:
      Active-low, open-drain interrupt output for threshold events.

  vdd-supply: true

  led-current-microamp:
    description: Proximity LED drive current.
    enum: [12500, 25000, 50000, 100000]
    default: 100000

  proximity-near-level:
    $ref: /schemas/types.yaml#/definitions/uint32
    description: Proximity ADC count considered as "near".

required:
  - compatible
  - reg

additionalProperties: false

examples:
  - |
    #include <linux/interrupt.h>
    i2c {
        #address-cells = <1>;
        #size-cells = <0>;

        light-sensor@39 {
            compatible = "adi,adxxxx";
            reg = <0x39>;
            interrupts = <15 IRQ_TYPE_LEVEL_LOW>;
            interrupt-parent = <&gpio>;
            vdd-supply = <&vdd_3v3>;
            led-current-microamp = <50000>;
        };
    };
```

---

## 4. Kconfig

Add to `drivers/iio/light/Kconfig` in alphabetical order.

```kconfig
config ADXXXX
	tristate "Analog Devices ADxxxx ambient light and proximity sensor"
	depends on I2C
	select REGMAP_I2C
	help
	  Say yes here to build support for the Analog Devices ADxxxx
	  ambient light, IR intensity, and proximity sensor.

	  This driver provides raw and processed illuminance readings,
	  IR intensity, and proximity detection through the IIO subsystem.

	  To compile this driver as a module, choose M here: the module will
	  be called adxxxx.
```

---

## 5. Makefile

Add to `drivers/iio/light/Makefile` in alphabetical order.

```makefile
obj-$(CONFIG_ADXXXX) += adxxxx.o
```

---

## 6. Driver Source (.c)

### Light-Specific Channel Definitions

A combined ALS + proximity sensor typically exposes multiple channel types.
Use `IIO_LIGHT` for the computed-lux channel, `IIO_INTENSITY` with modifiers
for raw spectral bands, and `IIO_PROXIMITY` for the proximity engine.

```c
// SPDX-License-Identifier: GPL-2.0+
/*
 * ADxxxx Ambient Light / Proximity Sensor driver
 *
 * Copyright 2024 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include <linux/iio/iio.h>
#include <linux/iio/events.h>
#include <linux/iio/sysfs.h>

/* ------------------------------------------------------------------ */
/* Register Definitions                                                */
/* ------------------------------------------------------------------ */

#define ADXXXX_REG_CONTROL		0x00
#define ADXXXX_REG_ALS_DATA		0x04
#define ADXXXX_REG_IR_DATA		0x06
#define ADXXXX_REG_PROX_DATA		0x08
#define ADXXXX_REG_ALS_THRESH_LO	0x0A
#define ADXXXX_REG_ALS_THRESH_HI	0x0C
#define ADXXXX_REG_PROX_THRESH_LO	0x0E
#define ADXXXX_REG_PROX_THRESH_HI	0x10
#define ADXXXX_REG_INT_STATUS		0x12
#define ADXXXX_REG_INT_ENABLE		0x14
#define ADXXXX_REG_GAIN			0x16
#define ADXXXX_REG_INTEG_TIME		0x18
#define ADXXXX_REG_LED_CURRENT		0x1A
#define ADXXXX_REG_ID			0x1C

#define ADXXXX_ALS_GAIN			GENMASK(3, 0)
#define ADXXXX_PROX_GAIN		GENMASK(7, 4)
#define ADXXXX_ALS_INTEG_TIME		GENMASK(3, 0)
#define ADXXXX_LED_CURRENT		GENMASK(7, 0)
#define ADXXXX_INT_ALS			BIT(0)
#define ADXXXX_INT_PROX			BIT(1)

/* ------------------------------------------------------------------ */
/* Device State                                                        */
/* ------------------------------------------------------------------ */

struct adxxxx_data {
	struct regmap		*regmap;
	struct mutex		lock;		/* Protect device state */
	int			als_gain;
	int			als_int_time_us;
	int			prox_gain;
	int			cur_resolution;	/* Resolution x 10000 for lux calc */
	int			led_current_ua;
};

/* ------------------------------------------------------------------ */
/* Gain & Integration Time Tables                                      */
/* ------------------------------------------------------------------ */

static const int adxxxx_als_gain_map[] = { 1, 4, 16, 64 };
static const int adxxxx_prox_gain_map[] = { 1, 2, 4, 8 };

/*
 * Integration time in microseconds and corresponding register values.
 * Columns: { time_us, reg_value }
 */
static const int adxxxx_int_time[][2] = {
	{  25000, 0x00 },
	{  50000, 0x01 },
	{ 100000, 0x02 },
	{ 200000, 0x03 },
	{ 400000, 0x04 },
	{ 800000, 0x05 },
};

static IIO_CONST_ATTR(in_illuminance_integration_time_available,
		       "0.025 0.05 0.1 0.2 0.4 0.8");
static IIO_CONST_ATTR(in_illuminance_scale_available, "1 4 16 64");
static IIO_CONST_ATTR(proximity_scale_available, "1 2 4 8");

static struct attribute *adxxxx_attributes[] = {
	&iio_const_attr_in_illuminance_integration_time_available.dev_attr.attr,
	&iio_const_attr_in_illuminance_scale_available.dev_attr.attr,
	&iio_const_attr_proximity_scale_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group adxxxx_attr_group = {
	.attrs = adxxxx_attributes,
};

/* ------------------------------------------------------------------ */
/* IIO Channel Specification                                           */
/* ------------------------------------------------------------------ */

/*
 * Multi-channel layout:
 *   - IIO_LIGHT:     computed illuminance (lux), with INT_TIME and SCALE
 *   - IIO_INTENSITY: raw visible-light and IR photodiode counts
 *   - IIO_PROXIMITY: reflected-IR proximity count
 */

static const struct iio_event_spec adxxxx_als_event_spec[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE),
	},
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE),
	},
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_EITHER,
		.mask_separate = BIT(IIO_EV_INFO_ENABLE),
	},
};

static const struct iio_event_spec adxxxx_prox_event_spec[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE),
	},
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE),
	},
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_EITHER,
		.mask_separate = BIT(IIO_EV_INFO_ENABLE),
	},
};

enum adxxxx_chan {
	CHAN_ALS,
	CHAN_VISIBLE,
	CHAN_IR,
	CHAN_PROXIMITY,
};

static const struct iio_chan_spec adxxxx_channels[] = {
	/* Illuminance channel -- provides computed lux via PROCESSED */
	{
		.type = IIO_LIGHT,
		.channel = CHAN_ALS,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				BIT(IIO_CHAN_INFO_PROCESSED) |
				BIT(IIO_CHAN_INFO_INT_TIME) |
				BIT(IIO_CHAN_INFO_SCALE),
		.event_spec = adxxxx_als_event_spec,
		.num_event_specs = ARRAY_SIZE(adxxxx_als_event_spec),
	},
	/* Visible-light intensity (clear/broadband) */
	{
		.type = IIO_INTENSITY,
		.modified = 1,
		.channel2 = IIO_MOD_LIGHT_CLEAR,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |
				BIT(IIO_CHAN_INFO_INT_TIME),
	},
	/* IR intensity */
	{
		.type = IIO_INTENSITY,
		.modified = 1,
		.channel2 = IIO_MOD_LIGHT_IR,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |
				BIT(IIO_CHAN_INFO_INT_TIME),
	},
	/* Proximity */
	{
		.type = IIO_PROXIMITY,
		.channel = CHAN_PROXIMITY,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				BIT(IIO_CHAN_INFO_CALIBBIAS),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),
		.event_spec = adxxxx_prox_event_spec,
		.num_event_specs = ARRAY_SIZE(adxxxx_prox_event_spec),
	},
};

/* ------------------------------------------------------------------ */
/* Lux Calculation Helper                                              */
/* ------------------------------------------------------------------ */

/*
 * Convert raw visible and IR counts to a lux value.
 *
 * Common algorithm (device-specific coefficients from datasheet):
 *   lux = (C0_coeff * visible_raw - C1_coeff * ir_raw) / (gain * int_time)
 *
 * Some devices provide a direct lux-per-count resolution factor that
 * varies with gain and integration time. In that case:
 *   lux = resolution * raw_count
 *
 * The resolution factor should be cached and recalculated whenever gain
 * or integration time changes (see veml6030 for a reference pattern).
 */
static int adxxxx_calc_lux(struct adxxxx_data *data, int *val, int *val2)
{
	unsigned int als_raw;
	int ret;

	ret = regmap_read(data->regmap, ADXXXX_REG_ALS_DATA, &als_raw);
	if (ret)
		return ret;

	/*
	 * lux = resolution * raw / 10000
	 * Return as IIO_VAL_INT_PLUS_MICRO.
	 */
	*val = (als_raw * data->cur_resolution) / 10000;
	*val2 = (als_raw * data->cur_resolution) % 10000 * 100;

	return IIO_VAL_INT_PLUS_MICRO;
}

/* ------------------------------------------------------------------ */
/* read_raw / write_raw                                                */
/* ------------------------------------------------------------------ */

static int adxxxx_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	struct adxxxx_data *data = iio_priv(indio_dev);
	unsigned int regval;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->type) {
		case IIO_LIGHT:
		case IIO_INTENSITY:
			if (chan->channel2 == IIO_MOD_LIGHT_IR)
				ret = regmap_read(data->regmap,
						  ADXXXX_REG_IR_DATA, &regval);
			else
				ret = regmap_read(data->regmap,
						  ADXXXX_REG_ALS_DATA, &regval);
			if (ret)
				return ret;
			*val = regval;
			return IIO_VAL_INT;

		case IIO_PROXIMITY:
			ret = regmap_read(data->regmap,
					  ADXXXX_REG_PROX_DATA, &regval);
			if (ret)
				return ret;
			*val = regval;
			return IIO_VAL_INT;

		default:
			return -EINVAL;
		}

	case IIO_CHAN_INFO_PROCESSED:
		if (chan->type != IIO_LIGHT)
			return -EINVAL;

		guard(mutex)(&data->lock);
		return adxxxx_calc_lux(data, val, val2);

	case IIO_CHAN_INFO_INT_TIME:
		guard(mutex)(&data->lock);
		*val = 0;
		*val2 = data->als_int_time_us;
		return IIO_VAL_INT_PLUS_MICRO;

	case IIO_CHAN_INFO_SCALE:
		guard(mutex)(&data->lock);
		switch (chan->type) {
		case IIO_LIGHT:
		case IIO_INTENSITY:
			*val = adxxxx_als_gain_map[data->als_gain];
			return IIO_VAL_INT;
		case IIO_PROXIMITY:
			*val = adxxxx_prox_gain_map[data->prox_gain];
			return IIO_VAL_INT;
		default:
			return -EINVAL;
		}

	default:
		return -EINVAL;
	}
}

static int adxxxx_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long mask)
{
	struct adxxxx_data *data = iio_priv(indio_dev);
	int i;

	switch (mask) {
	case IIO_CHAN_INFO_INT_TIME:
		if (val != 0)
			return -EINVAL;

		for (i = 0; i < ARRAY_SIZE(adxxxx_int_time); i++) {
			if (adxxxx_int_time[i][0] == val2) {
				guard(mutex)(&data->lock);
				/* Write register, update cached resolution */
				data->als_int_time_us = val2;
				return regmap_update_bits(data->regmap,
						ADXXXX_REG_INTEG_TIME,
						ADXXXX_ALS_INTEG_TIME,
						adxxxx_int_time[i][1]);
			}
		}
		return -EINVAL;

	case IIO_CHAN_INFO_SCALE:
		if (val2 != 0)
			return -EINVAL;

		switch (chan->type) {
		case IIO_LIGHT:
		case IIO_INTENSITY:
			for (i = 0; i < ARRAY_SIZE(adxxxx_als_gain_map); i++) {
				if (adxxxx_als_gain_map[i] == val) {
					guard(mutex)(&data->lock);
					data->als_gain = i;
					return regmap_update_bits(data->regmap,
							ADXXXX_REG_GAIN,
							ADXXXX_ALS_GAIN, i);
				}
			}
			return -EINVAL;

		case IIO_PROXIMITY:
			for (i = 0; i < ARRAY_SIZE(adxxxx_prox_gain_map); i++) {
				if (adxxxx_prox_gain_map[i] == val) {
					guard(mutex)(&data->lock);
					data->prox_gain = i;
					return regmap_update_bits(data->regmap,
							ADXXXX_REG_GAIN,
							ADXXXX_PROX_GAIN,
							FIELD_PREP(ADXXXX_PROX_GAIN, i));
				}
			}
			return -EINVAL;

		default:
			return -EINVAL;
		}

	default:
		return -EINVAL;
	}
}
```

---

## 7. Threshold Events

Light and proximity sensors typically generate hardware interrupts when a
measurement crosses a configured threshold. The IIO events subsystem exposes
these through `events/` in sysfs.

### Event Specification

```c
/*
 * Each threshold channel (ALS and proximity) needs:
 *   - RISING threshold value
 *   - FALLING threshold value
 *   - ENABLE control
 *
 * For hysteresis support, add IIO_EV_INFO_HYSTERESIS to the mask:
 */
static const struct iio_event_spec adxxxx_als_event_spec[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
				 BIT(IIO_EV_INFO_HYSTERESIS),
	},
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
				 BIT(IIO_EV_INFO_HYSTERESIS),
	},
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_EITHER,
		.mask_separate = BIT(IIO_EV_INFO_PERIOD) |
				 BIT(IIO_EV_INFO_ENABLE),
	},
};
```

### Event Callbacks

```c
static int adxxxx_read_event_value(struct iio_dev *indio_dev,
				   const struct iio_chan_spec *chan,
				   enum iio_event_type type,
				   enum iio_event_direction dir,
				   enum iio_event_info info,
				   int *val, int *val2)
{
	struct adxxxx_data *data = iio_priv(indio_dev);
	unsigned int reg;
	int ret;

	switch (info) {
	case IIO_EV_INFO_VALUE:
		switch (chan->type) {
		case IIO_LIGHT:
			reg = (dir == IIO_EV_DIR_RISING) ?
				ADXXXX_REG_ALS_THRESH_HI :
				ADXXXX_REG_ALS_THRESH_LO;
			break;
		case IIO_PROXIMITY:
			reg = (dir == IIO_EV_DIR_RISING) ?
				ADXXXX_REG_PROX_THRESH_HI :
				ADXXXX_REG_PROX_THRESH_LO;
			break;
		default:
			return -EINVAL;
		}

		ret = regmap_read(data->regmap, reg, val);
		if (ret)
			return ret;
		*val2 = 0;
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

static int adxxxx_write_event_value(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    enum iio_event_type type,
				    enum iio_event_direction dir,
				    enum iio_event_info info,
				    int val, int val2)
{
	struct adxxxx_data *data = iio_priv(indio_dev);
	unsigned int reg;

	if (info != IIO_EV_INFO_VALUE)
		return -EINVAL;

	switch (chan->type) {
	case IIO_LIGHT:
		reg = (dir == IIO_EV_DIR_RISING) ?
			ADXXXX_REG_ALS_THRESH_HI :
			ADXXXX_REG_ALS_THRESH_LO;
		break;
	case IIO_PROXIMITY:
		reg = (dir == IIO_EV_DIR_RISING) ?
			ADXXXX_REG_PROX_THRESH_HI :
			ADXXXX_REG_PROX_THRESH_LO;
		break;
	default:
		return -EINVAL;
	}

	return regmap_write(data->regmap, reg, val);
}

static int adxxxx_read_event_config(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    enum iio_event_type type,
				    enum iio_event_direction dir)
{
	struct adxxxx_data *data = iio_priv(indio_dev);
	unsigned int reg;
	int ret;

	ret = regmap_read(data->regmap, ADXXXX_REG_INT_ENABLE, &reg);
	if (ret)
		return ret;

	switch (chan->type) {
	case IIO_LIGHT:
		return !!(reg & ADXXXX_INT_ALS);
	case IIO_PROXIMITY:
		return !!(reg & ADXXXX_INT_PROX);
	default:
		return -EINVAL;
	}
}

static int adxxxx_write_event_config(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     enum iio_event_type type,
				     enum iio_event_direction dir,
				     int state)
{
	struct adxxxx_data *data = iio_priv(indio_dev);
	unsigned int mask;

	switch (chan->type) {
	case IIO_LIGHT:
		mask = ADXXXX_INT_ALS;
		break;
	case IIO_PROXIMITY:
		mask = ADXXXX_INT_PROX;
		break;
	default:
		return -EINVAL;
	}

	if (state)
		return regmap_set_bits(data->regmap,
				       ADXXXX_REG_INT_ENABLE, mask);
	else
		return regmap_clear_bits(data->regmap,
					ADXXXX_REG_INT_ENABLE, mask);
}
```

### Interrupt Handler

```c
static irqreturn_t adxxxx_irq_handler(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct adxxxx_data *data = iio_priv(indio_dev);
	unsigned int status;
	int ret;

	ret = regmap_read(data->regmap, ADXXXX_REG_INT_STATUS, &status);
	if (ret)
		return IRQ_HANDLED;

	if (status & ADXXXX_INT_ALS) {
		iio_push_event(indio_dev,
			       IIO_UNMOD_EVENT_CODE(IIO_LIGHT, 0,
						    IIO_EV_TYPE_THRESH,
						    IIO_EV_DIR_EITHER),
			       iio_get_time_ns(indio_dev));
	}

	if (status & ADXXXX_INT_PROX) {
		iio_push_event(indio_dev,
			       IIO_UNMOD_EVENT_CODE(IIO_PROXIMITY, 0,
						    IIO_EV_TYPE_THRESH,
						    IIO_EV_DIR_EITHER),
			       iio_get_time_ns(indio_dev));
	}

	/* Clear interrupt flags by writing status back */
	regmap_write(data->regmap, ADXXXX_REG_INT_STATUS, status);

	return IRQ_HANDLED;
}
```

### iio_info with Events

```c
static const struct iio_info adxxxx_info = {
	.read_raw = adxxxx_read_raw,
	.write_raw = adxxxx_write_raw,
	.read_event_value = adxxxx_read_event_value,
	.write_event_value = adxxxx_write_event_value,
	.read_event_config = adxxxx_read_event_config,
	.write_event_config = adxxxx_write_event_config,
	.attrs = &adxxxx_attr_group,
};

/* When no IRQ is available, omit event callbacks */
static const struct iio_info adxxxx_info_no_irq = {
	.read_raw = adxxxx_read_raw,
	.write_raw = adxxxx_write_raw,
	.attrs = &adxxxx_attr_group,
};
```

---

## 8. Devicetree Parsing

Light sensors may have DT properties for proximity LED current, window
correction factor, and other calibration parameters.

```c
static int adxxxx_parse_dt(struct device *dev, struct adxxxx_data *data)
{
	u32 led_current;
	int ret;

	/* LED drive current for proximity, default 100 mA */
	ret = device_property_read_u32(dev, "led-current-microamp",
				       &led_current);
	if (ret)
		led_current = 100000;

	data->led_current_ua = led_current;

	return regmap_update_bits(data->regmap, ADXXXX_REG_LED_CURRENT,
				  ADXXXX_LED_CURRENT,
				  led_current / 1000);
}
```

### Common Light Sensor DT Properties

| Property                  | API                                    | Purpose                                      |
|---------------------------|----------------------------------------|----------------------------------------------|
| `led-current-microamp`    | `device_property_read_u32()`           | Proximity emitter LED drive current           |
| `proximity-near-level`    | `device_property_read_u32()`           | Proximity threshold for "near" indication     |
| `adi,window-factor`       | `device_property_read_u32()`           | Lens/window transmittance correction factor   |
| `adi,wait-time-us`        | `device_property_read_u32()`           | Wait time between measurement cycles          |
| `interrupts`              | `client->irq` (auto-parsed)           | Threshold interrupt GPIO                      |

---

## 9. Test & Debug

### sysfs Interface for Light Sensors

Light sensors expose the following attributes under
`/sys/bus/iio/devices/iio:deviceN/`:

```
# Illuminance (lux)
in_illuminance_raw                    # Raw ALS ADC count
in_illuminance_input                  # Processed lux reading (PROCESSED)
in_illuminance_scale                  # Current ALS gain setting
in_illuminance_integration_time       # Current integration time (seconds)

# Intensity -- modified channels (visible, IR, clear, etc.)
in_intensity_clear_raw                # Raw visible/clear photodiode count
in_intensity_ir_raw                   # Raw IR photodiode count
in_intensity_both_raw                 # Raw white/broadband photodiode count

# Proximity
in_proximity_raw                      # Raw proximity ADC count
in_proximity_scale                    # Proximity gain setting
in_proximity_calibbias                # Proximity offset calibration

# Available values
in_illuminance_integration_time_available
in_illuminance_scale_available
proximity_scale_available
```

### Event Interface

```sh
# Threshold events (requires interrupt wired)
events/in_illuminance_thresh_rising_value     # ALS high threshold
events/in_illuminance_thresh_falling_value    # ALS low threshold
events/in_illuminance_thresh_either_en        # ALS interrupt enable

events/in_proximity_thresh_rising_value       # Proximity high threshold
events/in_proximity_thresh_falling_value      # Proximity low threshold
events/in_proximity_thresh_either_en          # Proximity interrupt enable

# Monitor events with iio_event_monitor
iio_event_monitor /dev/iio:device0
```

### Userspace Testing Commands

```sh
# Read illuminance
cat /sys/bus/iio/devices/iio:device0/in_illuminance_raw
cat /sys/bus/iio/devices/iio:device0/in_illuminance_input

# Read proximity
cat /sys/bus/iio/devices/iio:device0/in_proximity_raw

# Read IR intensity
cat /sys/bus/iio/devices/iio:device0/in_intensity_ir_raw

# Change integration time to 200 ms
echo 0.2 > /sys/bus/iio/devices/iio:device0/in_illuminance_integration_time

# Change ALS gain to 16x
echo 16 > /sys/bus/iio/devices/iio:device0/in_illuminance_scale

# Set proximity threshold and enable interrupt
echo 100 > /sys/bus/iio/devices/iio:device0/events/in_proximity_thresh_rising_value
echo 20 > /sys/bus/iio/devices/iio:device0/events/in_proximity_thresh_falling_value
echo 1 > /sys/bus/iio/devices/iio:device0/events/in_proximity_thresh_either_en

# List all attributes
iio_info
iio_attr -d iio:device0 -c
```

### debugfs Register Access

```sh
# Requires CONFIG_DEBUG_FS
echo 0x16 > /sys/kernel/debug/iio/iio:device0/direct_reg_access
cat /sys/kernel/debug/iio/iio:device0/direct_reg_access
```

---

## 10. Key Conventions

### License

All new drivers must use GPL-2.0 or GPL-2.0+:

```c
// SPDX-License-Identifier: GPL-2.0+
```

The SPDX tag goes on the very first line. `MODULE_LICENSE("GPL")` at the bottom
must match.

### Units

| Channel Type    | sysfs Name Prefix        | Unit for PROCESSED             |
|-----------------|--------------------------|--------------------------------|
| `IIO_LIGHT`     | `in_illuminance_`        | Lux                            |
| `IIO_INTENSITY` | `in_intensity_`          | Unitless (raw count)           |
| `IIO_PROXIMITY` | `in_proximity_`          | Unitless (raw count, lower = farther) |
| `IIO_DISTANCE`  | `in_distance_`           | Meters                         |

- `IIO_LIGHT` with `IIO_CHAN_INFO_PROCESSED` must return values in **lux**.
- Integration time is expressed in **seconds** (fractional, via
  `IIO_VAL_INT_PLUS_MICRO`).
- Scale (gain) is typically a dimensionless multiplier.

### Modifiers for Color / Spectral Channels

When a sensor provides per-color channels, use `IIO_INTENSITY` with `.modified = 1`
and set `.channel2` to the appropriate modifier:

| Modifier                  | sysfs Suffix   | Description           |
|---------------------------|----------------|-----------------------|
| `IIO_MOD_LIGHT_CLEAR`    | `_clear`       | Broadband visible     |
| `IIO_MOD_LIGHT_RED`      | `_red`         | Red spectral band     |
| `IIO_MOD_LIGHT_GREEN`    | `_green`       | Green spectral band   |
| `IIO_MOD_LIGHT_BLUE`     | `_blue`        | Blue spectral band    |
| `IIO_MOD_LIGHT_IR`       | `_ir`          | Infrared band         |
| `IIO_MOD_LIGHT_BOTH`     | `_both`        | White / all bands     |
| `IIO_MOD_LIGHT_UV`       | `_uv`          | Ultraviolet band      |

### Memory Management

Use `devm_*` functions exclusively. Common patterns for light drivers:

```c
devm_iio_device_alloc()           /* Allocate IIO device + private data */
devm_regmap_init_i2c()            /* Initialize I2C regmap */
devm_regulator_get_enable()       /* Power supply */
devm_request_threaded_irq()       /* Threshold interrupt */
devm_add_action_or_reset()        /* Custom shutdown cleanup */
devm_iio_device_register()        /* Register IIO device */
devm_mutex_init()                 /* Initialize mutex */
```

### Coding Style Reminders

- Use `FIELD_GET()` / `FIELD_PREP()` with `GENMASK()` for register bitfields.
- Use `guard(mutex)(&data->lock)` for scoped locking where possible.
- Use `dev_err_probe()` for all error returns in `probe()`.
- Include headers in alphabetical order within each group.

---

## 11. Commit Message Format

### Subject Line

```
iio: light: <devname>: <brief description>
```

### Examples

```
iio: light: adxxxx: add support for ADxxxx ALS/proximity sensor
iio: light: adxxxx: add proximity threshold event support
iio: light: adxxxx: fix integration time calculation
iio: light: adxxxx: use devm_iio_device_register
```

### Patch Series for a New Light Driver

1. `dt-bindings: iio: light: add adi,adxxxx.yaml` -- DT binding
2. `iio: light: adxxxx: add support for ADxxxx` -- Driver source
3. `MAINTAINERS: add entry for ADxxxx IIO driver` -- Maintainer entry

### Full Example

```
iio: light: adxxxx: add support for ADxxxx ALS/proximity sensor

The ADxxxx is a combined ambient light and proximity sensor with
I2C interface. It integrates visible and IR photodiodes plus an
LED-driven proximity engine.

This driver supports:
  - Ambient light measurement with configurable gain and integration time
  - IR intensity raw readings
  - Proximity detection with programmable LED current
  - Lux calculation from raw visible/IR channel data
  - ALS and proximity threshold interrupt events

Signed-off-by: First Last <first.last@analog.com>
```
