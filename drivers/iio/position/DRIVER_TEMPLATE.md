# Linux IIO Position Sensor Driver Template

This template covers the file structure, code skeletons, and conventions
needed to write a Linux kernel IIO driver for a position sensor (angular
encoder, linear position encoder, or multi-turn absolute position sensor)
that maps into the `iio/position/` subsystem.

Reference drivers:
- `drivers/iio/position/iqs624-pos.c` -- Azoteq IQS624/625 angular position
- `drivers/iio/position/hid-sensor-custom-intel-hinge.c` -- HID hinge angles

Replace `<devname>` with the part number (e.g., `ad2s1210`) and `<DEVNAME>`
with its uppercase form (e.g., `AD2S1210`) throughout.

---

## 1. Purpose

The `iio/position/` subsystem covers angular and linear position sensors:
rotary encoders, linear encoders, resolvers, and similar devices that report
a mechanical or electromagnetic position measurement.

### IIO Channel Types

| Channel Type    | Usage                                   | sysfs prefix   |
|-----------------|-----------------------------------------|----------------|
| `IIO_ANGL`      | Angular position (rotary encoders)     | `in_angl`      |
| `IIO_POSITION`  | Linear position                         | `in_position`  |

### Typical info_mask Bits

| Bit                        | Purpose                                           |
|----------------------------|---------------------------------------------------|
| `IIO_CHAN_INFO_RAW`        | Raw position count from the converter              |
| `IIO_CHAN_INFO_SCALE`      | Conversion factor: RAW * SCALE = position in radians (angular) or meters (linear) |
| `IIO_CHAN_INFO_OFFSET`     | Zero-position offset (calibration datum)           |

### Unit Conventions

- **Angular position** (`IIO_ANGL`): the IIO subsystem expects radians.
  The scale factor converts raw counts to radians. Userspace tools commonly
  convert to degrees (multiply by 180/pi). The iqs624-pos driver returns
  scale as `(pi/180) * interval_divisor`, converting degree-based raw values
  to radians.
- **Linear position** (`IIO_POSITION`): no fixed kernel unit; typically
  meters or millimeters depending on the device. Document the unit in the
  driver and binding.

---

## 2. File Checklist

| File                                                                 | Action   | Required |
|----------------------------------------------------------------------|----------|----------|
| `drivers/iio/position/<devname>.c`                                   | Create   | Yes      |
| `drivers/iio/position/Kconfig`                                       | Modify   | Yes      |
| `drivers/iio/position/Makefile`                                      | Modify   | Yes      |
| `Documentation/devicetree/bindings/iio/position/adi,<devname>.yaml`  | Create   | Yes      |
| `include/dt-bindings/iio/position/adi,<devname>.h`                   | Create   | Optional |

### Notes

- A header under `include/dt-bindings/` is needed only when the binding
  shares enum constants (e.g., operating-mode values) between the YAML
  binding and the driver source.
- Position sensor drivers live in `drivers/iio/position/`, not
  `drivers/iio/resolver/`. The resolver subdirectory is reserved for
  resolver-to-digital converters that use excitation/demodulation.

---

## 3. DT Binding (.yaml)

Bindings live under
`Documentation/devicetree/bindings/iio/position/adi,<devname>.yaml`.

Most ADI position sensors use SPI. The binding describes the bus
connection, resolution selection, and operating mode.

```yaml
# SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)
%YAML 1.2
---
$id: http://devicetree.org/schemas/iio/position/adi,adxxxx.yaml#
$schema: http://devicetree.org/meta-schemas/core.yaml#

title: Analog Devices ADXXXX Position Sensor

maintainers:
  - First Last <first.last@analog.com>

description: |
  The ADXXXX is an angular/linear position sensor with a <N>-bit
  converter. It supports absolute and incremental operating modes.

properties:
  compatible:
    enum:
      - adi,adxxxx

  reg:
    maxItems: 1

  spi-max-frequency:
    maximum: 10000000

  clocks:
    maxItems: 1
    description: External clock input, if required.

  clock-names:
    const: mclk

  vdd-supply:
    description: Main power supply.

  reset-gpios:
    maxItems: 1
    description: Hardware reset, active low.

  adi,resolution:
    $ref: /schemas/types.yaml#/definitions/uint32
    enum: [10, 12, 14, 16]
    description: |
      Converter resolution in bits. Determines the number of counts
      per revolution (angular) or per full-scale range (linear).

  adi,mode:
    $ref: /schemas/types.yaml#/definitions/string
    enum: [absolute, incremental]
    default: absolute
    description: |
      Operating mode. Absolute mode reports the position as a single
      reading relative to a fixed datum. Incremental mode reports
      position changes from a reference point.

required:
  - compatible
  - reg
  - vdd-supply

additionalProperties: false

examples:
  - |
    #include <linux/gpio/consumer.h>

    spi {
        #address-cells = <1>;
        #size-cells = <0>;

        position@0 {
            compatible = "adi,adxxxx";
            reg = <0>;
            spi-max-frequency = <10000000>;
            vdd-supply = <&vdd_3v3>;
            reset-gpios = <&gpio 5 GPIO_ACTIVE_LOW>;
            adi,resolution = <16>;
            adi,mode = "absolute";
        };
    };
```

---

## 4. Kconfig

Add the entry to `drivers/iio/position/Kconfig` in **alphabetical order**,
inside the existing `menu "Linear and angular position sensors"` block.

```kconfig
config ADXXXX
	tristate "Analog Devices ADXXXX position sensor driver"
	depends on SPI_MASTER
	select REGMAP_SPI
	help
	  Say yes here to build support for the Analog Devices ADXXXX
	  angular position sensor.

	  To compile this driver as a module, choose M here: the module
	  will be called adxxxx.
```

### Common Optional Selects for Position Drivers

| Select                 | Purpose                                         |
|------------------------|-------------------------------------------------|
| `REGMAP_SPI`           | regmap SPI bus abstraction                      |
| `IIO_BUFFER`           | Buffer support for continuous position capture   |
| `IIO_TRIGGERED_BUFFER` | Triggered buffer infrastructure                  |
| `GPIOLIB`              | Reset/mode GPIO pins                             |

---

## 5. Makefile

Add the entry to `drivers/iio/position/Makefile` in **alphabetical order**.

```makefile
obj-$(CONFIG_ADXXXX) += adxxxx.o
```

For multi-file drivers:

```makefile
obj-$(CONFIG_ADXXXX) += adxxxx.o
adxxxx-y := adxxxx-core.o adxxxx-calibration.o
```

---

## 6. Driver Source (.c)

### Complete Skeleton (SPI Angular Position Sensor)

```c
// SPDX-License-Identifier: GPL-2.0+
/*
 * ADXXXX SPI angular position sensor driver
 *
 * Copyright 2024 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

/* ------------------------------------------------------------------ */
/* Register Definitions                                                */
/* ------------------------------------------------------------------ */

#define ADXXXX_REG_POSITION		0x00
#define ADXXXX_REG_VELOCITY		0x01
#define ADXXXX_REG_TURNS		0x02
#define ADXXXX_REG_CONTROL		0x03
#define ADXXXX_REG_ZERO_OFFSET		0x04
#define ADXXXX_REG_STATUS		0x05
#define ADXXXX_REG_CALIBRATION		0x06

#define ADXXXX_CONTROL_MODE		GENMASK(1, 0)
#define ADXXXX_CONTROL_RES		GENMASK(5, 4)
#define ADXXXX_STATUS_READY		BIT(0)

#define ADXXXX_MODE_ABSOLUTE		0
#define ADXXXX_MODE_INCREMENTAL		1

/* ------------------------------------------------------------------ */
/* Device State                                                        */
/* ------------------------------------------------------------------ */

struct adxxxx_chip_info {
	const char		*name;
	unsigned int		chip_id;
	unsigned int		max_resolution;
	bool			has_multiturn;
};

struct adxxxx_state {
	struct regmap		*regmap;
	const struct adxxxx_chip_info *chip_info;
	struct gpio_desc	*reset_gpio;
	struct mutex		lock;	/* Protect device state and bus */
	unsigned int		resolution;
	unsigned int		mode;
	u16			zero_offset;
};

/* ------------------------------------------------------------------ */
/* Hardware Helpers                                                     */
/* ------------------------------------------------------------------ */

static int adxxxx_hw_reset(struct adxxxx_state *st)
{
	if (!st->reset_gpio)
		return 0;

	gpiod_set_value_cansleep(st->reset_gpio, 1);
	fsleep(100);
	gpiod_set_value_cansleep(st->reset_gpio, 0);
	fsleep(1000);

	return 0;
}

static int adxxxx_set_mode(struct adxxxx_state *st, unsigned int mode)
{
	return regmap_update_bits(st->regmap, ADXXXX_REG_CONTROL,
				  ADXXXX_CONTROL_MODE,
				  FIELD_PREP(ADXXXX_CONTROL_MODE, mode));
}

/* ------------------------------------------------------------------ */
/* IIO Channel Specification                                           */
/* ------------------------------------------------------------------ */

/*
 * Angular position channel.
 * .type = IIO_ANGL:  sysfs exposes in_angl<N>_raw, in_angl<N>_scale, etc.
 * For multi-turn encoders, add a second channel for the turn count.
 */
#define ADXXXX_ANGL_CHANNEL(_idx) {					\
	.type = IIO_ANGL,						\
	.indexed = 1,							\
	.channel = (_idx),						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_SCALE) |			\
			      BIT(IIO_CHAN_INFO_OFFSET),			\
	.scan_index = (_idx),						\
	.scan_type = {							\
		.sign = 'u',						\
		.realbits = 16,						\
		.storagebits = 16,					\
		.endianness = IIO_BE,					\
	},								\
}

/*
 * Turn-count channel for multi-turn encoders.
 * Reports the number of complete revolutions.
 */
#define ADXXXX_TURNS_CHANNEL(_idx) {					\
	.type = IIO_ANGL,						\
	.indexed = 1,							\
	.channel = (_idx),						\
	.extend_name = "turns",						\
	.address = ADXXXX_REG_TURNS,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
	.scan_index = (_idx),						\
	.scan_type = {							\
		.sign = 'u',						\
		.realbits = 16,						\
		.storagebits = 16,					\
		.endianness = IIO_BE,					\
	},								\
}

static const struct iio_chan_spec adxxxx_channels[] = {
	ADXXXX_ANGL_CHANNEL(0),
};

static const struct iio_chan_spec adxxxx_multiturn_channels[] = {
	ADXXXX_ANGL_CHANNEL(0),
	ADXXXX_TURNS_CHANNEL(1),
};

/* ------------------------------------------------------------------ */
/* ext_info for Calibration (see Section 7)                            */
/* ------------------------------------------------------------------ */

static ssize_t adxxxx_calibrate_store(struct iio_dev *indio_dev,
				      uintptr_t private,
				      struct iio_chan_spec const *chan,
				      const char *buf, size_t len);
static ssize_t adxxxx_zero_offset_show(struct iio_dev *indio_dev,
				       uintptr_t private,
				       struct iio_chan_spec const *chan,
				       char *buf);
static ssize_t adxxxx_zero_offset_store(struct iio_dev *indio_dev,
					uintptr_t private,
					struct iio_chan_spec const *chan,
					const char *buf, size_t len);

static const struct iio_chan_spec_ext_info adxxxx_ext_info[] = {
	{
		.name = "calibrate",
		.write = adxxxx_calibrate_store,
		.shared = IIO_SEPARATE,
	},
	{
		.name = "zero_offset",
		.read = adxxxx_zero_offset_show,
		.write = adxxxx_zero_offset_store,
		.shared = IIO_SEPARATE,
	},
	{ }
};

/* ------------------------------------------------------------------ */
/* read_raw / write_raw Callbacks                                      */
/* ------------------------------------------------------------------ */

static int adxxxx_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long info)
{
	struct adxxxx_state *st = iio_priv(indio_dev);
	unsigned int regval;
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		if (!iio_device_claim_direct(indio_dev))
			return -EBUSY;

		mutex_lock(&st->lock);
		ret = regmap_read(st->regmap,
				  chan->address ? chan->address :
				  ADXXXX_REG_POSITION,
				  &regval);
		mutex_unlock(&st->lock);

		iio_device_release_direct(indio_dev);

		if (ret)
			return ret;

		*val = regval;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		/*
		 * Angular position scale: convert raw counts to radians.
		 * Full scale = 2*pi radians = 2^resolution counts.
		 *
		 *   scale = 2*pi / 2^resolution
		 *
		 * Use IIO_VAL_FRACTIONAL:
		 *   val = numerator (2 * pi * 1000000)
		 *   val2 = denominator (2^resolution * 1000000)
		 *
		 * Simpler: IIO_VAL_INT_PLUS_NANO for small values.
		 *
		 * The iqs624 driver uses: (pi/180) * divisor as
		 * IIO_VAL_FRACTIONAL since raw is in degrees.
		 *
		 * Adapt to your device's raw output unit.
		 */
		*val = 628318;	/* 2 * pi * 100000 */
		*val2 = (1 << st->resolution) * 100000;
		return IIO_VAL_FRACTIONAL;

	case IIO_CHAN_INFO_OFFSET:
		mutex_lock(&st->lock);
		*val = st->zero_offset;
		mutex_unlock(&st->lock);
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

static int adxxxx_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long info)
{
	struct adxxxx_state *st = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_OFFSET:
		if (val < 0 || val >= (1 << st->resolution))
			return -EINVAL;

		guard(mutex)(&st->lock);

		st->zero_offset = val;
		return regmap_write(st->regmap, ADXXXX_REG_ZERO_OFFSET, val);

	default:
		return -EINVAL;
	}
}

/* ------------------------------------------------------------------ */
/* debugfs Register Access                                             */
/* ------------------------------------------------------------------ */

static int adxxxx_reg_access(struct iio_dev *indio_dev,
			     unsigned int reg, unsigned int writeval,
			     unsigned int *readval)
{
	struct adxxxx_state *st = iio_priv(indio_dev);

	if (readval)
		return regmap_read(st->regmap, reg, readval);

	return regmap_write(st->regmap, reg, writeval);
}

/* ------------------------------------------------------------------ */
/* iio_info                                                            */
/* ------------------------------------------------------------------ */

static const struct iio_info adxxxx_info = {
	.read_raw = adxxxx_read_raw,
	.write_raw = adxxxx_write_raw,
	.debugfs_reg_access = adxxxx_reg_access,
};

/* ------------------------------------------------------------------ */
/* regmap Configuration                                                */
/* ------------------------------------------------------------------ */

static const struct regmap_config adxxxx_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = ADXXXX_REG_CALIBRATION,
};

/* ------------------------------------------------------------------ */
/* Probe                                                               */
/* ------------------------------------------------------------------ */

static int adxxxx_probe(struct spi_device *spi)
{
	const struct adxxxx_chip_info *info;
	struct device *dev = &spi->dev;
	struct adxxxx_state *st;
	struct iio_dev *indio_dev;
	u32 resolution;
	int ret;

	info = spi_get_device_match_data(spi);
	if (!info)
		return dev_err_probe(dev, -ENODEV,
				     "Failed to get match data\n");

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->chip_info = info;

	ret = devm_mutex_init(dev, &st->lock);
	if (ret)
		return ret;

	st->regmap = devm_regmap_init_spi(spi, &adxxxx_regmap_config);
	if (IS_ERR(st->regmap))
		return dev_err_probe(dev, PTR_ERR(st->regmap),
				     "Failed to initialize regmap\n");

	/* Power supply */
	ret = devm_regulator_get_enable(dev, "vdd");
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to enable vdd supply\n");

	/* Optional hardware reset GPIO */
	st->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						  GPIOD_OUT_LOW);
	if (IS_ERR(st->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(st->reset_gpio),
				     "Failed to get reset GPIO\n");

	adxxxx_hw_reset(st);

	/* Parse DT properties (see Section 8) */
	ret = device_property_read_u32(dev, "adi,resolution", &resolution);
	if (ret)
		resolution = info->max_resolution;

	if (resolution > info->max_resolution)
		return dev_err_probe(dev, -EINVAL,
				     "Resolution %u exceeds max %u\n",
				     resolution, info->max_resolution);

	st->resolution = resolution;

	/* Parse operating mode */
	st->mode = ADXXXX_MODE_ABSOLUTE;
	if (device_property_match_string(dev, "adi,mode", "incremental") >= 0)
		st->mode = ADXXXX_MODE_INCREMENTAL;

	ret = adxxxx_set_mode(st, st->mode);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to set operating mode\n");

	/* IIO device setup */
	indio_dev->name = info->name;
	indio_dev->info = &adxxxx_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	if (info->has_multiturn) {
		indio_dev->channels = adxxxx_multiturn_channels;
		indio_dev->num_channels =
			ARRAY_SIZE(adxxxx_multiturn_channels);
	} else {
		indio_dev->channels = adxxxx_channels;
		indio_dev->num_channels = ARRAY_SIZE(adxxxx_channels);
	}

	return devm_iio_device_register(dev, indio_dev);
}

/* ------------------------------------------------------------------ */
/* Match Tables                                                        */
/* ------------------------------------------------------------------ */

static const struct adxxxx_chip_info adxxxx_chip_info = {
	.name = "adxxxx",
	.chip_id = 0x00,
	.max_resolution = 16,
	.has_multiturn = false,
};

static const struct of_device_id adxxxx_of_match[] = {
	{ .compatible = "adi,adxxxx", .data = &adxxxx_chip_info },
	{ }
};
MODULE_DEVICE_TABLE(of, adxxxx_of_match);

static const struct spi_device_id adxxxx_ids[] = {
	{ "adxxxx", (kernel_ulong_t)&adxxxx_chip_info },
	{ }
};
MODULE_DEVICE_TABLE(spi, adxxxx_ids);

static struct spi_driver adxxxx_driver = {
	.driver = {
		.name = "adxxxx",
		.of_match_table = adxxxx_of_match,
	},
	.probe = adxxxx_probe,
	.id_table = adxxxx_ids,
};
module_spi_driver(adxxxx_driver);

MODULE_AUTHOR("First Last <first.last@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADXXXX position sensor driver");
MODULE_LICENSE("GPL");
```

### Key Position-Specific Patterns

**Absolute vs Incremental mode**: Absolute encoders report the current
position directly. Incremental encoders report pulses from a reference
point. Some devices support both modes selected via a register or DT
property.

**Multi-turn support**: Devices like multi-turn absolute encoders provide
a separate turn-count register alongside the within-revolution position.
Model this as a second `IIO_ANGL` channel with `extend_name = "turns"`,
or use `IIO_POSITION` for the combined multi-turn value.

**Angular velocity**: Some position sensors also report velocity. If
needed, add an `IIO_ANGL_VEL` channel alongside the `IIO_ANGL` channel,
following the same pattern as resolver drivers.

---

## 7. Calibration

Position sensors typically require zero-position calibration. The
standard approach uses `ext_info` to expose calibration commands and
zero-offset values through custom sysfs attributes.

### ext_info for Calibration Commands

```c
/*
 * Writing "1" triggers a zero-position calibration: the current
 * position is latched as the new zero reference.
 */
static ssize_t adxxxx_calibrate_store(struct iio_dev *indio_dev,
				      uintptr_t private,
				      struct iio_chan_spec const *chan,
				      const char *buf, size_t len)
{
	struct adxxxx_state *st = iio_priv(indio_dev);
	bool calibrate;
	int ret;

	ret = kstrtobool(buf, &calibrate);
	if (ret)
		return ret;

	if (!calibrate)
		return len;

	guard(mutex)(&st->lock);

	/* Trigger hardware calibration sequence */
	ret = regmap_write(st->regmap, ADXXXX_REG_CALIBRATION, 0x01);
	if (ret)
		return ret;

	/* Read back the new zero offset */
	ret = regmap_read(st->regmap, ADXXXX_REG_ZERO_OFFSET,
			  (unsigned int *)&st->zero_offset);
	if (ret)
		return ret;

	return len;
}

/*
 * Read the current zero-position offset.
 */
static ssize_t adxxxx_zero_offset_show(struct iio_dev *indio_dev,
				       uintptr_t private,
				       struct iio_chan_spec const *chan,
				       char *buf)
{
	struct adxxxx_state *st = iio_priv(indio_dev);
	u16 offset;

	guard(mutex)(&st->lock);
	offset = st->zero_offset;

	return sysfs_emit(buf, "%u\n", offset);
}

/*
 * Write a specific zero-position offset value.
 */
static ssize_t adxxxx_zero_offset_store(struct iio_dev *indio_dev,
					uintptr_t private,
					struct iio_chan_spec const *chan,
					const char *buf, size_t len)
{
	struct adxxxx_state *st = iio_priv(indio_dev);
	u16 offset;
	int ret;

	ret = kstrtou16(buf, 0, &offset);
	if (ret)
		return ret;

	guard(mutex)(&st->lock);

	ret = regmap_write(st->regmap, ADXXXX_REG_ZERO_OFFSET, offset);
	if (ret)
		return ret;

	st->zero_offset = offset;

	return len;
}
```

### Attaching ext_info to a Channel

To add calibration attributes to a channel definition, set the
`.ext_info` field in the `iio_chan_spec`:

```c
#define ADXXXX_ANGL_CHANNEL_CAL(_idx) {					\
	.type = IIO_ANGL,						\
	.indexed = 1,							\
	.channel = (_idx),						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_SCALE) |			\
			      BIT(IIO_CHAN_INFO_OFFSET),			\
	.ext_info = adxxxx_ext_info,					\
	.scan_index = (_idx),						\
	.scan_type = {							\
		.sign = 'u',						\
		.realbits = 16,						\
		.storagebits = 16,					\
		.endianness = IIO_BE,					\
	},								\
}
```

This exposes the following sysfs entries per channel:

```
/sys/bus/iio/devices/iio:device0/in_angl0_calibrate       (write-only)
/sys/bus/iio/devices/iio:device0/in_angl0_zero_offset      (read/write)
```

### Alternative: OFFSET via info_mask

For simple zero-point adjustment without a hardware calibration command,
the `IIO_CHAN_INFO_OFFSET` in `info_mask_separate` suffices. Use `ext_info`
only when you need a trigger-style calibration action or additional
calibration parameters beyond a single offset value.

---

## 8. DT Parsing

### Resolution and Operating Mode

```c
static int adxxxx_parse_dt(struct adxxxx_state *st, struct device *dev)
{
	u32 resolution;
	int ret;

	/* Resolution: number of bits per revolution / full-scale */
	ret = device_property_read_u32(dev, "adi,resolution", &resolution);
	if (ret)
		resolution = st->chip_info->max_resolution;

	if (resolution > st->chip_info->max_resolution)
		return dev_err_probe(dev, -EINVAL,
				     "Resolution %u exceeds device max %u\n",
				     resolution,
				     st->chip_info->max_resolution);

	st->resolution = resolution;

	/* Operating mode: absolute (default) or incremental */
	st->mode = ADXXXX_MODE_ABSOLUTE;
	if (device_property_match_string(dev, "adi,mode",
					 "incremental") >= 0)
		st->mode = ADXXXX_MODE_INCREMENTAL;

	return 0;
}
```

### Common DT Properties for Position Sensors

| Property           | API                                     | Purpose                          |
|--------------------|-----------------------------------------|----------------------------------|
| `adi,resolution`   | `device_property_read_u32()`           | Converter resolution in bits     |
| `adi,mode`         | `device_property_match_string()`       | Absolute / incremental mode      |
| `reset-gpios`      | `devm_gpiod_get_optional()`            | Hardware reset pin               |
| `vdd-supply`       | `devm_regulator_get_enable()`          | Main power supply                |
| `clocks`           | `devm_clk_get_optional_enabled()`      | External clock input             |

---

## 9. Test & Debug

### sysfs Interface

Position sensors expose channels through sysfs:

```
/sys/bus/iio/devices/iio:device0/
    name                         # Device name (e.g. "adxxxx")
    in_angl0_raw                 # Raw position count
    in_angl0_scale               # Scale factor (raw * scale = radians)
    in_angl0_offset              # Zero-position offset
    in_angl0_calibrate           # Trigger calibration (ext_info)
    in_angl0_zero_offset         # Read/write zero offset (ext_info)
    in_angl1_turns_raw           # Turn count (multi-turn only)
```

### Reading and Converting Position

```sh
# Read raw angle count
cat /sys/bus/iio/devices/iio:device0/in_angl0_raw
# Example output: 16384

# Read scale factor (radians per count)
cat /sys/bus/iio/devices/iio:device0/in_angl0_scale
# Example output: 0.000095873  (2*pi / 2^16)

# Compute angle in radians
#   angle_rad = raw * scale = 16384 * 0.000095873 = 1.5708 rad

# Convert to degrees
#   angle_deg = angle_rad * (180 / pi) = 1.5708 * 57.2958 = 90.0 deg
```

### Calibration from Userspace

```sh
# Trigger zero-position calibration (sets current position as zero)
echo 1 > /sys/bus/iio/devices/iio:device0/in_angl0_calibrate

# Read back the calibration offset
cat /sys/bus/iio/devices/iio:device0/in_angl0_zero_offset

# Set a specific zero offset
echo 8192 > /sys/bus/iio/devices/iio:device0/in_angl0_zero_offset
```

### debugfs Register Access

```sh
# Read register 0x00 (position)
echo 0x00 > /sys/kernel/debug/iio/iio:device0/direct_reg_access
cat /sys/kernel/debug/iio/iio:device0/direct_reg_access

# Write 0x0000 to register 0x04 (zero offset)
echo 0x04 0x0000 > /sys/kernel/debug/iio/iio:device0/direct_reg_access
```

### IIO Userspace Tools

```sh
# List all IIO devices and channels
iio_info

# Read a single attribute
iio_attr -d iio:device0 -c in_angl0_raw

# For buffered capture (if triggered buffer is set up)
iio_readdev -b 256 -s 1024 iio:device0
```

---

## 10. Key Conventions

### License

All new ADI IIO drivers must use GPL-2.0 or GPL-2.0+:

```c
// SPDX-License-Identifier: GPL-2.0+
```

The SPDX tag goes on the very first line. `MODULE_LICENSE("GPL")` at the
bottom must match.

### Radians vs Degrees

The IIO subsystem convention for `IIO_ANGL` channels is **radians**. The
`scale` attribute must convert raw counts to radians. Userspace is
responsible for converting to degrees if needed.

The iqs624-pos driver illustrates this: it computes scale as
`(pi / 180) * divisor` to convert degree-based raw readings to radians.

If the hardware natively reports degrees (common for many ADI position
encoders), apply the conversion factor in the scale:

```c
/*
 * Hardware reports degrees directly.
 * Scale to radians: raw_degrees * (pi / 180)
 *
 * pi/180 = 0.0174533
 * As IIO_VAL_INT_PLUS_NANO: val=0, val2=17453293
 */
*val = 0;
*val2 = 17453293;
return IIO_VAL_INT_PLUS_NANO;
```

If the hardware reports raw counts with full-scale = 2^N = one revolution:

```c
/*
 * Scale to radians: raw * (2*pi / 2^resolution)
 * As IIO_VAL_FRACTIONAL: val = 2*pi*SCALE, val2 = 2^resolution*SCALE
 */
*val = 628318;   /* 2 * pi * 100000 */
*val2 = (1 << st->resolution) * 100000;
return IIO_VAL_FRACTIONAL;
```

### Memory Management

Use `devm_*` (device-managed) allocations exclusively:

| Function                             | Purpose                              |
|--------------------------------------|--------------------------------------|
| `devm_iio_device_alloc()`           | Allocate IIO device + private data   |
| `devm_iio_device_register()`        | Register IIO device (auto-unregister)|
| `devm_regmap_init_spi()`            | Initialize SPI regmap                |
| `devm_regulator_get_enable()`       | Get and enable a regulator           |
| `devm_gpiod_get_optional()`         | Get an optional GPIO descriptor      |
| `devm_mutex_init()`                 | Initialize a mutex with devm cleanup |
| `devm_clk_get_optional_enabled()`   | Get and enable an optional clock     |

### Coding Style

- Follow the kernel coding style (`Documentation/process/coding-style.rst`).
- Run `scripts/checkpatch.pl` before submitting.
- Use `FIELD_GET()` / `FIELD_PREP()` with `GENMASK()` for register bitfields.
- Use `BIT()` for single-bit definitions.
- Use `dev_err_probe()` for all error returns in `probe()`.
- Include headers in alphabetical order within each group.
- Use `guard(mutex)(&st->lock)` for scoped locking where possible.

---

## 11. Commit Format

### Subject Line Prefix

```
iio: position: <devname>: <brief description>
```

### Examples

```
iio: position: adxxxx: add support for ADXXXX angular position sensor
iio: position: adxxxx: add multi-turn tracking support
iio: position: adxxxx: fix zero-offset calibration race condition
```

### DT Binding Commit

```
dt-bindings: iio: position: add adi,adxxxx.yaml
```

### Patch Series for a New Position Driver

A typical new driver submission is a patch series:

1. `dt-bindings: iio: position: add adi,<devname>.yaml` -- DT binding
2. `iio: position: <devname>: add support for <DEVNAME>` -- Driver source
3. `MAINTAINERS: add entry for <DEVNAME> IIO driver` -- Maintainer entry

### Full Commit Message Example

```
iio: position: adxxxx: add support for ADXXXX angular position sensor

The ADXXXX is a 16-bit angular position sensor with SPI interface.
It supports absolute and incremental operating modes with
configurable resolution up to 16 bits per revolution.

This driver supports:
  - Absolute and incremental position measurement
  - Configurable resolution via devicetree
  - Zero-position calibration via ext_info sysfs
  - Multi-turn revolution counting (ADXXXX-MT variant)
  - Hardware reset via GPIO

Signed-off-by: First Last <first.last@analog.com>
```
