# Linux IIO DAC Driver Template

This template covers the structure, code patterns, and conventions for writing
a Linux kernel IIO driver for an Analog Devices voltage-output DAC. It builds
on the consolidated IIO template (`drivers/iio/DRIVER_TEMPLATE.md`) and adds
DAC-specific guidance such as output channels, powerdown support, and
vref-supply handling.

---

## 1. Purpose

An IIO DAC driver exposes one or more **output** voltage channels through the
IIO subsystem. Each channel uses:

- **Channel type**: `IIO_VOLTAGE`
- **Direction**: `.output = 1` in the `iio_chan_spec`
- **`info_mask` bits**:
  - `IIO_CHAN_INFO_RAW` (separate) -- the DAC code written to the device
  - `IIO_CHAN_INFO_SCALE` (shared by type or separate) -- derived from Vref
  - `IIO_CHAN_INFO_OFFSET` (shared by type, optional) -- for bipolar ranges

Unlike ADC drivers, DAC drivers typically do **not** use triggered buffers or
`scan_index`/`scan_type` for continuous capture. The primary interface is
direct sysfs writes (`out_voltageX_raw`).

Most ADI DAC parts also expose **powerdown** control through `ext_info`
attributes (`out_voltageX_powerdown`, `out_voltageX_powerdown_mode`).

---

## 2. File Checklist

| File                                                                   | Action   | Required |
|------------------------------------------------------------------------|----------|----------|
| `drivers/iio/dac/<devname>.c`                                          | Create   | Yes      |
| `drivers/iio/dac/Kconfig`                                              | Modify   | Yes      |
| `drivers/iio/dac/Makefile`                                             | Modify   | Yes      |
| `Documentation/devicetree/bindings/iio/dac/adi,<devname>.yaml`         | Create   | Yes      |
| `drivers/iio/dac/<devname>.h`                                          | Create   | Optional |

### Notes

- A header file is only needed when the driver is split across multiple source
  files (e.g. separate SPI and I2C bus glue like `ad5686-spi.c` /
  `ad5696-i2c.c` sharing `ad5686.c`).
- For simple single-bus DACs, everything goes in a single `.c` file.

---

## 3. DT Binding

Bindings live under `Documentation/devicetree/bindings/iio/dac/adi,<devname>.yaml`.

### SPI Bus DAC Example

```yaml
# SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)
%YAML 1.2
---
$id: http://devicetree.org/schemas/iio/dac/adi,adxxxx.yaml#
$schema: http://devicetree.org/meta-schemas/core.yaml#

title: Analog Devices ADXXXX DAC

maintainers:
  - First Last <first.last@analog.com>

description: |
  The ADXXXX is a low power, 16-bit, buffered voltage output DAC with an
  SPI-compatible serial interface. It operates from a single 2.7V to 5.5V
  supply and is monotonic by design.

properties:
  compatible:
    enum:
      - adi,adxxxx

  reg:
    maxItems: 1

  spi-max-frequency:
    maximum: 50000000

  vdd-supply:
    description: Analog power supply.

  iovdd-supply:
    description: Digital I/O power supply.

  vref-supply:
    description:
      External reference voltage. Sets the full-scale output range. If not
      provided, the internal reference is used (on "R" variants only).

  reset-gpios:
    description: Active-low hardware reset.
    maxItems: 1

  ldac-gpios:
    description:
      LDAC pin used as a hardware trigger to update the DAC output. If not
      present, software LDAC is used instead.
    maxItems: 1

required:
  - compatible
  - reg
  - vdd-supply

allOf:
  - $ref: /schemas/spi/spi-peripheral-props.yaml#

unevaluatedProperties: false

examples:
  - |
    spi {
        #address-cells = <1>;
        #size-cells = <0>;
        dac@0 {
            compatible = "adi,adxxxx";
            reg = <0>;
            spi-max-frequency = <10000000>;
            vdd-supply = <&vdd_3v3>;
            vref-supply = <&dac_vref>;
        };
    };
```

### I2C Bus DAC Example

```yaml
examples:
  - |
    i2c {
        #address-cells = <1>;
        #size-cells = <0>;
        dac@c {
            compatible = "adi,adxxxx";
            reg = <0x0c>;
            vdd-supply = <&vdd_3v3>;
            vref-supply = <&dac_vref>;
        };
    };
```

### Common DAC Binding Properties

| Property          | Purpose                                             |
|-------------------|-----------------------------------------------------|
| `vref-supply`     | External reference voltage regulator                |
| `vdd-supply`      | Analog power supply                                 |
| `iovdd-supply`    | Digital I/O power supply                            |
| `reset-gpios`     | Hardware reset pin (active-low)                     |
| `ldac-gpios`      | Load-DAC trigger pin                                |
| `adi,range-double`| Boolean; doubles the output span (0..2*Vref)        |

---

## 4. Kconfig

Add the entry to `drivers/iio/dac/Kconfig` in **alphabetical order**.

### SPI DAC

```kconfig
config ADXXXX
	tristate "Analog Devices ADXXXX DAC driver"
	depends on SPI_MASTER
	select REGMAP_SPI
	help
	  Say yes here to build support for Analog Devices ADXXXX
	  digital-to-analog converter.

	  To compile this driver as a module, choose M here: the module will be
	  called adxxxx.
```

### I2C DAC

```kconfig
config ADXXXX
	tristate "Analog Devices ADXXXX DAC driver"
	depends on I2C
	select REGMAP_I2C
	help
	  Say yes here to build support for Analog Devices ADXXXX
	  digital-to-analog converter.

	  To compile this driver as a module, choose M here: the module will be
	  called adxxxx.
```

### Common Optional Selects for DACs

| Select            | Purpose                              |
|-------------------|--------------------------------------|
| `REGMAP_SPI`      | regmap SPI bus abstraction           |
| `REGMAP_I2C`      | regmap I2C bus abstraction           |
| `GPIOLIB`         | GPIO usage (reset, LDAC)             |

DAC drivers typically do **not** need `IIO_BUFFER`, `IIO_TRIGGERED_BUFFER`, or
`AD_SIGMA_DELTA`.

---

## 5. Makefile

Add the entry to `drivers/iio/dac/Makefile` in **alphabetical order**.

```makefile
obj-$(CONFIG_ADXXXX) += adxxxx.o
```

For multi-file drivers (e.g. shared core with SPI/I2C bus glue):

```makefile
obj-$(CONFIG_AD5686)        += ad5686.o
obj-$(CONFIG_AD5686_SPI)    += ad5686-spi.o
obj-$(CONFIG_AD5696_I2C)    += ad5696-i2c.o
```

---

## 6. Driver Source

### Complete DAC Skeleton (SPI, Modern Style)

```c
// SPDX-License-Identifier: GPL-2.0
/*
 * ADXXXX Voltage Output DAC driver
 *
 * Copyright 2025 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/kstrtox.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/sysfs.h>

/* ------------------------------------------------------------------ */
/* Register Definitions                                                */
/* ------------------------------------------------------------------ */

#define ADXXXX_REG_DAC_DATA(ch)		(0x10 + (ch))
#define ADXXXX_REG_CONTROL		0x01
#define ADXXXX_REG_POWERDOWN		0x04

#define ADXXXX_PD_MODE_MASK(ch)		(GENMASK(1, 0) << (2 * (ch)))
#define ADXXXX_DAC_MAX_VAL		GENMASK(15, 0)

/* ------------------------------------------------------------------ */
/* Powerdown Mode Enum                                                 */
/* ------------------------------------------------------------------ */

enum adxxxx_powerdown_mode {
	ADXXXX_PD_1K_TO_GND = 1,
	ADXXXX_PD_100K_TO_GND,
	ADXXXX_PD_THREE_STATE,
};

/* ------------------------------------------------------------------ */
/* Device State                                                        */
/* ------------------------------------------------------------------ */

struct adxxxx_chip_info {
	const char		*name;
	unsigned int		num_channels;
	unsigned int		resolution;
	bool			has_internal_ref;
};

struct adxxxx_chan {
	enum adxxxx_powerdown_mode	powerdown_mode;
	bool				powerdown;
};

struct adxxxx_state {
	struct regmap		*regmap;
	/* lock to protect against concurrent access to device and state */
	struct mutex		lock;
	const struct adxxxx_chip_info *chip_info;
	struct adxxxx_chan	chan[8];
	int			vref_mv;
};

/* ------------------------------------------------------------------ */
/* Powerdown Support                                                   */
/* ------------------------------------------------------------------ */

/* See Section 7 below for full powerdown pattern explanation. */

static const char * const adxxxx_powerdown_modes[] = {
	"1kohm_to_gnd",
	"100kohm_to_gnd",
	"three_state",
};

static int adxxxx_get_powerdown_mode(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan)
{
	struct adxxxx_state *st = iio_priv(indio_dev);

	guard(mutex)(&st->lock);
	return st->chan[chan->channel].powerdown_mode - 1;
}

static int adxxxx_set_powerdown_mode(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     unsigned int mode)
{
	struct adxxxx_state *st = iio_priv(indio_dev);

	guard(mutex)(&st->lock);
	st->chan[chan->channel].powerdown_mode = mode + 1;

	return 0;
}

static const struct iio_enum adxxxx_powerdown_mode_enum = {
	.items = adxxxx_powerdown_modes,
	.num_items = ARRAY_SIZE(adxxxx_powerdown_modes),
	.get = adxxxx_get_powerdown_mode,
	.set = adxxxx_set_powerdown_mode,
};

static ssize_t adxxxx_read_powerdown(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     char *buf)
{
	struct adxxxx_state *st = iio_priv(indio_dev);

	guard(mutex)(&st->lock);
	return sysfs_emit(buf, "%d\n", st->chan[chan->channel].powerdown);
}

static ssize_t adxxxx_write_powerdown(struct iio_dev *indio_dev,
				      uintptr_t private,
				      const struct iio_chan_spec *chan,
				      const char *buf, size_t len)
{
	struct adxxxx_state *st = iio_priv(indio_dev);
	unsigned int mask, val, mode;
	bool powerdown;
	int ret;

	ret = kstrtobool(buf, &powerdown);
	if (ret)
		return ret;

	guard(mutex)(&st->lock);
	mode = powerdown ? st->chan[chan->channel].powerdown_mode : 0;
	mask = ADXXXX_PD_MODE_MASK(chan->channel);
	val = FIELD_PREP(mask, mode);

	ret = regmap_update_bits(st->regmap, ADXXXX_REG_POWERDOWN, mask, val);
	if (ret)
		return ret;

	st->chan[chan->channel].powerdown = powerdown;

	return len;
}

static const struct iio_chan_spec_ext_info adxxxx_ext_info[] = {
	{
		.name = "powerdown",
		.shared = IIO_SEPARATE,
		.read = adxxxx_read_powerdown,
		.write = adxxxx_write_powerdown,
	},
	IIO_ENUM("powerdown_mode", IIO_SEPARATE,
		 &adxxxx_powerdown_mode_enum),
	IIO_ENUM_AVAILABLE("powerdown_mode", IIO_SHARED_BY_TYPE,
			   &adxxxx_powerdown_mode_enum),
	{ }
};

/* ------------------------------------------------------------------ */
/* IIO Channel Specification                                           */
/* ------------------------------------------------------------------ */

/*
 * DAC channels MUST set .output = 1.  This causes sysfs entries to
 * appear as out_voltageX_raw (not in_voltageX_raw).
 *
 * Buffers and scan_type are usually not needed for DACs.
 */
#define ADXXXX_DAC_CHANNEL(_chan) {				\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.output = 1,						\
	.channel = (_chan),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
			      BIT(IIO_CHAN_INFO_SCALE),		\
	.ext_info = adxxxx_ext_info,				\
}

static const struct iio_chan_spec adxxxx_channels[] = {
	ADXXXX_DAC_CHANNEL(0),
	ADXXXX_DAC_CHANNEL(1),
	ADXXXX_DAC_CHANNEL(2),
	ADXXXX_DAC_CHANNEL(3),
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
		/*
		 * For DACs, read_raw on RAW typically reads back the
		 * value cached in the input register (not an ADC
		 * conversion).  Claim direct mode to prevent conflict
		 * with any buffer operation.
		 */
		if (!iio_device_claim_direct(indio_dev))
			return -EBUSY;

		guard(mutex)(&st->lock);
		ret = regmap_read(st->regmap,
				  ADXXXX_REG_DAC_DATA(chan->channel),
				  &regval);

		iio_device_release_direct(indio_dev);

		if (ret)
			return ret;

		*val = regval & ADXXXX_DAC_MAX_VAL;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		/*
		 * Scale = Vref_mV / 2^resolution
		 * Returned as IIO_VAL_FRACTIONAL_LOG2.
		 * val  = Vref in millivolts
		 * val2 = number of bits (exponent)
		 */
		*val = st->vref_mv;
		*val2 = st->chip_info->resolution;
		return IIO_VAL_FRACTIONAL_LOG2;

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
	case IIO_CHAN_INFO_RAW:
		if (val < 0 || val > ADXXXX_DAC_MAX_VAL)
			return -EINVAL;

		guard(mutex)(&st->lock);
		return regmap_write(st->regmap,
				    ADXXXX_REG_DAC_DATA(chan->channel), val);
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
/* Probe                                                               */
/* ------------------------------------------------------------------ */

static const struct regmap_config adxxxx_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0xFF,
};

static int adxxxx_probe(struct spi_device *spi)
{
	const struct adxxxx_chip_info *chip_info;
	struct device *dev = &spi->dev;
	struct adxxxx_state *st;
	struct iio_dev *indio_dev;
	int ret, vref_uv;

	chip_info = spi_get_device_match_data(spi);
	if (!chip_info)
		return dev_err_probe(dev, -ENODEV,
				     "Failed to get match data\n");

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->chip_info = chip_info;

	st->regmap = devm_regmap_init_spi(spi, &adxxxx_regmap_config);
	if (IS_ERR(st->regmap))
		return dev_err_probe(dev, PTR_ERR(st->regmap),
				     "Failed to init regmap\n");

	ret = devm_mutex_init(dev, &st->lock);
	if (ret)
		return ret;

	/* Enable power supply(ies) */
	ret = devm_regulator_get_enable(dev, "vdd");
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to enable vdd supply\n");

	/*
	 * Get external Vref.  devm_regulator_get_enable_read_voltage()
	 * returns -ENODEV when the supply is absent in the DT; in that
	 * case fall back to the internal reference if available.
	 */
	vref_uv = devm_regulator_get_enable_read_voltage(dev, "vref");
	if (vref_uv < 0 && vref_uv != -ENODEV)
		return dev_err_probe(dev, vref_uv,
				     "Failed to get vref supply\n");

	if (vref_uv > 0) {
		st->vref_mv = vref_uv / 1000;
	} else if (chip_info->has_internal_ref) {
		st->vref_mv = 2500;  /* Internal 2.5 V reference */
	} else {
		return dev_err_probe(dev, -ENODEV,
				     "No reference voltage available\n");
	}

	/* Hardware init: reset, verify chip ID, etc. */
	/* ... */

	indio_dev->name = chip_info->name;
	indio_dev->info = &adxxxx_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = adxxxx_channels;
	indio_dev->num_channels = chip_info->num_channels;

	return devm_iio_device_register(dev, indio_dev);
}

/* ------------------------------------------------------------------ */
/* Match Tables                                                        */
/* ------------------------------------------------------------------ */

static const struct adxxxx_chip_info adxxxx_chip = {
	.name = "adxxxx",
	.num_channels = 4,
	.resolution = 16,
	.has_internal_ref = true,
};

static const struct of_device_id adxxxx_of_match[] = {
	{ .compatible = "adi,adxxxx", .data = &adxxxx_chip },
	{ }
};
MODULE_DEVICE_TABLE(of, adxxxx_of_match);

static const struct spi_device_id adxxxx_ids[] = {
	{ "adxxxx", (kernel_ulong_t)&adxxxx_chip },
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
MODULE_DESCRIPTION("Analog Devices ADXXXX DAC driver");
MODULE_LICENSE("GPL");
```

### I2C DAC Variant

For I2C-based DACs, replace the SPI-specific sections:

```c
#include <linux/i2c.h>
#include <linux/regmap.h>

static const struct regmap_config adxxxx_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = 0xFF,
};

static int adxxxx_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct adxxxx_state *st;
	struct iio_dev *indio_dev;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->regmap = devm_regmap_init_i2c(client, &adxxxx_regmap_config);
	if (IS_ERR(st->regmap))
		return dev_err_probe(dev, PTR_ERR(st->regmap),
				     "Failed to init regmap\n");

	/* ... same init as SPI variant ... */

	return devm_iio_device_register(dev, indio_dev);
}

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
```

### Key DAC-Specific Driver Points

1. **`.output = 1`**: Every `iio_chan_spec` for a DAC channel must set
   `.output = 1`. This makes sysfs entries appear under `out_voltageX_*`
   instead of `in_voltageX_*`.

2. **`INDIO_DIRECT_MODE` only**: DACs rarely use `INDIO_BUFFER_TRIGGERED` or
   `INDIO_BUFFER_SOFTWARE`. The primary interface is direct sysfs read/write.

3. **`write_raw` for `IIO_CHAN_INFO_RAW`**: This is how userspace sets the
   DAC output code. Validate the range (`0` to `2^resolution - 1`), then
   write the value to the hardware data register.

4. **`read_raw` for `IIO_CHAN_INFO_RAW`**: Reads back the value currently
   latched in the DAC's input/data register. This is a register readback,
   not an analog measurement.

5. **`read_raw` for `IIO_CHAN_INFO_SCALE`**: Returns `Vref_mV / 2^resolution`
   using `IIO_VAL_FRACTIONAL_LOG2`.

6. **No `scan_type` needed** unless the driver supports triggered buffer
   output (which is rare).

---

## 7. Powerdown Support

Most ADI DAC parts support per-channel powerdown modes where the output is
disconnected from the DAC core and optionally connected to GND through a
resistor or left in a high-impedance state. The IIO framework exposes this
through `ext_info` attributes.

### Sysfs Attributes Created

For each output channel, powerdown support adds:

| Sysfs Attribute                                | Description                         |
|------------------------------------------------|-------------------------------------|
| `out_voltageX_powerdown`                       | `0` = active, `1` = powered down    |
| `out_voltageX_powerdown_mode`                  | Current mode (e.g. `1kohm_to_gnd`)  |
| `out_voltageX_powerdown_mode_available`         | List of supported modes             |

### Implementation Pattern

The powerdown implementation requires three components:

**1. Mode strings and `iio_enum`**:

```c
static const char * const adxxxx_powerdown_modes[] = {
	"1kohm_to_gnd",
	"100kohm_to_gnd",
	"three_state",
};

static const struct iio_enum adxxxx_powerdown_mode_enum = {
	.items = adxxxx_powerdown_modes,
	.num_items = ARRAY_SIZE(adxxxx_powerdown_modes),
	.get = adxxxx_get_powerdown_mode,
	.set = adxxxx_set_powerdown_mode,
};
```

Common powerdown mode strings used across ADI DAC drivers:
- `"1kohm_to_gnd"` -- output pulled to GND through 1 kOhm
- `"100kohm_to_gnd"` -- output pulled to GND through 100 kOhm
- `"three_state"` -- output is high-impedance (tri-state)
- `"7.7kohm_to_gnd"`, `"32kohm_to_gnd"` -- part-specific variants

**2. Powerdown read/write callbacks** (shown in the skeleton above):
- `adxxxx_read_powerdown()` -- returns `"0\n"` or `"1\n"` via `sysfs_emit()`
- `adxxxx_write_powerdown()` -- parses a boolean with `kstrtobool()`, then
  writes the appropriate mode bits to the hardware powerdown register

**3. `ext_info` array** attached to the channel spec:

```c
static const struct iio_chan_spec_ext_info adxxxx_ext_info[] = {
	{
		.name = "powerdown",
		.shared = IIO_SEPARATE,
		.read = adxxxx_read_powerdown,
		.write = adxxxx_write_powerdown,
	},
	IIO_ENUM("powerdown_mode", IIO_SEPARATE,
		 &adxxxx_powerdown_mode_enum),
	IIO_ENUM_AVAILABLE("powerdown_mode", IIO_SHARED_BY_TYPE,
			   &adxxxx_powerdown_mode_enum),
	{ }
};
```

### Per-Channel State

Store the powerdown state per channel in the driver state struct:

```c
struct adxxxx_chan {
	enum adxxxx_powerdown_mode	powerdown_mode;
	bool				powerdown;
};

struct adxxxx_state {
	/* ... */
	struct adxxxx_chan	chan[MAX_CHANNELS];
};
```

Initialize the default powerdown mode in `probe()` or a setup function:

```c
for (i = 0; i < chip_info->num_channels; i++)
	st->chan[i].powerdown_mode = ADXXXX_PD_1K_TO_GND;
```

---

## 8. DT Parsing

### Vref Supply

The reference voltage determines the full-scale output range. DAC drivers
typically handle it in one of two patterns:

**Pattern A -- `devm_regulator_get_enable_read_voltage()` (modern, preferred)**:

```c
vref_uv = devm_regulator_get_enable_read_voltage(dev, "vref");
if (vref_uv < 0 && vref_uv != -ENODEV)
	return dev_err_probe(dev, vref_uv, "Failed to get vref\n");

if (vref_uv > 0)
	st->vref_mv = vref_uv / 1000;
else if (chip_info->has_internal_ref)
	st->vref_mv = 2500;
else
	return dev_err_probe(dev, -ENODEV, "No vref available\n");
```

**Pattern B -- `devm_regulator_get_optional()` + manual enable (older)**:

```c
st->vref_reg = devm_regulator_get_optional(dev, "vref");
if (IS_ERR(st->vref_reg)) {
	if (PTR_ERR(st->vref_reg) != -ENODEV)
		return PTR_ERR(st->vref_reg);
	/* No external ref -- use internal */
	st->vref_mv = chip_info->int_vref_mv;
} else {
	ret = regulator_enable(st->vref_reg);
	if (ret)
		return ret;
	st->vref_mv = regulator_get_voltage(st->vref_reg) / 1000;
}
```

### Power Supplies

```c
/* Single supply */
ret = devm_regulator_get_enable(dev, "vdd");
if (ret)
	return dev_err_probe(dev, ret, "Failed to enable vdd\n");

/* Multiple supplies */
static const char * const regulators[] = { "vdd", "iovdd" };
ret = devm_regulator_bulk_get_enable(dev, ARRAY_SIZE(regulators),
				     regulators);
if (ret)
	return dev_err_probe(dev, ret, "Failed to enable regulators\n");
```

### Optional GPIOs (Reset, LDAC)

```c
/* Reset GPIO (active-low) */
struct gpio_desc *reset_gpio;
reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
if (IS_ERR(reset_gpio))
	return dev_err_probe(dev, PTR_ERR(reset_gpio),
			     "Failed to get reset GPIO\n");
if (reset_gpio) {
	fsleep(1000);  /* Hold reset for 1 ms */
	gpiod_set_value_cansleep(reset_gpio, 0);
	fsleep(10000); /* Wait for device to come out of reset */
}

/* LDAC GPIO */
st->ldac_gpio = devm_gpiod_get_optional(dev, "ldac", GPIOD_OUT_LOW);
if (IS_ERR(st->ldac_gpio))
	return dev_err_probe(dev, PTR_ERR(st->ldac_gpio),
			     "Failed to get ldac GPIO\n");
```

### Output Range Configuration

Some DACs support configurable output ranges via DT properties:

```c
if (device_property_read_bool(dev, "adi,range-double"))
	st->vref_mv *= 2;
```

---

## 9. Test & Debug

### Sysfs Interface (DAC-Specific)

DAC channels expose `out_voltage*` attributes (note the `out_` prefix):

```
/sys/bus/iio/devices/iio:device0/
    name                                     # Device name
    out_voltage0_raw                         # DAC code for channel 0
    out_voltage0_scale                       # Scale factor (mV/LSB)
    out_voltage1_raw                         # DAC code for channel 1
    out_voltage1_scale                       # Scale factor
    out_voltage0_powerdown                   # 0 = active, 1 = powered down
    out_voltage0_powerdown_mode              # e.g. "1kohm_to_gnd"
    out_voltage0_powerdown_mode_available    # List of modes
```

### Setting a DAC Output

```sh
# Set channel 0 to mid-scale on a 16-bit DAC
echo 32768 > /sys/bus/iio/devices/iio:device0/out_voltage0_raw

# Read back the current DAC code
cat /sys/bus/iio/devices/iio:device0/out_voltage0_raw

# Calculate the output voltage:
#   Vout = raw * scale = raw * (Vref_mV / 2^bits)
cat /sys/bus/iio/devices/iio:device0/out_voltage0_scale
```

### Powerdown Control

```sh
# Power down channel 0
echo 1 > /sys/bus/iio/devices/iio:device0/out_voltage0_powerdown

# Check current powerdown mode
cat /sys/bus/iio/devices/iio:device0/out_voltage0_powerdown_mode

# Change powerdown mode before powering down
echo three_state > /sys/bus/iio/devices/iio:device0/out_voltage0_powerdown_mode

# See available modes
cat /sys/bus/iio/devices/iio:device0/out_voltage0_powerdown_mode_available

# Re-enable the output
echo 0 > /sys/bus/iio/devices/iio:device0/out_voltage0_powerdown
```

### debugfs Register Access

```sh
# Read register 0x10
echo 0x10 > /sys/kernel/debug/iio/iio:device0/direct_reg_access
cat /sys/kernel/debug/iio/iio:device0/direct_reg_access

# Write 0x8000 to register 0x10
echo 0x10 0x8000 > /sys/kernel/debug/iio/iio:device0/direct_reg_access
```

### IIO Userspace Tools

```sh
# List all IIO devices and their channels
iio_info

# Read a specific attribute
iio_attr -d iio:device0 -c out_voltage0_raw
iio_attr -d iio:device0 -c out_voltage0_powerdown
```

---

## 10. Key Conventions

### Memory Management

Use `devm_*` (device-managed) allocations exclusively:

| Function                               | Purpose                             |
|----------------------------------------|-------------------------------------|
| `devm_iio_device_alloc()`             | Allocate IIO device + private data  |
| `devm_iio_device_register()`          | Register IIO device (auto-unregister)|
| `devm_regmap_init_spi()`              | Initialize SPI regmap               |
| `devm_regmap_init_i2c()`              | Initialize I2C regmap               |
| `devm_regulator_get_enable()`         | Get and enable a power regulator    |
| `devm_regulator_get_enable_read_voltage()` | Get, enable, and read Vref    |
| `devm_regulator_bulk_get_enable()`    | Enable multiple regulators at once  |
| `devm_mutex_init()`                   | Initialize a mutex with devm cleanup|
| `devm_gpiod_get_optional()`           | Get an optional GPIO descriptor     |

### License

All new ADI IIO drivers must use GPL-2.0 or GPL-2.0+:

```c
// SPDX-License-Identifier: GPL-2.0
```

The SPDX tag goes on the very first line. `MODULE_LICENSE("GPL")` at the
bottom must match. Do not use `"GPL v2"` in new drivers; use `"GPL"`.

### Vref Regulator

- The external reference voltage is typically described as `vref-supply` in
  the DT binding and fetched with `devm_regulator_get_enable_read_voltage()`
  using the supply name `"vref"`.
- Store the reference in millivolts (`st->vref_mv`) for use in scale
  calculations.
- For parts with an internal reference (often 2.5 V), fall back to it when no
  external supply is provided.

### Return Value Conventions for read_raw (DAC Usage)

| Return Value               | DAC Usage                                                 |
|----------------------------|-----------------------------------------------------------|
| `IIO_VAL_INT`              | RAW DAC code readback                                     |
| `IIO_VAL_FRACTIONAL_LOG2`  | SCALE: `val = Vref_mV`, `val2 = resolution_bits`         |
| `IIO_VAL_INT`              | OFFSET: integer offset for bipolar ranges                 |

### Locking

- Use `struct mutex` to protect device register access and shared state.
- Initialize with `devm_mutex_init()` in `probe()`.
- Use `guard(mutex)(&st->lock)` (scoped lock, auto-release) in new code.
- Use `iio_device_claim_direct()` / `iio_device_release_direct()` when raw
  access conflicts with buffer operation (rarely needed for DACs, but
  good practice).

### Coding Style

- Follow the kernel coding style (`Documentation/process/coding-style.rst`).
- Run `scripts/checkpatch.pl` before submitting.
- Use `FIELD_GET()` / `FIELD_PREP()` with `GENMASK()` for register bitfields.
- Use `dev_err_probe()` for all error returns in `probe()`.
- Include headers in alphabetical order within each group.

---

## 11. Commit Format

### Subject Line Prefix

```
iio: dac: <devname>: <brief description>
```

### Examples

```
iio: dac: ad3530r: add support for AD3530R and AD3531R
iio: dac: ad5686: fix output scaling for 14-bit variants
iio: dac: ad5761: add powerdown mode support
iio: dac: ad5791: use devm_iio_device_register
```

### Patch Series for a New DAC Driver

A typical new DAC driver submission is a patch series:

1. `dt-bindings: iio: dac: add adi,adxxxx.yaml` -- DT binding
2. `iio: dac: adxxxx: add support for ADXXXX` -- Driver source
3. `MAINTAINERS: add entry for ADXXXX IIO DAC driver` -- Maintainer entry

### Full Example

```
iio: dac: ad3530r: add support for AD3530R and AD3531R

The AD3530R/AD3531R are low-power, 16-bit, buffered voltage output
digital-to-analog converters with software-programmable gain. They
include a 2.5V internal reference and support per-channel powerdown
modes.

This driver supports:
  - Direct output voltage control via sysfs
  - Per-channel powerdown with configurable mode
  - External and internal reference voltage selection
  - Hardware and software LDAC triggering

Signed-off-by: First Last <first.last@analog.com>
```
