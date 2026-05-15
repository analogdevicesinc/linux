# Linux IIO ADDAC Driver Template

ADDAC (combined ADC+DAC) devices with software-configurable per-channel
functions: voltage input, voltage output, current input, current output.

---

## 1. Purpose

IIO ADDAC drivers serve combined analog-to-digital and digital-to-analog
converter devices where each channel can be independently configured for a
specific function at runtime. Reference devices include the AD74115
(single-channel) and AD74413R (quad-channel).

These devices expose `IIO_VOLTAGE` channels with both input and output
directions on the same physical device. Channels may also appear as
`IIO_CURRENT` depending on the configured function. The channel direction
and type are determined by the per-channel function assignment in the
devicetree.

### IIO Mapping

| Attribute               | Value                                          |
|--------------------------|------------------------------------------------|
| IIO subdirectory         | `drivers/iio/addac/`                           |
| Primary channel type     | `IIO_VOLTAGE` (input + output)                 |
| Secondary channel types  | `IIO_CURRENT`, `IIO_RESISTANCE`                |
| Typical `info_mask` bits | RAW, SCALE, OFFSET, SAMP_FREQ                 |
| Channel direction        | Mixed: `.output = 0` for ADC, `.output = 1` for DAC |

---

## 2. File Checklist

| File                                                                  | Action | Required |
|-----------------------------------------------------------------------|--------|----------|
| `drivers/iio/addac/<devname>.c`                                       | Create | Yes      |
| `drivers/iio/addac/Kconfig`                                           | Modify | Yes      |
| `drivers/iio/addac/Makefile`                                          | Modify | Yes      |
| `Documentation/devicetree/bindings/iio/addac/adi,<devname>.yaml`      | Create | Yes      |
| `include/dt-bindings/iio/addac/adi,<devname>.h`                       | Create | Yes      |

### Notes

- The `dt-bindings` header is required for ADDAC drivers because channel
  function constants (e.g. `CH_FUNC_VOLTAGE_OUTPUT`) are shared between
  the DT binding YAML and the driver source. This allows DT authors to
  use symbolic names instead of raw integers.

---

## 3. DT Binding (.yaml)

Bindings live under `Documentation/devicetree/bindings/iio/addac/adi,<devname>.yaml`.

ADDAC bindings require per-channel child nodes with a function configuration
property (`adi,ch-func`) that determines the channel's operating mode.

```yaml
# SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)
%YAML 1.2
---
$id: http://devicetree.org/schemas/iio/addac/adi,adxxxx.yaml#
$schema: http://devicetree.org/meta-schemas/core.yaml#

title: Analog Devices ADxxxx ADDAC

maintainers:
  - First Last <first.last@analog.com>

description: |
  The ADxxxx is a quad-channel software configurable input/output
  solution. Each channel can be independently configured for voltage
  output, current output, voltage input, current input, resistance
  measurement, or digital input.

properties:
  compatible:
    enum:
      - adi,adxxxx

  reg:
    maxItems: 1

  '#address-cells':
    const: 1

  '#size-cells':
    const: 0

  spi-max-frequency:
    maximum: 1000000

  spi-cpol: true

  interrupts:
    maxItems: 1

  refin-supply: true

  shunt-resistor-micro-ohms:
    description:
      Shunt (sense) resistor value in micro-Ohms.
    default: 100000000

  reset-gpios:
    maxItems: 1

required:
  - compatible
  - reg
  - spi-max-frequency
  - refin-supply

patternProperties:
  "^channel@[0-3]$":
    type: object
    additionalProperties: false
    description: Represents a configurable input/output channel.

    properties:
      reg:
        description: |
          The channel number. Up to 4 channels numbered from 0 to 3.
        minimum: 0
        maximum: 3

      adi,ch-func:
        $ref: /schemas/types.yaml#/definitions/uint32
        description: |
          Channel function.
          0 - CH_FUNC_HIGH_IMPEDANCE
          1 - CH_FUNC_VOLTAGE_OUTPUT
          2 - CH_FUNC_CURRENT_OUTPUT
          3 - CH_FUNC_VOLTAGE_INPUT
          4 - CH_FUNC_CURRENT_INPUT_EXT_POWER
          5 - CH_FUNC_CURRENT_INPUT_LOOP_POWER
        minimum: 0
        maximum: 5
        default: 0

      adi,gpo-comparator:
        type: boolean
        description: |
          Whether to configure GPO as a comparator output.

      drive-strength-microamp:
        description: |
          Sink current for channels configured as digital input.
        minimum: 0
        maximum: 1800
        default: 0
        multipleOf: 120

    required:
      - reg

allOf:
  - $ref: /schemas/spi/spi-peripheral-props.yaml#

unevaluatedProperties: false

examples:
  - |
    #include <dt-bindings/iio/addac/adi,adxxxx.h>

    spi {
        #address-cells = <1>;
        #size-cells = <0>;

        addac@0 {
            compatible = "adi,adxxxx";
            reg = <0>;
            spi-max-frequency = <1000000>;
            spi-cpol;

            #address-cells = <1>;
            #size-cells = <0>;

            interrupt-parent = <&gpio>;
            interrupts = <26 2>;

            refin-supply = <&addac_refin>;
            reset-gpios = <&gpio2 6 0>;

            channel@0 {
                reg = <0>;
                adi,ch-func = <CH_FUNC_VOLTAGE_OUTPUT>;
            };

            channel@1 {
                reg = <1>;
                adi,ch-func = <CH_FUNC_CURRENT_OUTPUT>;
            };

            channel@2 {
                reg = <2>;
                adi,ch-func = <CH_FUNC_VOLTAGE_INPUT>;
            };

            channel@3 {
                reg = <3>;
                adi,ch-func = <CH_FUNC_CURRENT_INPUT_EXT_POWER>;
            };
        };
    };
```

### I2C Bus Variant

For I2C-connected ADDAC devices, replace the SPI bus properties:

```yaml
properties:
  compatible:
    enum:
      - adi,adxxxx

  reg:
    maxItems: 1

  '#address-cells':
    const: 1

  '#size-cells':
    const: 0

  interrupts:
    maxItems: 1

  refin-supply: true

examples:
  - |
    i2c {
        #address-cells = <1>;
        #size-cells = <0>;

        addac@20 {
            compatible = "adi,adxxxx";
            reg = <0x20>;

            #address-cells = <1>;
            #size-cells = <0>;

            refin-supply = <&addac_refin>;

            channel@0 {
                reg = <0>;
                adi,ch-func = <CH_FUNC_VOLTAGE_OUTPUT>;
            };
        };
    };
```

---

## 4. Kconfig

Add the entry to `drivers/iio/addac/Kconfig` in alphabetical order.

```kconfig
config ADXXXX
	tristate "Analog Devices ADxxxx ADDAC driver"
	depends on GPIOLIB && SPI
	select CRC8
	select REGMAP_SPI
	select IIO_BUFFER
	select IIO_TRIGGERED_BUFFER
	help
	  Say yes here to build support for Analog Devices ADxxxx
	  quad-channel software configurable input/output solution.

	  To compile this driver as a module, choose M here: the
	  module will be called adxxxx.
```

### Common Optional Selects for ADDAC

| Select                | Purpose                                     |
|-----------------------|---------------------------------------------|
| `CRC8`               | CRC validation on SPI frames                |
| `REGMAP_SPI`         | regmap SPI bus abstraction                  |
| `GPIOLIB`            | GPIO subsystem (GPO/comparator outputs)     |
| `IIO_BUFFER`         | Buffer support for continuous capture       |
| `IIO_TRIGGERED_BUFFER` | Triggered buffer infrastructure           |

---

## 5. Makefile

Add the entry to `drivers/iio/addac/Makefile` in alphabetical order.

```makefile
obj-$(CONFIG_ADXXXX) += adxxxx.o
```

---

## 6. Driver Source (.c)

ADDAC drivers are distinguished by having mixed input/output `IIO_VOLTAGE`
and `IIO_CURRENT` channels on the same device. The channel set exposed to
userspace is determined at probe time by the per-channel function assigned
in the devicetree.

### Complete Skeleton (SPI ADDAC)

```c
// SPDX-License-Identifier: GPL-2.0
/*
 * ADxxxx ADDAC driver
 *
 * Copyright 2024 Analog Devices Inc.
 * Author: First Last <first.last@analog.com>
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/crc8.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/driver.h>
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/interrupt.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#include <dt-bindings/iio/addac/adi,adxxxx.h>

/* ------------------------------------------------------------------ */
/* Register Definitions                                                */
/* ------------------------------------------------------------------ */

#define ADXXXX_REG_CH_FUNC_SETUP(x)	(0x01 + (x))
#define ADXXXX_CH_FUNC_SETUP_MASK	GENMASK(3, 0)

#define ADXXXX_REG_ADC_CONFIG(x)	(0x05 + (x))
#define ADXXXX_REG_DAC_CODE(x)		(0x16 + (x))
#define ADXXXX_REG_ADC_RESULT(x)	(0x26 + (x))
#define ADXXXX_REG_ADC_CONV_CTRL	0x23

#define ADXXXX_DAC_CODE_MAX		GENMASK(12, 0)
#define ADXXXX_ADC_RESULT_MAX		GENMASK(15, 0)

#define ADXXXX_CHANNEL_MAX		4
#define ADXXXX_FRAME_SIZE		4

/* ------------------------------------------------------------------ */
/* Channel Function Enum (matches dt-bindings header)                  */
/* ------------------------------------------------------------------ */

/*
 * These values must match the CH_FUNC_* constants defined in
 * include/dt-bindings/iio/addac/adi,adxxxx.h
 */

/* ------------------------------------------------------------------ */
/* Per-Channel Config and Device State                                 */
/* ------------------------------------------------------------------ */

struct adxxxx_channel_config {
	u32		func;
	bool		initialized;
};

struct adxxxx_channels {
	struct iio_chan_spec	*channels;
	unsigned int		num_channels;
};

struct adxxxx_state {
	struct adxxxx_channel_config	channel_configs[ADXXXX_CHANNEL_MAX];
	struct spi_device		*spi;
	struct device			*dev;
	struct regmap			*regmap;
	struct regulator		*refin_reg;
	struct iio_trigger		*trig;

	/*
	 * Synchronize consecutive operations when doing a one-shot
	 * conversion and when updating the ADC samples SPI message.
	 */
	struct mutex			lock;

	const char			*name;

	/*
	 * DMA (thus cache coherency maintenance) may require the
	 * transfer buffers to live in their own cache lines.
	 */
	u8 buf[ADXXXX_FRAME_SIZE] __aligned(IIO_DMA_MINALIGN);
};

/* ------------------------------------------------------------------ */
/* Channel Specification: Mixed Input/Output                           */
/* ------------------------------------------------------------------ */

/*
 * DAC channel macro -- output channels (.output = 1).
 * scan_index = -1 because output channels are not scanned into buffers.
 */
#define ADXXXX_DAC_CHANNEL(_type, extra_mask_separate)		\
	{							\
		.type = (_type),				\
		.indexed = 1,					\
		.output = 1,					\
		.scan_index = -1,				\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW)	\
				      | BIT(IIO_CHAN_INFO_SCALE)\
				      | (extra_mask_separate),	\
	}

/*
 * ADC channel macro -- input channels (.output = 0).
 * These participate in buffer scans.
 */
#define ADXXXX_ADC_CHANNEL(_type, extra_mask_separate)		\
	{							\
		.type = (_type),				\
		.indexed = 1,					\
		.output = 0,					\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW)	\
				      | BIT(IIO_CHAN_INFO_SAMP_FREQ)\
				      | BIT(IIO_CHAN_INFO_SCALE)\
				      | BIT(IIO_CHAN_INFO_OFFSET)\
				      | (extra_mask_separate),	\
		.info_mask_separate_available =			\
				BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
		.scan_type = {					\
			.sign = 'u',				\
			.realbits = 16,				\
			.storagebits = 32,			\
			.shift = 8,				\
			.endianness = IIO_BE,			\
		},						\
	}

/*
 * Per-function channel arrays.
 * Each function mode produces a different set of IIO channels.
 * Voltage output mode: DAC voltage out + ADC current readback.
 * Current output mode: DAC current out + ADC voltage readback.
 * Voltage input mode:  ADC voltage in only.
 * Current input mode:  ADC current in only.
 */

static struct iio_chan_spec adxxxx_voltage_output_channels[] = {
	ADXXXX_DAC_CHANNEL(IIO_VOLTAGE, 0),
	ADXXXX_ADC_CHANNEL(IIO_CURRENT, 0),
};

static struct iio_chan_spec adxxxx_current_output_channels[] = {
	ADXXXX_DAC_CHANNEL(IIO_CURRENT, 0),
	ADXXXX_ADC_CHANNEL(IIO_VOLTAGE, 0),
};

static struct iio_chan_spec adxxxx_voltage_input_channels[] = {
	ADXXXX_ADC_CHANNEL(IIO_VOLTAGE, 0),
};

static struct iio_chan_spec adxxxx_current_input_channels[] = {
	ADXXXX_ADC_CHANNEL(IIO_CURRENT, 0),
};

/*
 * Channel map: indexed by CH_FUNC_* enum value.
 * Maps each function to the appropriate channel array.
 */
#define _ADXXXX_CHANNELS(_channels)			\
	{						\
		.channels = _channels,			\
		.num_channels = ARRAY_SIZE(_channels),	\
	}

#define ADXXXX_CHANNELS(name) \
	_ADXXXX_CHANNELS(adxxxx_ ## name ## _channels)

static const struct adxxxx_channels adxxxx_channels_map[] = {
	[CH_FUNC_HIGH_IMPEDANCE]         = ADXXXX_CHANNELS(voltage_input),
	[CH_FUNC_VOLTAGE_OUTPUT]         = ADXXXX_CHANNELS(voltage_output),
	[CH_FUNC_CURRENT_OUTPUT]         = ADXXXX_CHANNELS(current_output),
	[CH_FUNC_VOLTAGE_INPUT]          = ADXXXX_CHANNELS(voltage_input),
	[CH_FUNC_CURRENT_INPUT_EXT_POWER] = ADXXXX_CHANNELS(current_input),
	[CH_FUNC_CURRENT_INPUT_LOOP_POWER] = ADXXXX_CHANNELS(current_input),
};

/* ------------------------------------------------------------------ */
/* read_raw / write_raw: Handle Mixed Channel Directions               */
/* ------------------------------------------------------------------ */

/*
 * The key ADDAC pattern: read_raw and write_raw must branch on both
 * chan->type (IIO_VOLTAGE vs IIO_CURRENT) and chan->output (DAC vs ADC)
 * to return appropriate scale, offset, and raw values.
 */

static int adxxxx_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long info)
{
	struct adxxxx_state *st = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		if (chan->output) {
			/* Read back DAC code */
			guard(mutex)(&st->lock);
			return regmap_read(st->regmap,
					   ADXXXX_REG_DAC_CODE(chan->channel),
					   val);
		}

		/* Read ADC conversion result */
		if (!iio_device_claim_direct(indio_dev))
			return -EBUSY;

		guard(mutex)(&st->lock);
		/* Trigger single conversion, wait, read result */
		/* ret = adxxxx_get_single_adc_result(st, chan->channel, val); */
		iio_device_release_direct(indio_dev);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_VOLTAGE:
			if (chan->output) {
				/* DAC voltage scale: Vmax / code_max */
				*val = 12000;  /* mV */
				*val2 = ADXXXX_DAC_CODE_MAX;
				return IIO_VAL_FRACTIONAL;
			}
			/* ADC voltage scale */
			*val = 10000;
			*val2 = ADXXXX_ADC_RESULT_MAX;
			return IIO_VAL_FRACTIONAL;

		case IIO_CURRENT:
			if (chan->output) {
				/* DAC current scale */
				*val = 25000;  /* uA per code */
				*val2 = ADXXXX_DAC_CODE_MAX;
				return IIO_VAL_FRACTIONAL;
			}
			/* ADC current scale */
			*val = 25000;
			*val2 = ADXXXX_ADC_RESULT_MAX;
			return IIO_VAL_FRACTIONAL;

		default:
			return -EINVAL;
		}

	case IIO_CHAN_INFO_OFFSET:
		switch (chan->type) {
		case IIO_VOLTAGE:
			/* Compute offset based on ADC range config */
			*val = 0;
			return IIO_VAL_INT;
		case IIO_CURRENT:
			*val = 0;
			return IIO_VAL_INT;
		default:
			return -EINVAL;
		}

	case IIO_CHAN_INFO_SAMP_FREQ:
		guard(mutex)(&st->lock);
		/* Return current ADC sample rate */
		*val = 4800;
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
	case IIO_CHAN_INFO_RAW:
		if (!chan->output)
			return -EINVAL;

		if (val < 0 || val > ADXXXX_DAC_CODE_MAX)
			return -EINVAL;

		guard(mutex)(&st->lock);
		return regmap_write(st->regmap,
				    ADXXXX_REG_DAC_CODE(chan->channel), val);

	case IIO_CHAN_INFO_SAMP_FREQ:
		guard(mutex)(&st->lock);
		/* Configure ADC sample rate */
		return 0;

	default:
		return -EINVAL;
	}
}

/* ------------------------------------------------------------------ */
/* iio_info                                                            */
/* ------------------------------------------------------------------ */

static const struct iio_info adxxxx_info = {
	.read_raw = adxxxx_read_raw,
	.write_raw = adxxxx_write_raw,
	.read_avail = adxxxx_read_avail,          /* see section 7 */
	.update_scan_mode = adxxxx_update_scan_mode,
};

/* ------------------------------------------------------------------ */
/* Probe                                                               */
/* ------------------------------------------------------------------ */

static int adxxxx_probe(struct spi_device *spi)
{
	struct adxxxx_state *st;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;
	st->dev = &spi->dev;

	ret = devm_mutex_init(st->dev, &st->lock);
	if (ret)
		return ret;

	st->regmap = devm_regmap_init(st->dev, NULL, st,
				      &adxxxx_regmap_config);
	if (IS_ERR(st->regmap))
		return PTR_ERR(st->regmap);

	st->refin_reg = devm_regulator_get(st->dev, "refin");
	if (IS_ERR(st->refin_reg))
		return dev_err_probe(st->dev, PTR_ERR(st->refin_reg),
				     "Failed to get refin regulator\n");

	ret = regulator_enable(st->refin_reg);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(st->dev, adxxxx_regulator_disable,
				       st->refin_reg);
	if (ret)
		return ret;

	/* Reset the device */
	/* ... */

	indio_dev->name = "adxxxx";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &adxxxx_info;

	/* Parse per-channel function configs from DT (section 8) */
	ret = adxxxx_parse_channel_configs(indio_dev);
	if (ret)
		return ret;

	/* Build the channel array from per-channel function map */
	ret = adxxxx_setup_channels(indio_dev);
	if (ret)
		return ret;

	/* Set up triggered buffer */
	ret = devm_iio_triggered_buffer_setup(st->dev, indio_dev,
					      &iio_pollfunc_store_time,
					      &adxxxx_trigger_handler,
					      &adxxxx_buffer_ops);
	if (ret)
		return ret;

	return devm_iio_device_register(st->dev, indio_dev);
}

/* ------------------------------------------------------------------ */
/* Match Tables                                                        */
/* ------------------------------------------------------------------ */

static const struct of_device_id adxxxx_of_match[] = {
	{ .compatible = "adi,adxxxx", },
	{ }
};
MODULE_DEVICE_TABLE(of, adxxxx_of_match);

static const struct spi_device_id adxxxx_ids[] = {
	{ "adxxxx", },
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
MODULE_DESCRIPTION("Analog Devices ADxxxx ADDAC driver");
MODULE_LICENSE("GPL");
```

---

## 7. Channel Function Config

ADDAC devices are multi-function: the same physical channel can operate as a
voltage output, current output, voltage input, or current input. The function
is selected per-channel at probe time via the devicetree.

### dt-bindings Header

Create `include/dt-bindings/iio/addac/adi,<devname>.h` with symbolic constants
that match the hardware register values:

```c
/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause) */

#ifndef _DT_BINDINGS_IIO_ADDAC_ADI_ADXXXX_H
#define _DT_BINDINGS_IIO_ADDAC_ADI_ADXXXX_H

#define CH_FUNC_HIGH_IMPEDANCE              0x0
#define CH_FUNC_VOLTAGE_OUTPUT              0x1
#define CH_FUNC_CURRENT_OUTPUT              0x2
#define CH_FUNC_VOLTAGE_INPUT               0x3
#define CH_FUNC_CURRENT_INPUT_EXT_POWER     0x4
#define CH_FUNC_CURRENT_INPUT_LOOP_POWER    0x5
/* Add device-specific functions as needed (resistance, digital, HART...) */

#define CH_FUNC_MIN  CH_FUNC_HIGH_IMPEDANCE
#define CH_FUNC_MAX  CH_FUNC_CURRENT_INPUT_LOOP_POWER

#endif /* _DT_BINDINGS_IIO_ADDAC_ADI_ADXXXX_H */
```

### Channel Map Pattern

The driver defines separate `iio_chan_spec` arrays for each function mode, then
maps them via an array indexed by the `CH_FUNC_*` enum:

```c
static const struct adxxxx_channels adxxxx_channels_map[] = {
	[CH_FUNC_HIGH_IMPEDANCE]           = ADXXXX_CHANNELS(voltage_input),
	[CH_FUNC_VOLTAGE_OUTPUT]           = ADXXXX_CHANNELS(voltage_output),
	[CH_FUNC_CURRENT_OUTPUT]           = ADXXXX_CHANNELS(current_output),
	[CH_FUNC_VOLTAGE_INPUT]            = ADXXXX_CHANNELS(voltage_input),
	[CH_FUNC_CURRENT_INPUT_EXT_POWER]  = ADXXXX_CHANNELS(current_input),
	[CH_FUNC_CURRENT_INPUT_LOOP_POWER] = ADXXXX_CHANNELS(current_input),
};
```

### Hardware Function Setup

After parsing DT, write the function code to the channel function setup
register:

```c
static int adxxxx_set_channel_function(struct adxxxx_state *st,
				       unsigned int channel, u32 func)
{
	int ret;

	/* First set high-impedance to safely transition */
	ret = regmap_update_bits(st->regmap,
				 ADXXXX_REG_CH_FUNC_SETUP(channel),
				 ADXXXX_CH_FUNC_SETUP_MASK,
				 CH_FUNC_HIGH_IMPEDANCE);
	if (ret)
		return ret;

	/* Short delay for safe transition between modes */
	/* ... */

	/* Set the desired function */
	return regmap_update_bits(st->regmap,
				  ADXXXX_REG_CH_FUNC_SETUP(channel),
				  ADXXXX_CH_FUNC_SETUP_MASK, func);
}
```

---

## 8. DT Parsing

ADDAC drivers iterate over channel child nodes to read the per-channel
function assignment. Each child node contains a `reg` property (channel index)
and an `adi,ch-func` property (function enum value).

### Per-Channel Config Parsing

```c
static int adxxxx_parse_channel_config(struct iio_dev *indio_dev,
				       struct fwnode_handle *channel_node)
{
	struct adxxxx_state *st = iio_priv(indio_dev);
	struct adxxxx_channel_config *config;
	u32 index;
	int ret;

	ret = fwnode_property_read_u32(channel_node, "reg", &index);
	if (ret)
		return dev_err_probe(st->dev, ret,
				     "Failed to read channel reg\n");

	if (index >= ADXXXX_CHANNEL_MAX)
		return dev_err_probe(st->dev, -EINVAL,
				     "Channel index %u too large\n", index);

	config = &st->channel_configs[index];
	if (config->initialized)
		return dev_err_probe(st->dev, -EINVAL,
				     "Channel %u already initialized\n", index);

	config->func = CH_FUNC_HIGH_IMPEDANCE;
	fwnode_property_read_u32(channel_node, "adi,ch-func", &config->func);

	if (config->func < CH_FUNC_MIN || config->func > CH_FUNC_MAX)
		return dev_err_probe(st->dev, -EINVAL,
				     "Invalid channel function %u\n",
				     config->func);

	/* Accumulate total channel count for allocation */
	indio_dev->num_channels +=
		adxxxx_channels_map[config->func].num_channels;

	config->initialized = true;

	return 0;
}

static int adxxxx_parse_channel_configs(struct iio_dev *indio_dev)
{
	struct adxxxx_state *st = iio_priv(indio_dev);

	device_for_each_child_node_scoped(st->dev, channel_node) {
		int ret;

		ret = adxxxx_parse_channel_config(indio_dev, channel_node);
		if (ret)
			return ret;
	}

	return 0;
}
```

### Building the Channel Array

After parsing, build the consolidated `iio_chan_spec` array by copying from
the per-function channel templates:

```c
static int adxxxx_setup_channels(struct iio_dev *indio_dev)
{
	struct adxxxx_state *st = iio_priv(indio_dev);
	struct adxxxx_channel_config *config;
	struct iio_chan_spec *channels, *chans;
	unsigned int i, num_chans, chan_i;
	int ret;

	channels = devm_kcalloc(st->dev, sizeof(*channels),
				indio_dev->num_channels, GFP_KERNEL);
	if (!channels)
		return -ENOMEM;

	indio_dev->channels = channels;

	for (i = 0; i < ADXXXX_CHANNEL_MAX; i++) {
		config = &st->channel_configs[i];
		chans = adxxxx_channels_map[config->func].channels;
		num_chans = adxxxx_channels_map[config->func].num_channels;

		memcpy(channels, chans, num_chans * sizeof(*chans));

		/* Assign the physical channel index to each spec */
		for (chan_i = 0; chan_i < num_chans; chan_i++) {
			struct iio_chan_spec *chan = &channels[chan_i];

			chan->channel = i;
			if (chan->output)
				chan->scan_index = -1;
			else
				chan->scan_index = i;
		}

		/* Program the hardware for this function */
		ret = adxxxx_set_channel_function(st, i, config->func);
		if (ret)
			return ret;

		channels += num_chans;
	}

	return 0;
}
```

### Common DT Properties for ADDAC

| Property                    | API                                 | Purpose                         |
|-----------------------------|-------------------------------------|---------------------------------|
| `reg`                       | `fwnode_property_read_u32()`       | Channel index (0..N-1)          |
| `adi,ch-func`               | `fwnode_property_read_u32()`       | Channel function enum           |
| `adi,gpo-comparator`        | `fwnode_property_read_bool()`      | GPO comparator mode             |
| `drive-strength-microamp`   | `fwnode_property_read_u32()`       | Digital input sink current      |
| `shunt-resistor-micro-ohms` | `device_property_read_u32()`       | Sense resistor (device-level)   |

---

## 9. Test & Debug

### Sysfs Interface

ADDAC devices expose mixed input and output channels. The exact set of sysfs
attributes depends on which function is assigned to each channel:

```
/sys/bus/iio/devices/iio:device0/
    name                               # Device name ("ad74413r")

    # Channel 0: voltage output mode
    out_voltage0_raw                   # DAC code (write to set output)
    out_voltage0_scale                 # Voltage per LSB
    in_current0_raw                    # ADC readback of output current
    in_current0_scale                  # Current scale
    in_current0_offset                 # Current offset
    in_current0_sampling_frequency     # ADC rate

    # Channel 1: current output mode
    out_current1_raw                   # DAC code
    out_current1_scale                 # Current per LSB
    in_voltage1_raw                    # ADC readback of output voltage
    in_voltage1_scale                  # Voltage scale
    in_voltage1_offset                 # Voltage offset

    # Channel 2: voltage input mode
    in_voltage2_raw                    # ADC reading
    in_voltage2_scale                  # Voltage scale
    in_voltage2_offset                 # Voltage offset
    in_voltage2_sampling_frequency     # ADC rate

    # Channel 3: current input mode
    in_current3_raw                    # ADC reading
    in_current3_scale                  # Current scale
    in_current3_offset                 # Current offset
    in_current3_sampling_frequency     # ADC rate
```

### Verification Commands

```sh
# List all IIO devices
iio_info

# Read a specific channel attribute
cat /sys/bus/iio/devices/iio:device0/in_voltage2_raw
cat /sys/bus/iio/devices/iio:device0/in_voltage2_scale

# Set DAC output (voltage output on channel 0)
echo 4096 > /sys/bus/iio/devices/iio:device0/out_voltage0_raw

# Read back DAC-driven current (readback ADC for channel 0)
cat /sys/bus/iio/devices/iio:device0/in_current0_raw

# Read available sampling frequencies
cat /sys/bus/iio/devices/iio:device0/in_voltage2_sampling_frequency_available

# Set sampling frequency
echo 4800 > /sys/bus/iio/devices/iio:device0/in_voltage2_sampling_frequency

# Buffered continuous read
iio_readdev -b 256 -s 1024 iio:device0

# debugfs register access (requires CONFIG_DEBUG_FS)
echo 0x26 > /sys/kernel/debug/iio/iio:device0/direct_reg_access
cat /sys/kernel/debug/iio/iio:device0/direct_reg_access
```

### Kernel Debugging Tips

- Enable `CONFIG_IIO_SIMPLE_DUMMY` to study a minimal reference driver.
- Use `dev_dbg()` / `dev_info()` for debug prints; never `printk()`.
- Verify each channel's function was correctly programmed by reading back the
  `CH_FUNC_SETUP` register via debugfs.

---

## 10. Key Conventions

### License

All new ADI IIO drivers must use GPL-2.0 or GPL-2.0+:

```c
// SPDX-License-Identifier: GPL-2.0
```

The SPDX tag goes on the very first line. `MODULE_LICENSE("GPL")` at the
bottom must match.

### Memory Management

Use `devm_*` (device-managed) allocations exclusively:

| Function                             | Purpose                             |
|--------------------------------------|-------------------------------------|
| `devm_iio_device_alloc()`           | Allocate IIO device + private data  |
| `devm_iio_device_register()`        | Register IIO device (auto-unregister) |
| `devm_iio_triggered_buffer_setup()` | Set up triggered buffer             |
| `devm_regulator_get()`              | Get regulator handle                |
| `devm_add_action_or_reset()`        | Register custom cleanup callback    |
| `devm_regmap_init()`                | Initialize regmap                   |
| `devm_mutex_init()`                 | Initialize a mutex with devm cleanup |
| `devm_kcalloc()`                    | Allocate zeroed array               |
| `devm_request_irq()`               | Request interrupt with auto-free    |
| `devm_gpiochip_add_data()`          | Register GPIO chip (GPO outputs)    |

### Multi-Function Device Patterns

ADDAC drivers are inherently multi-function. Key patterns:

1. **Per-channel config struct**: Store the function assignment and any
   per-channel options in an array of config structs, one per physical channel.

2. **Channel map array**: Define separate `iio_chan_spec` arrays for each
   function mode. Use a lookup table indexed by the function enum to select
   the correct channel set at probe time.

3. **Dynamic channel array**: Allocate the final `iio_chan_spec` array at probe
   after summing up the channel counts from each configured function. Copy
   from the per-function templates and assign physical channel indices.

4. **Branching on direction**: In `read_raw` and `write_raw`, always check
   `chan->output` to distinguish DAC operations from ADC operations. Also
   check `chan->type` (`IIO_VOLTAGE` vs `IIO_CURRENT`) since the same function
   may expose multiple channel types.

5. **GPIO subsystem integration**: ADDAC devices often expose GPO pins that
   can be configured as GPIO outputs or comparator outputs. Register
   `gpio_chip` instances when GPO channels are configured.

### Coding Style

- Follow kernel coding style (`Documentation/process/coding-style.rst`).
- Run `scripts/checkpatch.pl` before submitting.
- Use `FIELD_GET()` / `FIELD_PREP()` with `GENMASK()` for register bitfields.
- Use `BIT()` for single-bit definitions.
- Use `dev_err_probe()` for all error returns in `probe()`.
- Include headers in alphabetical order within each group.
- Use `guard(mutex)(&st->lock)` (scoped lock) where possible.

### Return Value Conventions for read_raw

| Return Value                  | Meaning                      | `*val`, `*val2` Usage             |
|-------------------------------|------------------------------|-----------------------------------|
| `IIO_VAL_INT`                 | Integer value                | Result = `val`                    |
| `IIO_VAL_INT_PLUS_MICRO`     | Integer + fractional (micro) | Result = `val + val2/1000000`     |
| `IIO_VAL_FRACTIONAL`         | Fraction                     | Result = `val / val2`             |
| `IIO_VAL_FRACTIONAL_LOG2`    | Log2 denominator             | Result = `val / 2^val2`           |
| Negative errno                | Error                        | -EINVAL, -EIO, etc.              |

---

## 11. Commit Format

### Subject Line Prefix

```
iio: addac: <devname>: <brief description>
```

### Examples

```
iio: addac: ad74413r: add support for AD74412R/AD74413R
iio: addac: ad74115: add support for AD74115H
iio: addac: ad74413r: fix current input scale calculation
iio: addac: ad74115: add HART function support
```

### DT Binding Commit

```
dt-bindings: iio: addac: add adi,adxxxx.yaml
```

### dt-bindings Header Commit

```
dt-bindings: iio: addac: add channel function defines for adxxxx
```

### Patch Series for a New ADDAC Driver

A typical new ADDAC driver submission is a patch series:

1. `dt-bindings: iio: addac: add adi,<devname>.yaml` -- DT binding
2. `dt-bindings: iio: addac: add channel function defines for <devname>` -- dt-bindings header
3. `iio: addac: <devname>: add support for <DEVNAME>` -- Driver source
4. `MAINTAINERS: add entry for <DEVNAME> IIO driver` -- Maintainer entry

### Full Example

```
iio: addac: ad74413r: add support for AD74412R/AD74413R

The AD74412R and AD74413R are quad-channel software configurable
input/output solutions for building and process control applications.
Each channel can be independently configured for:
  - Voltage output (0 to 11V)
  - Current output (0 to 25mA)
  - Voltage input (0 to 10V)
  - Current input (0 to 25mA, external or loop powered)
  - Resistance input (RTD measurement)
  - Digital input (logic or loop powered)

The devices feature a 16-bit ADC and four 13-bit DACs with an SPI
interface. The AD74413R additionally supports HART communication.

Signed-off-by: First Last <first.last@analog.com>
```
