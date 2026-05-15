# Linux IIO ADC Driver Template

This template covers writing a Linux kernel IIO driver for Analog Devices ADC
parts. It builds on the common IIO patterns from `drivers/iio/DRIVER_TEMPLATE.md`
and specializes them for analog-to-digital converters: `IIO_VOLTAGE` input
channels (single-ended and differential), sigma-delta / SAR / pipeline
architectures, reference voltage handling, and buffered sampling.

---

## 1. Purpose & Subsystem

ADC drivers live under the IIO (Industrial I/O) subsystem and expose input
voltage channels to userspace.

| Attribute          | Value                                                                               |
|--------------------|-------------------------------------------------------------------------------------|
| IIO subdirectory   | `drivers/iio/adc/`                                                                  |
| Channel type       | `IIO_VOLTAGE` (input)                                                               |
| Channel variants   | Single-ended (`.indexed = 1`) and differential (`.differential = 1`)                |
| Architectures      | Sigma-delta, SAR (successive approximation), pipeline                               |
| Typical info_mask  | `RAW`, `SCALE`, `OFFSET`, `SAMP_FREQ`, `OVERSAMPLING_RATIO`                        |

### info_mask Bit Usage

| Bit                        | Meaning for ADCs                                                     |
|----------------------------|----------------------------------------------------------------------|
| `IIO_CHAN_INFO_RAW`        | Raw conversion result (integer code)                                 |
| `IIO_CHAN_INFO_SCALE`      | Volts per LSB, typically Vref / 2^N (returned as FRACTIONAL_LOG2)    |
| `IIO_CHAN_INFO_OFFSET`     | Bipolar offset: -(2^(N-1)) for signed, 0 for unipolar               |
| `IIO_CHAN_INFO_SAMP_FREQ`  | Output data rate in Hz (per-channel or shared)                       |
| `IIO_CHAN_INFO_OVERSAMPLING_RATIO` | Hardware oversampling / averaging factor                      |

---

## 2. File Checklist

| File                                                              | Action   | Required |
|-------------------------------------------------------------------|----------|----------|
| `drivers/iio/adc/<devname>.c`                                     | Create   | Yes      |
| `drivers/iio/adc/Kconfig`                                         | Modify   | Yes      |
| `drivers/iio/adc/Makefile`                                        | Modify   | Yes      |
| `Documentation/devicetree/bindings/iio/adc/adi,<devname>.yaml`    | Create   | Yes      |
| `include/dt-bindings/iio/adc/<devname>.h`                         | Create   | Optional |

### Notes

- The optional header is only needed when the driver exports dt-binding
  constants (e.g. reference selection enums shared between the YAML binding
  and the C driver).
- For multi-file drivers (e.g. separate trigger or calibration logic), list
  all object files in the Makefile using the `<mod>-y :=` syntax.

---

## 3. Devicetree Binding

Bindings live under `Documentation/devicetree/bindings/iio/adc/adi,<devname>.yaml`.

ADC bindings typically include bus properties (SPI or I2C), reference voltage
supplies, an interrupt for data-ready signaling, and child nodes describing
each configured channel.

### SPI ADC Binding Example

```yaml
# SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)
%YAML 1.2
---
$id: http://devicetree.org/schemas/iio/adc/adi,ad7124.yaml#
$schema: http://devicetree.org/meta-schemas/core.yaml#

title: Analog Devices AD7124 ADC

maintainers:
  - First Last <first.last@analog.com>

description: |
  The AD7124 is a low power, low noise, complete analog front end for high
  precision measurement applications. It contains a 24-bit sigma-delta ADC.

properties:
  compatible:
    enum:
      - adi,ad7124-4
      - adi,ad7124-8

  reg:
    maxItems: 1

  spi-max-frequency:
    maximum: 5000000

  clocks:
    maxItems: 1

  clock-names:
    const: mclk

  interrupts:
    maxItems: 1

  '#address-cells':
    const: 1

  '#size-cells':
    const: 0

  avdd-supply: true
  refin1-supply: true
  refin2-supply: true

patternProperties:
  "^channel@[0-9a-f]+$":
    type: object
    additionalProperties: false
    description: Represents an ADC channel.

    properties:
      reg:
        minimum: 0
        maximum: 15

      diff-channels:
        $ref: /schemas/types.yaml#/definitions/uint32-array
        items:
          - description: Positive input (AINP)
          - description: Negative input (AINM)

      bipolar:
        type: boolean
        description: Set for bipolar operation.

      adi,reference-select:
        $ref: /schemas/types.yaml#/definitions/uint32
        enum: [0, 1, 2, 3]
        description: Reference source selection.

      adi,buffered-positive:
        type: boolean
        description: Enable input buffer on positive input.

      adi,buffered-negative:
        type: boolean
        description: Enable input buffer on negative input.

    required:
      - reg
      - diff-channels

required:
  - compatible
  - reg

additionalProperties: false

examples:
  - |
    spi {
        #address-cells = <1>;
        #size-cells = <0>;

        adc@0 {
            compatible = "adi,ad7124-4";
            reg = <0>;
            spi-max-frequency = <5000000>;
            interrupts = <25 2>;
            interrupt-parent = <&gpio>;
            refin1-supply = <&adc_vref>;

            #address-cells = <1>;
            #size-cells = <0>;

            channel@0 {
                reg = <0>;
                diff-channels = <0 1>;
                adi,reference-select = <0>;
                adi,buffered-positive;
                adi,buffered-negative;
            };

            channel@1 {
                reg = <1>;
                diff-channels = <2 3>;
                bipolar;
                adi,reference-select = <0>;
            };
        };
    };
```

### ADC-Specific Binding Patterns

| Property               | Purpose                                              |
|------------------------|------------------------------------------------------|
| `#address-cells = <1>` | Required when channel child nodes use `reg`           |
| `#size-cells = <0>`    | Channel nodes have no size                            |
| `diff-channels`        | Two-element array: `<AINP AINM>` for differential     |
| `single-channel`       | Single element for single-ended inputs                |
| `bipolar`              | Boolean: enables bipolar (signed) mode                |
| `adi,reference-select` | Enum selecting REFIN1, REFIN2, AVDD, or internal ref  |
| `refin-supply`         | Regulator phandle for external reference voltage       |
| `avdd-supply`          | Regulator phandle for analog supply (often also ref)   |
| `adi,buffered-positive`| Enable analog input buffer on AIN+                    |
| `adi,buffered-negative`| Enable analog input buffer on AIN-                    |

---

## 4. Kconfig

Add the entry to `drivers/iio/adc/Kconfig` in **alphabetical order**. The
Kconfig file begins with:

```
# SPDX-License-Identifier: GPL-2.0-only
#
# ADC drivers
#
# When adding new entries keep the list in alphabetical order

menu "Analog to digital converters"
```

### Sigma-Delta ADC Example

```kconfig
config AD7124
	tristate "Analog Devices AD7124 ADC driver"
	depends on SPI_MASTER
	select AD_SIGMA_DELTA
	help
	  Say yes here to build support for Analog Devices AD7124-4 and
	  AD7124-8 low power, low noise 24-bit sigma-delta ADCs.

	  To compile this driver as a module, choose M here: the module will be
	  called ad7124.
```

### SAR ADC Example

```kconfig
config AD7944
	tristate "Analog Devices AD7944 ADC driver"
	depends on SPI_MASTER
	select IIO_BUFFER
	select IIO_TRIGGERED_BUFFER
	help
	  Say yes here to build support for Analog Devices AD7944, AD7985,
	  and AD7986 16/18-bit SAR ADCs.

	  To compile this driver as a module, choose M here: the module will be
	  called ad7944.
```

### Common Selects for ADC Drivers

| Select                    | When to use                                        |
|---------------------------|----------------------------------------------------|
| `AD_SIGMA_DELTA`          | Sigma-delta ADCs using the shared library           |
| `IIO_BUFFER`              | Any driver that supports continuous capture          |
| `IIO_TRIGGERED_BUFFER`    | Drivers using software-triggered buffers             |
| `IIO_BUFFER_DMAENGINE`   | Drivers using DMA-based buffer transfers             |
| `SPI_OFFLOAD`             | SPI offload engine for high-throughput SAR ADCs      |
| `REGMAP_SPI`              | When using regmap over SPI                           |

---

## 5. Makefile

Add the entry to `drivers/iio/adc/Makefile` in **alphabetical order**.

### Single-File Driver

```makefile
obj-$(CONFIG_AD7124) += ad7124.o
```

### Multi-File Driver

```makefile
obj-$(CONFIG_AD4170_4) += ad4170-4.o
ad4170-4-y := ad4170-4-core.o ad4170-4-trigger.o
```

---

## 6. Driver Source

### Includes

```c
// SPDX-License-Identifier: GPL-2.0+
/*
 * AD7124 SPI ADC driver
 *
 * Copyright 2024 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

/* For sigma-delta ADCs, also include: */
#include <linux/iio/adc/ad_sigma_delta.h>
```

### Register Definitions

Use `GENMASK()` for multi-bit fields, `BIT()` for single bits,
`FIELD_GET()` / `FIELD_PREP()` for access:

```c
#define AD7124_REG_STATUS		0x00
#define AD7124_REG_ADC_CONTROL		0x01
#define AD7124_REG_DATA			0x02
#define AD7124_REG_CHANNEL(x)		(0x09 + (x))
#define AD7124_REG_CONFIG(x)		(0x19 + (x))
#define AD7124_REG_FILTER(x)		(0x21 + (x))

#define AD7124_ADC_CONTROL_MODE		GENMASK(5, 2)
#define AD7124_CONFIG_BIPOLAR		BIT(11)
#define AD7124_CONFIG_REF_SEL		GENMASK(4, 3)
#define AD7124_CONFIG_PGA		GENMASK(2, 0)
#define AD7124_CHANNEL_AINP		GENMASK(9, 5)
#define AD7124_CHANNEL_AINM		GENMASK(4, 0)
```

### Device State Structure

```c
struct ad7124_chip_info {
	const char		*name;
	unsigned int		chip_id;
	unsigned int		num_channels;
	unsigned int		resolution;
};

struct ad7124_state {
	struct spi_device	*spi;
	const struct ad7124_chip_info *chip_info;
	struct regulator	*vref[4];
	struct mutex		lock;	/* Protect device state */
	unsigned int		num_channels;
	/*
	 * DMA-safe buffer for SPI transfers.
	 * Must be at the end of the struct and must be separately
	 * cache-line aligned.
	 */
	u8			buf[4] __aligned(IIO_DMA_MINALIGN);
};
```

### Channel Specification Macros

#### Single-Ended Channel

```c
#define ADXXXX_CHANNEL(_idx, _addr) {				\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.channel = (_idx),					\
	.address = (_addr),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
	.scan_index = (_idx),					\
	.scan_type = {						\
		.sign = 'u',					\
		.realbits = 16,					\
		.storagebits = 16,				\
		.endianness = IIO_BE,				\
	},							\
}
```

#### Differential Channel

```c
#define ADXXXX_DIFF_CHANNEL(_idx, _pos, _neg) {			\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.differential = 1,					\
	.channel = (_pos),					\
	.channel2 = (_neg),					\
	.address = (_idx),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
		BIT(IIO_CHAN_INFO_SCALE) |			\
		BIT(IIO_CHAN_INFO_OFFSET) |			\
		BIT(IIO_CHAN_INFO_SAMP_FREQ),			\
	.info_mask_shared_by_type_available =			\
		BIT(IIO_CHAN_INFO_SCALE),			\
	.scan_index = (_idx),					\
	.scan_type = {						\
		.sign = 'u',					\
		.realbits = 24,					\
		.storagebits = 32,				\
		.endianness = IIO_BE,				\
	},							\
}
```

Key differences between single-ended and differential:
- `.differential = 1` for differential, absent for single-ended.
- `.channel` = positive input, `.channel2` = negative input.
- Differential channels typically expose `OFFSET` (for bipolar support).
- In sysfs, differential appears as `in_voltage0-voltage1_raw`.

#### Channel Array with Timestamp

```c
static const struct iio_chan_spec adxxxx_channels[] = {
	ADXXXX_CHANNEL(0, 0),
	ADXXXX_CHANNEL(1, 1),
	ADXXXX_CHANNEL(2, 2),
	ADXXXX_CHANNEL(3, 3),
	IIO_CHAN_SOFT_TIMESTAMP(4),
};
```

### read_raw Callback

```c
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
		 * For SAR ADCs: trigger a single conversion and read data.
		 * For sigma-delta: use ad_sigma_delta_single_conversion().
		 * Prevent raw reads while buffer is active.
		 */
		if (!iio_device_claim_direct(indio_dev))
			return -EBUSY;

		mutex_lock(&st->lock);
		ret = adxxxx_read_reg(st, ADXXXX_REG_DATA + chan->address,
				      &regval);
		mutex_unlock(&st->lock);

		iio_device_release_direct(indio_dev);

		if (ret)
			return ret;

		*val = regval;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		/*
		 * Scale = Vref / 2^realbits
		 * Read the regulator voltage at runtime for accuracy.
		 * Return as IIO_VAL_FRACTIONAL_LOG2: val / 2^val2
		 *
		 * For bipolar: Scale = Vref / 2^(realbits-1)
		 */
		ret = regulator_get_voltage(st->vref);
		if (ret < 0)
			return ret;

		*val = ret / 1000;  /* uV to mV */
		*val2 = chan->scan_type.realbits;
		return IIO_VAL_FRACTIONAL_LOG2;

	case IIO_CHAN_INFO_OFFSET:
		/*
		 * Bipolar offset: -(2^(realbits-1))
		 * Unipolar: 0
		 */
		*val = -(1 << (chan->scan_type.realbits - 1));
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SAMP_FREQ:
		mutex_lock(&st->lock);
		/* Read current output data rate from hardware */
		*val = st->current_odr;
		*val2 = 0;
		mutex_unlock(&st->lock);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		*val = st->oversampling;
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}
```

### write_raw Callback

```c
static int adxxxx_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long info)
{
	struct adxxxx_state *st = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (val <= 0)
			return -EINVAL;

		guard(mutex)(&st->lock);

		/* Configure hardware sample rate register */
		return adxxxx_set_odr(st, chan->address, val);

	case IIO_CHAN_INFO_SCALE:
		if (val != 0)
			return -EINVAL;

		guard(mutex)(&st->lock);

		/* Set PGA gain to achieve the requested scale */
		return adxxxx_set_gain(st, chan->address, val2);

	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		guard(mutex)(&st->lock);

		return adxxxx_set_oversampling(st, val);

	default:
		return -EINVAL;
	}
}
```

### debugfs Register Access

```c
static int adxxxx_reg_access(struct iio_dev *indio_dev,
			     unsigned int reg, unsigned int writeval,
			     unsigned int *readval)
{
	struct adxxxx_state *st = iio_priv(indio_dev);

	if (readval)
		return adxxxx_read_reg(st, reg, readval);

	return adxxxx_write_reg(st, reg, writeval);
}
```

### iio_info Structure

```c
static const struct iio_info adxxxx_info = {
	.read_raw = adxxxx_read_raw,
	.write_raw = adxxxx_write_raw,
	.debugfs_reg_access = adxxxx_reg_access,
};
```

### SPI Probe Function

```c
static int adxxxx_probe(struct spi_device *spi)
{
	const struct adxxxx_chip_info *info;
	struct device *dev = &spi->dev;
	struct adxxxx_state *st;
	struct iio_dev *indio_dev;
	int ret;

	info = spi_get_device_match_data(spi);
	if (!info)
		return dev_err_probe(dev, -ENODEV,
				     "Failed to get match data\n");

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;
	st->chip_info = info;

	ret = devm_mutex_init(dev, &st->lock);
	if (ret)
		return ret;

	indio_dev->name = info->name;
	indio_dev->info = &adxxxx_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	/* Enable analog power supply */
	ret = devm_regulator_get_enable(dev, "avdd");
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to enable avdd supply\n");

	/* Get reference voltage regulator (needed for scale calculation) */
	st->vref = devm_regulator_get(dev, "refin");
	if (IS_ERR(st->vref))
		return dev_err_probe(dev, PTR_ERR(st->vref),
				     "Failed to get refin regulator\n");

	ret = regulator_enable(st->vref);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to enable refin supply\n");

	ret = devm_add_action_or_reset(dev, adxxxx_reg_disable, st->vref);
	if (ret)
		return ret;

	/* Parse channel configuration from devicetree child nodes */
	ret = adxxxx_parse_channels(indio_dev, dev);
	if (ret)
		return ret;

	/* Hardware init: reset, verify chip ID, configure defaults */
	ret = adxxxx_init_hw(st);
	if (ret)
		return ret;

	/* Set up triggered buffer for continuous sampling */
	ret = devm_iio_triggered_buffer_setup(dev, indio_dev, NULL,
					      adxxxx_trigger_handler, NULL);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to setup triggered buffer\n");

	return devm_iio_device_register(dev, indio_dev);
}
```

### Match Tables and Module Boilerplate

```c
static const struct adxxxx_chip_info adxxxx_chip_info = {
	.name = "adxxxx",
	.chip_id = 0x00,
	.num_channels = 4,
	.resolution = 16,
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
MODULE_DESCRIPTION("Analog Devices ADXXXX ADC driver");
MODULE_LICENSE("GPL");
```

---

## 7. Buffer & Trigger

ADC drivers typically use triggered buffers for continuous sampling. The
buffer collects conversion results from active channels on each trigger
event.

### Triggered Buffer Setup

Called from `probe()`:

```c
ret = devm_iio_triggered_buffer_setup(dev, indio_dev,
				      &iio_pollfunc_store_time,
				      adxxxx_trigger_handler,
				      NULL);
```

For sigma-delta ADCs, use the shared helper instead:

```c
ret = devm_ad_sd_setup_buffer_and_trigger(dev, indio_dev);
```

### Trigger Handler

```c
static irqreturn_t adxxxx_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct adxxxx_state *st = iio_priv(indio_dev);
	/*
	 * Buffer must be large enough for all channels + naturally
	 * aligned u64 timestamp.  Example for 4 x u16 channels:
	 */
	struct {
		u16 channels[4];
		s64 timestamp __aligned(8);
	} scan;
	int ret, i, j = 0;

	memset(&scan, 0, sizeof(scan));

	iio_for_each_active_channel(indio_dev, i) {
		unsigned int val;

		ret = adxxxx_read_reg(st, ADXXXX_REG_DATA + i, &val);
		if (ret)
			goto done;

		scan.channels[j++] = val;
	}

	iio_push_to_buffers_with_timestamp(indio_dev, &scan,
					   iio_get_time_ns(indio_dev));

done:
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}
```

### scan_type Configuration

The `scan_type` in `iio_chan_spec` tells the buffer subsystem how to pack
data:

```c
.scan_type = {
	.sign = 'u',          /* 'u' unsigned, 's' signed (bipolar) */
	.realbits = 24,       /* Actual resolution of the converter */
	.storagebits = 32,    /* Bits used in the buffer (power of 2) */
	.shift = 0,           /* Right-shift to apply to raw value */
	.endianness = IIO_BE, /* IIO_BE, IIO_LE, IIO_CPU */
},
```

Common ADC scan_type configurations:

| ADC Type                | sign | realbits | storagebits | endianness |
|-------------------------|------|----------|-------------|------------|
| 12-bit SAR, unipolar    | 'u'  | 12       | 16          | IIO_BE     |
| 16-bit SAR, unipolar    | 'u'  | 16       | 16          | IIO_BE     |
| 24-bit sigma-delta      | 'u'  | 24       | 32          | IIO_BE     |
| 24-bit sigma-delta, bipolar | 's' | 24    | 32          | IIO_BE     |
| 18-bit pipeline         | 'u'  | 18       | 32          | IIO_BE     |

### Kconfig Selects for Buffer Support

```kconfig
	select IIO_BUFFER
	select IIO_TRIGGERED_BUFFER
```

For sigma-delta ADCs, `AD_SIGMA_DELTA` already selects both:

```kconfig
config AD_SIGMA_DELTA
	tristate
	select IIO_BUFFER
	select IIO_TRIGGERED_BUFFER
```

---

## 8. DT Parsing

ADC drivers with configurable channels parse child nodes from devicetree.
Each child node describes one logical channel with its input mapping,
polarity, reference, and buffer settings.

### Parsing Channel Nodes

```c
static int adxxxx_parse_channels(struct iio_dev *indio_dev,
				 struct device *dev)
{
	struct adxxxx_state *st = iio_priv(indio_dev);
	struct iio_chan_spec *channels;
	unsigned int num_channels;
	unsigned int reg, ain[2];
	int ret;

	num_channels = device_get_child_node_count(dev);
	if (num_channels == 0)
		return dev_err_probe(dev, -ENODEV,
				     "No channel nodes defined\n");

	/* +1 for IIO_CHAN_SOFT_TIMESTAMP */
	channels = devm_kcalloc(dev, num_channels + 1,
				sizeof(*channels), GFP_KERNEL);
	if (!channels)
		return -ENOMEM;

	device_for_each_child_node_scoped(dev, child) {
		ret = fwnode_property_read_u32(child, "reg", &reg);
		if (ret)
			return dev_err_probe(dev, ret,
					     "Missing reg in %pfwP\n", child);

		if (reg >= num_channels)
			return dev_err_probe(dev, -EINVAL,
					     "Invalid reg %u in %pfwP\n",
					     reg, child);

		/* Differential channel input pair */
		ret = fwnode_property_read_u32_array(child,
						     "diff-channels",
						     ain, 2);
		if (ret)
			return dev_err_probe(dev, ret,
					     "Missing diff-channels in %pfwP\n",
					     child);

		channels[reg] = (struct iio_chan_spec) {
			.type = IIO_VOLTAGE,
			.indexed = 1,
			.differential = 1,
			.channel = ain[0],
			.channel2 = ain[1],
			.address = reg,
			.scan_index = reg,
			.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				BIT(IIO_CHAN_INFO_SCALE) |
				BIT(IIO_CHAN_INFO_OFFSET),
			.info_mask_shared_by_all =
				BIT(IIO_CHAN_INFO_SAMP_FREQ),
			.scan_type = {
				.sign = 'u',
				.realbits = st->chip_info->resolution,
				.storagebits = 32,
				.endianness = IIO_BE,
			},
		};

		/* Per-channel bipolar setting */
		if (fwnode_property_read_bool(child, "bipolar"))
			channels[reg].scan_type.sign = 's';

		/* Reference source selection */
		fwnode_property_read_u32(child, "adi,reference-select",
					&st->channel_config[reg].ref_sel);

		/* Buffered analog inputs */
		st->channel_config[reg].buf_positive =
			fwnode_property_read_bool(child,
						  "adi,buffered-positive");
		st->channel_config[reg].buf_negative =
			fwnode_property_read_bool(child,
						  "adi,buffered-negative");
	}

	/* Append software timestamp channel */
	channels[num_channels] = (struct iio_chan_spec)
		IIO_CHAN_SOFT_TIMESTAMP(num_channels);

	indio_dev->channels = channels;
	indio_dev->num_channels = num_channels + 1;

	return 0;
}
```

### Common ADC Devicetree Properties

| Property                 | API                                         | Purpose                      |
|--------------------------|---------------------------------------------|------------------------------|
| `reg`                    | `fwnode_property_read_u32()`                | Channel index / setup slot   |
| `diff-channels`         | `fwnode_property_read_u32_array()`           | Differential input pair      |
| `single-channel`        | `fwnode_property_read_u32()`                | Single-ended input           |
| `bipolar`               | `fwnode_property_read_bool()`               | Bipolar / unipolar mode      |
| `adi,reference-select`  | `fwnode_property_read_u32()`                | Reference source enum        |
| `adi,buffered-positive`  | `fwnode_property_read_bool()`              | Enable input buffer on AIN+  |
| `adi,buffered-negative`  | `fwnode_property_read_bool()`              | Enable input buffer on AIN-  |

---

## 9. Test & Debug

### Sysfs Interface

Every IIO ADC exposes channels through sysfs:

```
/sys/bus/iio/devices/iio:device0/
    name                               # Device name (e.g. "ad7124")
    in_voltage0_raw                    # Raw ADC reading, channel 0
    in_voltage0_scale                  # Scale factor (mV/LSB)
    in_voltage0_offset                 # Offset (bipolar channels)
    in_voltage0-voltage1_raw           # Differential (ch0 - ch1)
    in_voltage_scale_available         # Enumerated gain/scale values
    sampling_frequency                 # Output data rate
    sampling_frequency_available       # Supported ODR values
    oversampling_ratio                 # Hardware oversampling
```

### Manual Reads

```sh
# Read a single-ended channel
cat /sys/bus/iio/devices/iio:device0/in_voltage0_raw

# Read scale and compute voltage:  voltage_mV = raw * scale
cat /sys/bus/iio/devices/iio:device0/in_voltage0_scale

# For bipolar channels:  voltage_mV = (raw + offset) * scale
cat /sys/bus/iio/devices/iio:device0/in_voltage0_offset

# Set sampling frequency
echo 1000 > /sys/bus/iio/devices/iio:device0/sampling_frequency
```

### Buffered Reads

```sh
# List devices and channels
iio_info

# Continuous buffered read (requires triggered buffer)
iio_readdev -b 256 -s 1024 iio:device0

# Read specific channel attribute
iio_attr -d iio:device0 -c in_voltage0_raw
```

### debugfs Register Access

Requires `CONFIG_DEBUG_FS`:

```sh
# Read register 0x05
echo 0x05 > /sys/kernel/debug/iio/iio:device0/direct_reg_access
cat /sys/kernel/debug/iio/iio:device0/direct_reg_access

# Write 0x1234 to register 0x01
echo 0x01 0x1234 > /sys/kernel/debug/iio/iio:device0/direct_reg_access
```

### Kernel Debugging Tips

- Enable `CONFIG_IIO_SIMPLE_DUMMY` to study a minimal reference driver.
- Use `dev_dbg()` / `dev_info()` for debug prints; never use `printk()` in
  new drivers.
- For SPI bus debugging, enable `CONFIG_SPI_DEBUG`.
- Run `scripts/checkpatch.pl` on all patches before submitting.

---

## 10. Key Conventions

### Memory Management

Use `devm_*` (device-managed) allocations exclusively. Resources are freed
automatically when the device is removed:

| Function                             | Purpose                              |
|--------------------------------------|--------------------------------------|
| `devm_iio_device_alloc()`           | Allocate IIO device + private data   |
| `devm_iio_device_register()`        | Register IIO device (auto-unregister)|
| `devm_iio_triggered_buffer_setup()` | Set up triggered buffer              |
| `devm_regulator_get_enable()`       | Get and enable a regulator           |
| `devm_regulator_get()`              | Get regulator (manual enable/disable)|
| `devm_add_action_or_reset()`        | Register custom cleanup callback     |
| `devm_mutex_init()`                 | Initialize a mutex with devm cleanup |
| `devm_kcalloc()`                    | Allocate zeroed array                |
| `devm_request_irq()`               | Request interrupt with auto-free     |

### License

All new ADI IIO drivers must use GPL-2.0 or GPL-2.0+:

```c
// SPDX-License-Identifier: GPL-2.0+
```

The SPDX tag goes on the very first line. `MODULE_LICENSE("GPL")` at the
bottom must match.

### Scale Return Values

ADC scale is almost always returned as `IIO_VAL_FRACTIONAL_LOG2`:

```c
/* val = Vref_mV, val2 = realbits */
*val = vref_mv;
*val2 = chan->scan_type.realbits;
return IIO_VAL_FRACTIONAL_LOG2;
```

This tells userspace: `scale = val / 2^val2`, i.e. `Vref_mV / 2^N`.

For PGA gains, adjust the denominator:

```c
/* With gain: scale = Vref / (2^realbits * gain) */
*val = vref_mv;
*val2 = chan->scan_type.realbits + ilog2(gain);
return IIO_VAL_FRACTIONAL_LOG2;
```

### Return Value Reference

| Return Value                  | Meaning                                    | Usage                                |
|-------------------------------|--------------------------------------------|--------------------------------------|
| `IIO_VAL_INT`                 | Integer                                    | Result = `val`                       |
| `IIO_VAL_INT_PLUS_MICRO`     | Integer + micro fraction                   | Result = `val + val2/1000000`        |
| `IIO_VAL_INT_PLUS_NANO`      | Integer + nano fraction                    | Result = `val + val2/1000000000`     |
| `IIO_VAL_FRACTIONAL`         | Fraction                                   | Result = `val / val2`                |
| `IIO_VAL_FRACTIONAL_LOG2`    | Fraction with log2 denominator             | Result = `val / 2^val2`             |

### Mutex / Locking

- Use `struct mutex` for protecting device state and bus transactions.
- Initialize with `devm_mutex_init()` in `probe()`.
- Use `guard(mutex)(&st->lock)` (scoped lock, auto-release) where possible.
- Use `mutex_lock()` / `mutex_unlock()` when scoped locking is not suitable.
- Use `iio_device_claim_direct()` / `iio_device_release_direct()` to prevent
  raw reads while the buffer is active.

### Coding Style

- Follow the kernel coding style (`Documentation/process/coding-style.rst`).
- Run `scripts/checkpatch.pl` before submitting.
- Use `FIELD_GET()` / `FIELD_PREP()` with `GENMASK()` for register bitfields.
- Use `BIT()` for single-bit definitions.
- Use `dev_err_probe()` for all error returns in `probe()`.
- Include headers in alphabetical order within each group.

### Error Handling in probe()

```c
/* Preferred: dev_err_probe() handles -EPROBE_DEFER transparently */
ret = devm_regulator_get_enable(dev, "vdd");
if (ret)
	return dev_err_probe(dev, ret, "Failed to enable vdd\n");
```

---

## 11. Commit Format

### Subject Line Prefix

```
iio: adc: <devname>: <brief description>
```

### Examples

```
iio: adc: ad7124: add support for AD7124-4 and AD7124-8
iio: adc: ad7124: fix bipolar offset calculation
iio: adc: ad7944: add triggered buffer support
iio: adc: ad4000: use devm_iio_triggered_buffer_setup
```

### Patch Series for a New ADC Driver

A typical new ADC driver submission is a patch series:

1. `dt-bindings: iio: adc: add adi,<devname>.yaml` -- DT binding
2. `iio: adc: <devname>: add support for <DEVNAME>` -- Driver source
3. `MAINTAINERS: add entry for <DEVNAME> IIO driver` -- Maintainer entry

### Full Commit Message Example

```
iio: adc: ad7124: add support for AD7124-4 and AD7124-8

The AD7124-4/AD7124-8 are low power, low noise, 24-bit sigma-delta
analog-to-digital converters. They integrate a PGA and reference
buffers for a complete measurement solution.

This driver supports:
  - Single and continuous conversion modes
  - Configurable sample rate per channel
  - Differential input channels with per-channel PGA
  - Internal and external reference selection via devicetree
  - Triggered buffer for continuous data capture

Signed-off-by: First Last <first.last@analog.com>
```
