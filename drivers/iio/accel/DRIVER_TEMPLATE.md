# Linux IIO Accelerometer Driver Template

This template covers Linux IIO accelerometer drivers for Analog Devices parts.
It extends the consolidated IIO template (`drivers/iio/DRIVER_TEMPLATE.md`) with
accelerometer-specific channel definitions, multi-axis scan layout, event
handling for activity/freefall detection, and FIFO-based buffered capture.

Reference driver: `drivers/iio/accel/adxl380.c` (ADXL380/ADXL382).

---

## 1. Purpose -- IIO Accel

Accelerometer drivers live in `drivers/iio/accel/` and use `IIO_ACCEL` channels
with axis modifiers (`IIO_MOD_X`, `IIO_MOD_Y`, `IIO_MOD_Z`).

### Channel Configuration

| Attribute              | info_mask Level          | Notes                                       |
|------------------------|--------------------------|---------------------------------------------|
| `RAW`                  | `info_mask_separate`     | Per-axis raw reading (signed)               |
| `CALIBBIAS`            | `info_mask_separate`     | Per-axis calibration offset                 |
| `SCALE`                | `info_mask_shared_by_type` | Shared across X/Y/Z (same g-range)        |
| `SAMP_FREQ`            | `info_mask_shared_by_all` | Shared across all channels incl. temp      |
| `OFFSET`               | `info_mask_separate`     | Per-channel offset (typically temp only)    |
| `LOW_PASS_FILTER_3DB_FREQUENCY` | `info_mask_shared_by_type` | Anti-alias filter cutoff       |
| `HIGH_PASS_FILTER_3DB_FREQUENCY` | `info_mask_shared_by_type` | HP filter cutoff             |

### Modified Channels for Axis Addressing

Accelerometer channels use `.modified = 1` with `.channel2 = IIO_MOD_X/Y/Z`
instead of `.indexed = 1`. This produces sysfs names like
`in_accel_x_raw` rather than `in_accel0_raw`. The `.scan_index` field assigns
each axis a position in the buffer scan (0=X, 1=Y, 2=Z).

### Scale Convention

IIO accelerometer scale is in m/s^2 per LSB. The framework expects
`raw * scale = acceleration in m/s^2`. Typical return format is
`IIO_VAL_INT_PLUS_NANO` (e.g., `0.001307226` for a +/-4g, 16-bit device).

---

## 2. File Checklist

| File                                                                  | Action   | Required |
|-----------------------------------------------------------------------|----------|----------|
| `drivers/iio/accel/<devname>.c` (or `<devname>_core.c`)              | Create   | Yes      |
| `drivers/iio/accel/<devname>_spi.c`                                   | Create   | If SPI   |
| `drivers/iio/accel/<devname>_i2c.c`                                   | Create   | If I2C   |
| `drivers/iio/accel/<devname>.h`                                       | Create   | Multi-bus|
| `drivers/iio/accel/Kconfig`                                          | Modify   | Yes      |
| `drivers/iio/accel/Makefile`                                         | Modify   | Yes      |
| `Documentation/devicetree/bindings/iio/accel/adi,<devname>.yaml`     | Create   | Yes      |

### Multi-bus Architecture

When a device supports both SPI and I2C, split the driver into:

- **Core** (`<devname>_core.c` + `<devname>.h`): All IIO logic, channel defs,
  read_raw/write_raw, event handling, buffer ops. Uses `struct regmap *` for
  bus-agnostic register access. Exports `<devname>_probe()`.
- **SPI glue** (`<devname>_spi.c`): regmap_config for SPI, SPI probe, match
  tables. Calls `<devname>_probe(&spi->dev, regmap, chip_info)`.
- **I2C glue** (`<devname>_i2c.c`): regmap_config for I2C, I2C probe, match
  tables. Calls `<devname>_probe(&client->dev, regmap, chip_info)`.

---

## 3. DT Binding

Bindings live at `Documentation/devicetree/bindings/iio/accel/adi,<devname>.yaml`.

### SPI Bus Accelerometer

```yaml
# SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)
%YAML 1.2
---
$id: http://devicetree.org/schemas/iio/accel/adi,adxl380.yaml#
$schema: http://devicetree.org/meta-schemas/core.yaml#

title: Analog Devices ADXL380 3-Axis Accelerometer

maintainers:
  - First Last <first.last@analog.com>

description: |
  The ADXL380 is a low noise density, low power, 3-axis MEMS accelerometer
  with selectable measurement ranges of +/-4g, +/-8g, and +/-16g.

properties:
  compatible:
    enum:
      - adi,adxl380
      - adi,adxl382

  reg:
    maxItems: 1

  spi-max-frequency:
    maximum: 16000000

  interrupts:
    minItems: 1
    maxItems: 2

  interrupt-names:
    minItems: 1
    maxItems: 2
    items:
      enum:
        - INT0
        - INT1

  vddio-supply:
    description: Digital I/O supply.

  vsupply-supply:
    description: Analog supply.

required:
  - compatible
  - reg
  - interrupts
  - interrupt-names

additionalProperties: false

examples:
  - |
    #include <dt-bindings/interrupt-controller/irq.h>
    spi {
        #address-cells = <1>;
        #size-cells = <0>;

        accelerometer@0 {
            compatible = "adi,adxl380";
            reg = <0>;
            spi-max-frequency = <16000000>;
            interrupt-parent = <&gpio>;
            interrupts = <25 IRQ_TYPE_LEVEL_HIGH>;
            interrupt-names = "INT0";
            vddio-supply = <&vddio_3v3>;
            vsupply-supply = <&vsupply_1v8>;
        };
    };
```

### I2C Bus Accelerometer

```yaml
examples:
  - |
    #include <dt-bindings/interrupt-controller/irq.h>
    i2c {
        #address-cells = <1>;
        #size-cells = <0>;

        accelerometer@1d {
            compatible = "adi,adxl380";
            reg = <0x1d>;
            interrupt-parent = <&gpio>;
            interrupts = <25 IRQ_TYPE_LEVEL_HIGH>;
            interrupt-names = "INT0";
            vddio-supply = <&vddio_3v3>;
            vsupply-supply = <&vsupply_1v8>;
        };
    };
```

### Interrupt Properties

Most accelerometers provide one or two interrupt pins:

- **INT0/INT1**: DRDY (data-ready), FIFO watermark, activity/inactivity
  threshold, tap/double-tap gesture events.
- Use `interrupt-names` to let the driver select which pin to use
  (see `fwnode_irq_get_byname()` in driver code).
- Specify polarity via `IRQ_TYPE_LEVEL_HIGH` or `IRQ_TYPE_LEVEL_LOW`.

---

## 4. Kconfig

Add to `drivers/iio/accel/Kconfig` in alphabetical order. For multi-bus
drivers, use a base `tristate` symbol selected by bus-specific entries.

### Multi-bus Pattern (SPI + I2C)

```kconfig
config ADXL380
	tristate
	select IIO_BUFFER
	select IIO_TRIGGERED_BUFFER

config ADXL380_SPI
	tristate "Analog Devices ADXL380 3-Axis Accelerometer SPI Driver"
	depends on SPI
	select ADXL380
	select REGMAP_SPI
	help
	  Say yes here to add support for the Analog Devices ADXL380 triaxial
	  acceleration sensor.
	  To compile this driver as a module, choose M here: the
	  module will be called adxl380_spi.

config ADXL380_I2C
	tristate "Analog Devices ADXL380 3-Axis Accelerometer I2C Driver"
	depends on I2C
	select ADXL380
	select REGMAP_I2C
	help
	  Say yes here to add support for the Analog Devices ADXL380 triaxial
	  acceleration sensor.
	  To compile this driver as a module, choose M here: the
	  module will be called adxl380_i2c.
```

### Single-bus Pattern (SPI Only)

```kconfig
config ADXL345
	tristate "Analog Devices ADXL345 3-Axis Accelerometer Driver"
	depends on SPI_MASTER
	select IIO_BUFFER
	select IIO_TRIGGERED_BUFFER
	select REGMAP_SPI
	help
	  Say yes here to build support for the Analog Devices ADXL345
	  3-axis digital accelerometer.

	  To compile this driver as a module, choose M here: the module
	  will be called adxl345.
```

### Common Selects for Accelerometers

| Select                    | When to Use                                     |
|---------------------------|-------------------------------------------------|
| `IIO_BUFFER`              | Any buffered/FIFO capture                       |
| `IIO_TRIGGERED_BUFFER`    | Trigger-driven (DRDY, hrtimer) buffer capture   |
| `IIO_BUFFER_DMAENGINE`   | DMA-based buffer for high-rate devices          |
| `REGMAP_SPI` / `REGMAP_I2C` | regmap bus abstraction (standard practice)    |

---

## 5. Makefile

Add to `drivers/iio/accel/Makefile` in alphabetical order.

### Multi-bus Driver

```makefile
obj-$(CONFIG_ADXL380) += adxl380.o
obj-$(CONFIG_ADXL380_I2C) += adxl380_i2c.o
obj-$(CONFIG_ADXL380_SPI) += adxl380_spi.o
```

### Single-file Driver

```makefile
obj-$(CONFIG_ADXL345) += adxl345.o
```

---

## 6. Driver Source

### Accel Channel Macro (IIO_ACCEL with Axis Modifiers)

```c
#define ADXL380_ACCEL_CHANNEL(index, reg, axis) {			\
	.type = IIO_ACCEL,						\
	.address = reg,							\
	.modified = 1,							\
	.channel2 = IIO_MOD_##axis,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_CALIBBIAS),		\
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.info_mask_shared_by_all_available =				\
		BIT(IIO_CHAN_INFO_SAMP_FREQ),				\
	.info_mask_shared_by_type =					\
		BIT(IIO_CHAN_INFO_SCALE),				\
	.info_mask_shared_by_type_available =				\
		BIT(IIO_CHAN_INFO_SCALE),				\
	.scan_index = index,						\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = 16,						\
		.storagebits = 16,					\
		.endianness = IIO_BE,					\
	},								\
	.event_spec = adxl380_events,					\
	.num_event_specs = ARRAY_SIZE(adxl380_events)			\
}
```

Key design points:

- **`.modified = 1`**: Uses `IIO_MOD_X/Y/Z` axis modifiers instead of
  `.indexed`. This is the standard pattern for accelerometers.
- **`.info_mask_separate`**: RAW and CALIBBIAS are per-axis (each axis has
  its own register).
- **`.info_mask_shared_by_type`**: SCALE is shared across all `IIO_ACCEL`
  channels (changing the g-range affects all axes equally).
- **`.info_mask_shared_by_all`**: SAMP_FREQ is shared across accel + temp.
- **`.scan_index`**: Assigns buffer position (0=X, 1=Y, 2=Z). Must be unique
  and sequential for the FIFO read logic.
- **`.scan_type.sign = 's'`**: Accelerometer data is signed (bipolar).

### Channel Array

```c
static const struct iio_chan_spec adxl380_channels[] = {
	ADXL380_ACCEL_CHANNEL(0, ADXL380_X_DATA_H_REG, X),
	ADXL380_ACCEL_CHANNEL(1, ADXL380_Y_DATA_H_REG, Y),
	ADXL380_ACCEL_CHANNEL(2, ADXL380_Z_DATA_H_REG, Z),
	{
		.type = IIO_TEMP,
		.address = ADXL380_T_DATA_H_REG,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE) |
				      BIT(IIO_CHAN_INFO_OFFSET),
		.scan_index = 3,
		.scan_type = {
			.sign = 's',
			.realbits = 12,
			.storagebits = 16,
			.shift = 4,
			.endianness = IIO_BE,
		},
	},
};
```

### Device State Structure

```c
struct adxxxx_state {
	struct regmap		*regmap;
	struct device		*dev;
	const struct adxxxx_chip_info *chip_info;
	/*
	 * Synchronize access to members of driver state, and ensure
	 * atomicity of consecutive regmap operations.
	 */
	struct mutex		lock;
	u8			range;
	u8			odr;
	u8			fifo_set_size;
	u16			watermark;
	u32			act_threshold;
	u32			inact_threshold;
	int			irq;
	int			int_map[2];

	__be16			fifo_buf[MAX_FIFO_SAMPLES]
					__aligned(IIO_DMA_MINALIGN);
};
```

### read_raw for Accelerometer Channels

```c
static int adxxxx_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long info)
{
	struct adxxxx_state *st = iio_priv(indio_dev);
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;

		ret = adxxxx_read_chn(st, chan->address);
		iio_device_release_direct_mode(indio_dev);
		if (ret < 0)
			return ret;

		/* Sign-extend based on scan_type */
		*val = sign_extend32(ret >> chan->scan_type.shift,
				     chan->scan_type.realbits - 1);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_ACCEL:
			scoped_guard(mutex, &st->lock) {
				*val = st->chip_info->scale_tbl[st->range][0];
				*val2 = st->chip_info->scale_tbl[st->range][1];
			}
			/* m/s^2 per LSB, expressed as int + nano */
			return IIO_VAL_INT_PLUS_NANO;
		case IIO_TEMP:
			/* millidegrees Celsius per LSB */
			*val = 10000;
			*val2 = 102;
			return IIO_VAL_FRACTIONAL;
		default:
			return -EINVAL;
		}

	case IIO_CHAN_INFO_CALIBBIAS:
		if (chan->type != IIO_ACCEL)
			return -EINVAL;
		ret = adxxxx_read_calibbias(st, chan->scan_index, val);
		if (ret)
			return ret;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = adxxxx_get_odr(st, val);
		if (ret)
			return ret;
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}
```

### Probe with WHO_AM_I Check

```c
static int adxxxx_setup(struct iio_dev *indio_dev)
{
	struct adxxxx_state *st = iio_priv(indio_dev);
	unsigned int reg_val;
	int ret;

	/* Read DEVID register (WHO_AM_I) */
	ret = regmap_read(st->regmap, ADXXXX_DEVID_REG, &reg_val);
	if (ret)
		return ret;

	if (reg_val != ADXXXX_DEVID_VAL)
		dev_warn(st->dev, "Unknown chip id 0x%x\n", reg_val);

	/* Software reset */
	ret = regmap_write(st->regmap, ADXXXX_RESET_REG, ADXXXX_RESET_CODE);
	if (ret)
		return ret;

	/* Wait for reset to complete (see datasheet for timing) */
	fsleep(500);

	/* Enable channels, configure FIFO, set default range/ODR */
	/* ... */

	return 0;
}

int adxxxx_probe(struct device *dev, struct regmap *regmap,
		 const struct adxxxx_chip_info *chip_info)
{
	struct iio_dev *indio_dev;
	struct adxxxx_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->dev = dev;
	st->regmap = regmap;
	st->chip_info = chip_info;

	mutex_init(&st->lock);

	indio_dev->channels = adxxxx_channels;
	indio_dev->num_channels = ARRAY_SIZE(adxxxx_channels);
	indio_dev->name = chip_info->name;
	indio_dev->info = &adxxxx_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	/* Enable power supplies */
	ret = devm_regulator_get_enable(dev, "vddio");
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to enable vddio supply\n");

	ret = devm_regulator_get_enable(dev, "vsupply");
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to enable vsupply supply\n");

	/* Hardware init: WHO_AM_I, reset, configure defaults */
	ret = adxxxx_setup(indio_dev);
	if (ret)
		return ret;

	/* Set up FIFO-backed kfifo buffer */
	ret = devm_iio_kfifo_buffer_setup_ext(dev, indio_dev,
					      &adxxxx_buffer_ops,
					      adxxxx_fifo_attributes);
	if (ret)
		return ret;

	return devm_iio_device_register(dev, indio_dev);
}
EXPORT_SYMBOL_NS_GPL(adxxxx_probe, IIO_ADXXXX);
```

### SPI Glue Driver

```c
// SPDX-License-Identifier: GPL-2.0+
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>

#include "adxxxx.h"

static const struct regmap_config adxxxx_spi_regmap_config = {
	.reg_bits = 7,
	.pad_bits = 1,
	.val_bits = 8,
	.read_flag_mask = BIT(0),
	.readable_noinc_reg = adxxxx_readable_noinc_reg,
};

static int adxxxx_spi_probe(struct spi_device *spi)
{
	const struct adxxxx_chip_info *chip_data;
	struct regmap *regmap;

	chip_data = spi_get_device_match_data(spi);

	regmap = devm_regmap_init_spi(spi, &adxxxx_spi_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	return adxxxx_probe(&spi->dev, regmap, chip_data);
}

static const struct spi_device_id adxxxx_spi_id[] = {
	{ "adxxxx", (kernel_ulong_t)&adxxxx_chip_info },
	{ }
};
MODULE_DEVICE_TABLE(spi, adxxxx_spi_id);

static const struct of_device_id adxxxx_of_match[] = {
	{ .compatible = "adi,adxxxx", .data = &adxxxx_chip_info },
	{ }
};
MODULE_DEVICE_TABLE(of, adxxxx_of_match);

static struct spi_driver adxxxx_spi_driver = {
	.driver = {
		.name = "adxxxx_spi",
		.of_match_table = adxxxx_of_match,
	},
	.probe = adxxxx_spi_probe,
	.id_table = adxxxx_spi_id,
};
module_spi_driver(adxxxx_spi_driver);

MODULE_AUTHOR("First Last <first.last@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADXXXX 3-axis accelerometer SPI driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(IIO_ADXXXX);
```

---

## 7. Buffer & Trigger

### Hardware FIFO vs Triggered Buffer

Most ADI accelerometers have an on-chip FIFO. Two buffer strategies exist:

1. **Hardware FIFO + kfifo** (preferred for FIFO-equipped devices like
   ADXL380): The device FIFO fills autonomously. A watermark interrupt fires
   when N samples are available. The IRQ handler bulk-reads the FIFO and pushes
   to the IIO kfifo buffer. Use `devm_iio_kfifo_buffer_setup_ext()`.

2. **Triggered buffer** (for devices without FIFO): A DRDY interrupt triggers
   the poll function, which reads one sample set (X, Y, Z) and pushes to the
   buffer with a timestamp. Use `devm_iio_triggered_buffer_setup()`.

### FIFO-based IRQ Handler

```c
static irqreturn_t adxxxx_irq_handler(int irq, void *p)
{
	struct iio_dev *indio_dev = p;
	struct adxxxx_state *st = iio_priv(indio_dev);
	u8 status0, status1;
	u16 fifo_entries;
	int ret, i;

	guard(mutex)(&st->lock);

	ret = adxxxx_get_status(st, &status0, &status1);
	if (ret)
		return IRQ_HANDLED;

	/* Push threshold/tap events (see Section 8) */
	adxxxx_push_event(indio_dev, iio_get_time_ns(indio_dev), status1);

	if (!FIELD_GET(STATUS0_FIFO_WM_MSK, status0))
		return IRQ_HANDLED;

	ret = adxxxx_get_fifo_entries(st, &fifo_entries);
	if (ret)
		return IRQ_HANDLED;

	for (i = 0; i < fifo_entries; i += st->fifo_set_size) {
		ret = regmap_noinc_read(st->regmap, ADXXXX_FIFO_DATA,
					&st->fifo_buf[i],
					2 * st->fifo_set_size);
		if (ret)
			return IRQ_HANDLED;
		iio_push_to_buffers(indio_dev, &st->fifo_buf[i]);
	}

	return IRQ_HANDLED;
}
```

### Triggered Buffer Handler (for devices without FIFO)

```c
static irqreturn_t adxxxx_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct adxxxx_state *st = iio_priv(indio_dev);
	struct {
		s16 channels[3];	/* X, Y, Z */
		s64 timestamp __aligned(8);
	} scan;
	int ret, i, j = 0;

	memset(&scan, 0, sizeof(scan));

	iio_for_each_active_channel(indio_dev, i) {
		ret = adxxxx_read_axis(st, i, &scan.channels[j++]);
		if (ret)
			goto done;
	}

	iio_push_to_buffers_with_timestamp(indio_dev, &scan,
					   iio_get_time_ns(indio_dev));
done:
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}
```

### DRDY Trigger Setup

```c
/* In probe, for triggered buffer approach: */
st->drdy_trig = devm_iio_trigger_alloc(dev, "%s-dev%d",
					indio_dev->name,
					iio_device_id(indio_dev));
if (!st->drdy_trig)
	return -ENOMEM;

st->drdy_trig->ops = &adxxxx_trigger_ops;
iio_trigger_set_drvdata(st->drdy_trig, indio_dev);

ret = devm_iio_trigger_register(dev, st->drdy_trig);
if (ret)
	return dev_err_probe(dev, ret, "Failed to register trigger\n");

indio_dev->trig = iio_trigger_get(st->drdy_trig);

ret = devm_iio_triggered_buffer_setup(dev, indio_dev,
				      &iio_pollfunc_store_time,
				      adxxxx_trigger_handler, NULL);
```

### scan_type for Accelerometer Axes

```c
.scan_type = {
	.sign = 's',          /* Acceleration is signed (bipolar) */
	.realbits = 16,       /* ADC resolution (e.g., 16-bit) */
	.storagebits = 16,    /* Must be power of 2 >= realbits */
	.shift = 0,           /* Right-shift if data is left-justified */
	.endianness = IIO_BE, /* Big-endian from SPI/I2C read */
},
```

Common configurations:

| Device   | realbits | storagebits | shift | Notes                      |
|----------|----------|-------------|-------|----------------------------|
| ADXL345  | 13       | 16          | 0     | Right-justified            |
| ADXL355  | 20       | 32          | 4     | Left-justified in 24-bit  |
| ADXL380  | 16       | 16          | 0     | Full 16-bit               |

### Buffer Setup Ops (FIFO-based)

```c
static int adxxxx_buffer_postenable(struct iio_dev *indio_dev)
{
	struct adxxxx_state *st = iio_priv(indio_dev);

	guard(mutex)(&st->lock);

	/* 1. Go to standby */
	/* 2. Enable FIFO watermark interrupt mapping */
	/* 3. Disable channels not in active_scan_mask */
	/* 4. Calculate fifo_set_size from active channels */
	/* 5. Set watermark in hardware */
	/* 6. Enable FIFO */
	/* 7. Resume measurement */
	return 0;
}

static int adxxxx_buffer_predisable(struct iio_dev *indio_dev)
{
	struct adxxxx_state *st = iio_priv(indio_dev);

	guard(mutex)(&st->lock);

	/* 1. Go to standby */
	/* 2. Disable FIFO watermark interrupt */
	/* 3. Re-enable all channels */
	/* 4. Disable FIFO */
	/* 5. Resume measurement */
	return 0;
}

static const struct iio_buffer_setup_ops adxxxx_buffer_ops = {
	.postenable = adxxxx_buffer_postenable,
	.predisable = adxxxx_buffer_predisable,
};
```

---

## 8. Event Support

### Event Types for Accelerometers

| Event Type              | Direction        | Use Case                        |
|-------------------------|------------------|---------------------------------|
| `IIO_EV_TYPE_THRESH`   | `RISING`         | Activity detection              |
| `IIO_EV_TYPE_THRESH`   | `FALLING`        | Inactivity / freefall detection |
| `IIO_EV_TYPE_GESTURE`  | `SINGLETAP`      | Single tap detection            |
| `IIO_EV_TYPE_GESTURE`  | `DOUBLETAP`      | Double tap detection            |

### Event Spec Definition

```c
static const struct iio_event_spec adxxxx_events[] = {
	{
		/* Activity: acceleration exceeds threshold */
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_shared_by_type = BIT(IIO_EV_INFO_ENABLE) |
				       BIT(IIO_EV_INFO_VALUE) |
				       BIT(IIO_EV_INFO_PERIOD),
	},
	{
		/* Inactivity / Freefall: acceleration below threshold */
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_shared_by_type = BIT(IIO_EV_INFO_ENABLE) |
				       BIT(IIO_EV_INFO_VALUE) |
				       BIT(IIO_EV_INFO_PERIOD),
	},
	{
		/* Single tap gesture */
		.type = IIO_EV_TYPE_GESTURE,
		.dir = IIO_EV_DIR_SINGLETAP,
		.mask_separate = BIT(IIO_EV_INFO_ENABLE),
		.mask_shared_by_type = BIT(IIO_EV_INFO_VALUE) |
				       BIT(IIO_EV_INFO_RESET_TIMEOUT),
	},
	{
		/* Double tap gesture */
		.type = IIO_EV_TYPE_GESTURE,
		.dir = IIO_EV_DIR_DOUBLETAP,
		.mask_separate = BIT(IIO_EV_INFO_ENABLE),
		.mask_shared_by_type = BIT(IIO_EV_INFO_VALUE) |
				       BIT(IIO_EV_INFO_RESET_TIMEOUT) |
				       BIT(IIO_EV_INFO_TAP2_MIN_DELAY),
	},
};
```

### Pushing Events from IRQ Handler

```c
static void adxxxx_push_event(struct iio_dev *indio_dev, s64 timestamp,
			      u8 status)
{
	/* Activity detection */
	if (FIELD_GET(STATUS_ACT_MSK, status))
		iio_push_event(indio_dev,
			       IIO_MOD_EVENT_CODE(IIO_ACCEL, 0,
						  IIO_MOD_X_OR_Y_OR_Z,
						  IIO_EV_TYPE_THRESH,
						  IIO_EV_DIR_RISING),
			       timestamp);

	/* Inactivity / freefall */
	if (FIELD_GET(STATUS_INACT_MSK, status))
		iio_push_event(indio_dev,
			       IIO_MOD_EVENT_CODE(IIO_ACCEL, 0,
						  IIO_MOD_X_OR_Y_OR_Z,
						  IIO_EV_TYPE_THRESH,
						  IIO_EV_DIR_FALLING),
			       timestamp);

	/* Single tap */
	if (FIELD_GET(STATUS_SINGLE_TAP_MSK, status))
		iio_push_event(indio_dev,
			       IIO_MOD_EVENT_CODE(IIO_ACCEL, 0,
						  IIO_MOD_X_OR_Y_OR_Z,
						  IIO_EV_TYPE_GESTURE,
						  IIO_EV_DIR_SINGLETAP),
			       timestamp);

	/* Double tap */
	if (FIELD_GET(STATUS_DOUBLE_TAP_MSK, status))
		iio_push_event(indio_dev,
			       IIO_MOD_EVENT_CODE(IIO_ACCEL, 0,
						  IIO_MOD_X_OR_Y_OR_Z,
						  IIO_EV_TYPE_GESTURE,
						  IIO_EV_DIR_DOUBLETAP),
			       timestamp);
}
```

### iio_info Event Callbacks

```c
static const struct iio_info adxxxx_info = {
	.read_raw = adxxxx_read_raw,
	.read_avail = adxxxx_read_avail,
	.write_raw = adxxxx_write_raw,
	.write_raw_get_fmt = adxxxx_write_raw_get_fmt,
	.read_event_config = adxxxx_read_event_config,
	.write_event_config = adxxxx_write_event_config,
	.read_event_value = adxxxx_read_event_value,
	.write_event_value = adxxxx_write_event_value,
	.event_attrs = &adxxxx_event_attribute_group,
	.debugfs_reg_access = adxxxx_reg_access,
	.hwfifo_set_watermark = adxxxx_set_watermark,
};
```

### Event Value Read/Write

Event values map to hardware threshold/timing registers:

| Event Info                | Threshold Event       | Gesture Event              |
|---------------------------|-----------------------|----------------------------|
| `IIO_EV_INFO_VALUE`      | Act/inact threshold   | Tap threshold              |
| `IIO_EV_INFO_PERIOD`     | Act/inact time        | --                         |
| `IIO_EV_INFO_RESET_TIMEOUT` | --                 | Tap window timeout         |
| `IIO_EV_INFO_TAP2_MIN_DELAY` | --                | Double-tap latency         |

---

## 9. Test & Debug

### Sysfs Interface

```
/sys/bus/iio/devices/iio:device0/
    name                                # "adxl380"
    in_accel_x_raw                      # X-axis raw reading (signed)
    in_accel_y_raw                      # Y-axis raw reading
    in_accel_z_raw                      # Z-axis raw reading
    in_accel_x_calibbias                # X-axis calibration offset
    in_accel_y_calibbias                # Y-axis calibration offset
    in_accel_z_calibbias                # Z-axis calibration offset
    in_accel_scale                      # Shared scale (m/s^2 per LSB)
    in_accel_scale_available             # Available scale values
    sampling_frequency                  # ODR in Hz
    sampling_frequency_available         # Available ODR values
    in_temp_raw                          # Temperature raw (if present)
    in_temp_scale                        # Temperature scale
    in_temp_offset                       # Temperature offset
```

Note: Because accel channels use `.modified = 1`, sysfs names are
`in_accel_x_raw` (axis letter) not `in_accel0_raw` (numeric index).

### Event Sysfs

```
/sys/bus/iio/devices/iio:device0/events/
    in_accel_thresh_rising_en            # Enable activity detection
    in_accel_thresh_rising_value         # Activity threshold
    in_accel_thresh_rising_period        # Activity time window
    in_accel_thresh_falling_en           # Enable inactivity detection
    in_accel_thresh_falling_value        # Inactivity threshold
    in_accel_thresh_falling_period       # Inactivity time window
    in_accel_x_gesture_singletap_en      # Enable X-axis single tap
    in_accel_y_gesture_singletap_en      # Enable Y-axis single tap
    in_accel_z_gesture_singletap_en      # Enable Z-axis single tap
    in_accel_gesture_singletap_value     # Tap threshold
    in_accel_x_gesture_doubletap_en      # Enable X-axis double tap
```

### Reading Buffered Data

```sh
# Enable X, Y, Z channels in the buffer scan
echo 1 > /sys/bus/iio/devices/iio:device0/scan_elements/in_accel_x_en
echo 1 > /sys/bus/iio/devices/iio:device0/scan_elements/in_accel_y_en
echo 1 > /sys/bus/iio/devices/iio:device0/scan_elements/in_accel_z_en

# Set buffer length and enable
echo 128 > /sys/bus/iio/devices/iio:device0/buffer/length
echo 1 > /sys/bus/iio/devices/iio:device0/buffer/enable

# Read buffered data
cat /dev/iio:device0 | xxd | head

# Or use iio_readdev
iio_readdev -b 256 -s 1024 iio:device0
```

### Monitoring Events

```sh
# Use iio_event_monitor to watch for activity/tap events
iio_event_monitor iio:device0

# Example output:
# Event: time: 1234567890, type: thresh(rising), channel: in_accel
# Event: time: 1234567891, type: gesture(singletap), channel: in_accel_x
```

### debugfs Register Access

```sh
# Read WHO_AM_I register (0x00)
echo 0x00 > /sys/kernel/debug/iio/iio:device0/direct_reg_access
cat /sys/kernel/debug/iio/iio:device0/direct_reg_access

# Write to control register
echo 0x26 0x0C > /sys/kernel/debug/iio/iio:device0/direct_reg_access
```

---

## 10. Key Conventions

### Memory Management

Use `devm_*` exclusively. Critical functions for accel drivers:

| Function                          | Purpose                                  |
|-----------------------------------|------------------------------------------|
| `devm_iio_device_alloc()`        | Allocate IIO device + private state      |
| `devm_iio_device_register()`     | Register IIO device                      |
| `devm_iio_kfifo_buffer_setup_ext()` | Hardware FIFO + kfifo buffer          |
| `devm_iio_triggered_buffer_setup()` | Triggered buffer (no hw FIFO)         |
| `devm_regulator_get_enable()`    | Power supply management                  |
| `devm_request_threaded_irq()`    | IRQ for FIFO watermark / events          |
| `devm_regmap_init_spi/i2c()`    | Bus-agnostic register access             |
| `devm_iio_trigger_alloc()`      | DRDY trigger allocation                  |

### License

```c
// SPDX-License-Identifier: GPL-2.0+
/* ... */
MODULE_LICENSE("GPL");
```

### Scale Convention (m/s^2)

IIO accelerometer scale must produce values in m/s^2 (not g). The conversion
is: `scale_m_s2 = (range_g * 9.80665) / 2^realbits`. The scale is returned
as `IIO_VAL_INT_PLUS_NANO`. For example, ADXL380 at +/-4g, 16-bit:

```
scale = (4 * 9.80665) / 65536 = 0.000598550 m/s^2/LSB
```

Returned as `*val = 0, *val2 = 598550` (nano).

### Locking

- `guard(mutex)(&st->lock)` for scoped locking (auto-release at scope exit).
- `scoped_guard(mutex, &st->lock) { ... }` for locking a specific block.
- `iio_device_claim_direct_mode()` / `iio_device_release_direct_mode()` to
  prevent raw reads while the buffer is active.
- Always go to standby before changing configuration registers, then resume.

### Coding Style

- `FIELD_GET()` / `FIELD_PREP()` with `GENMASK()` for register bitfields.
- `sign_extend32()` for sign-extending raw readings.
- `get_unaligned_be16()` / `put_unaligned_be24()` for multi-byte register
  access via regmap bulk reads.
- `dev_err_probe()` for all probe-time error returns.
- `dev_warn()` for non-fatal issues like unexpected WHO_AM_I values.

---

## 11. Commit Format

### Subject Line Prefix

```
iio: accel: <devname>: <brief description>
```

### Examples

```
iio: accel: adxl380: add support for ADXL380 and ADXL382
iio: accel: adxl380: fix activity threshold scaling on range change
iio: accel: adxl345: add triggered buffer support
iio: accel: adxl355: convert to devm_iio_device_register
```

### Patch Series for a New Accelerometer Driver

1. `dt-bindings: iio: accel: add adi,adxl380.yaml`
2. `iio: accel: adxl380: add support for ADXL380 and ADXL382`
3. `MAINTAINERS: add entry for ADXL380 IIO driver`

### Full Commit Example

```
iio: accel: adxl380: add support for ADXL380 and ADXL382

The ADXL380/ADXL382 are low noise, low power, 3-axis MEMS
accelerometers. The ADXL380 supports ranges of +/-4g, +/-8g, +/-16g
and the ADXL382 supports +/-15g, +/-30g, +/-60g.

This driver supports:
  - SPI and I2C bus interfaces via regmap
  - Configurable output data rate and g-range
  - Per-axis calibration bias registers
  - Hardware FIFO with configurable watermark
  - Activity/inactivity threshold detection events
  - Single-tap and double-tap gesture events

Signed-off-by: First Last <first.last@analog.com>
```
