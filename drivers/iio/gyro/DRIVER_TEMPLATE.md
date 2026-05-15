# Linux IIO Gyroscope Driver Template

Reference drivers: `drivers/iio/gyro/adxrs290.c`, `drivers/iio/gyro/bmg160_core.c`

This template covers the files, code skeletons, and conventions needed to write
a Linux kernel IIO gyroscope driver. Gyroscopes use `IIO_ANGL_VEL` channels
with `IIO_MOD_X`, `IIO_MOD_Y`, `IIO_MOD_Z` axis modifiers and report angular
velocity in rad/s. Replace `<devname>` with the part number (e.g., `adxrs290`)
and `<DEVNAME>` with its uppercase form (e.g., `ADXRS290`) throughout.

---

## 1. Purpose

Gyroscope drivers live under `drivers/iio/gyro/` and expose angular velocity
data through the IIO subsystem.

### Channel Configuration

| Channel Type   | Modifier           | Count | Purpose              |
|----------------|--------------------|-------|----------------------|
| `IIO_ANGL_VEL` | `IIO_MOD_X/Y/Z`  | 3     | Angular velocity axes |
| `IIO_TEMP`     | none (optional)    | 0-1   | Die temperature       |

### info_mask Bits

| Bit                                           | Scope                      | Purpose                        |
|-----------------------------------------------|----------------------------|--------------------------------|
| `IIO_CHAN_INFO_RAW`                            | `info_mask_separate`       | Raw axis reading               |
| `IIO_CHAN_INFO_SCALE`                          | `info_mask_shared_by_type` | Scale factor (rad/s per LSB)   |
| `IIO_CHAN_INFO_SAMP_FREQ`                      | `info_mask_shared_by_type` | Output data rate               |
| `IIO_CHAN_INFO_CALIBBIAS`                       | `info_mask_separate`       | Per-axis offset calibration    |
| `IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY`  | `info_mask_shared_by_type` | Low-pass filter cutoff (opt.)  |
| `IIO_CHAN_INFO_HIGH_PASS_FILTER_3DB_FREQUENCY` | `info_mask_shared_by_type` | High-pass filter cutoff (opt.) |

### Units

Angular velocity is reported in **rad/s**. Hardware that provides degrees/sec
must convert in the scale callback:

```
rad/s = deg/s * (pi / 180)
```

For example, if 1 LSB = 0.005 deg/s, then scale = 0.005 * pi / 180 = 0.000087266 rad/s.

Temperature channels (die temp) report in **millidegrees Celsius**.

---

## 2. File Checklist

| File                                                            | Action | Required |
|-----------------------------------------------------------------|--------|----------|
| `drivers/iio/gyro/<devname>.c`                                  | Create | Yes      |
| `drivers/iio/gyro/Kconfig`                                      | Modify | Yes      |
| `drivers/iio/gyro/Makefile`                                      | Modify | Yes      |
| `Documentation/devicetree/bindings/iio/gyroscope/adi,<devname>.yaml` | Create | Yes |
| `MAINTAINERS`                                                    | Modify | Yes      |

For drivers that support both SPI and I2C, split into multiple files:

| File                                | Purpose                    |
|-------------------------------------|----------------------------|
| `drivers/iio/gyro/<devname>.h`      | Shared header              |
| `drivers/iio/gyro/<devname>_core.c` | Core driver logic          |
| `drivers/iio/gyro/<devname>_spi.c`  | SPI bus glue               |
| `drivers/iio/gyro/<devname>_i2c.c`  | I2C bus glue               |

---

## 3. Devicetree Binding

Bindings live under `Documentation/devicetree/bindings/iio/gyroscope/adi,<devname>.yaml`.

### SPI Gyroscope

```yaml
# SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)
%YAML 1.2
---
$id: http://devicetree.org/schemas/iio/gyroscope/adi,<devname>.yaml#
$schema: http://devicetree.org/meta-schemas/core.yaml#

title: Analog Devices <DEVNAME> Gyroscope

maintainers:
  - First Last <first.last@analog.com>

description: |
  The <DEVNAME> is a high performance digital gyroscope with
  SPI interface.

properties:
  compatible:
    enum:
      - adi,<devname>

  reg:
    maxItems: 1

  spi-max-frequency:
    maximum: 10000000

  interrupts:
    maxItems: 1
    description: Data-ready interrupt (active high or low).

  vdd-supply: true
  vddio-supply: true

  mount-matrix:
    description: Mounting rotation matrix (see iio-bindings.txt).

required:
  - compatible
  - reg

additionalProperties: false

examples:
  - |
    spi {
        #address-cells = <1>;
        #size-cells = <0>;

        gyroscope@0 {
            compatible = "adi,<devname>";
            reg = <0>;
            spi-max-frequency = <10000000>;
            interrupt-parent = <&gpio>;
            interrupts = <25 IRQ_TYPE_EDGE_RISING>;
            vdd-supply = <&vdd_3v3>;
        };
    };
```

### I2C Gyroscope

```yaml
examples:
  - |
    i2c {
        #address-cells = <1>;
        #size-cells = <0>;

        gyroscope@68 {
            compatible = "adi,<devname>";
            reg = <0x68>;
            interrupt-parent = <&gpio>;
            interrupts = <5 IRQ_TYPE_EDGE_RISING>;
            vdd-supply = <&vdd_3v3>;
            mount-matrix = "0", "1", "0",
                           "1", "0", "0",
                           "0", "0", "-1";
        };
    };
```

### Key Properties for Gyroscopes

| Property         | Purpose                                            |
|------------------|----------------------------------------------------|
| `interrupts`     | Data-ready or FIFO watermark interrupt GPIO         |
| `mount-matrix`   | 3x3 rotation matrix for board mounting orientation  |
| `vdd-supply`     | Main power supply                                   |
| `vddio-supply`   | I/O level power supply                              |
| `drive-open-drain` | Interrupt pin is open-drain                       |

---

## 4. Kconfig

Add the entry to `drivers/iio/gyro/Kconfig` in **alphabetical order**.

### SPI-Only Device

```kconfig
config <DEVNAME>
	tristate "Analog Devices <DEVNAME> Gyroscope SPI driver"
	depends on SPI
	select IIO_BUFFER
	select IIO_TRIGGERED_BUFFER
	help
	  Say yes here to build support for Analog Devices <DEVNAME>
	  digital output gyroscope.

	  To compile this driver as a module, choose M here: the module
	  will be called <devname>.
```

### I2C-Only Device

```kconfig
config <DEVNAME>
	tristate "Analog Devices <DEVNAME> Gyroscope I2C driver"
	depends on I2C
	select REGMAP_I2C
	select IIO_BUFFER
	select IIO_TRIGGERED_BUFFER
	help
	  Say yes here to build support for Analog Devices <DEVNAME>
	  digital output gyroscope.

	  To compile this driver as a module, choose M here: the module
	  will be called <devname>.
```

### Dual SPI/I2C Device

```kconfig
config <DEVNAME>
	tristate "Analog Devices <DEVNAME> Gyroscope driver"
	depends on (I2C || SPI_MASTER)
	select IIO_BUFFER
	select IIO_TRIGGERED_BUFFER
	select <DEVNAME>_I2C if (I2C)
	select <DEVNAME>_SPI if (SPI)
	help
	  Say yes here to build support for Analog Devices <DEVNAME>
	  tri-axis digital output gyroscope connected via I2C or SPI.

	  To compile this driver as a module, choose M here: the module
	  will be called <devname>_i2c or <devname>_spi.

config <DEVNAME>_I2C
	tristate
	select REGMAP_I2C

config <DEVNAME>_SPI
	tristate
	select REGMAP_SPI
```

---

## 5. Makefile

Add the entry to `drivers/iio/gyro/Makefile` in **alphabetical order**.

### Single-File Driver

```makefile
obj-$(CONFIG_<DEVNAME>) += <devname>.o
```

### Multi-File (SPI + I2C) Driver

```makefile
obj-$(CONFIG_<DEVNAME>) += <devname>_core.o
obj-$(CONFIG_<DEVNAME>_I2C) += <devname>_i2c.o
obj-$(CONFIG_<DEVNAME>_SPI) += <devname>_spi.o
```

---

## 6. Driver Source

### Complete Skeleton (SPI Gyroscope)

```c
// SPDX-License-Identifier: GPL-2.0+
/*
 * <DEVNAME> SPI Gyroscope driver
 *
 * Copyright 2024 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

/* ------------------------------------------------------------------ */
/* Register Definitions                                                */
/* ------------------------------------------------------------------ */

#define <DEVNAME>_REG_WHO_AM_I		0x00
#define <DEVNAME>_REG_DATAX0		0x08
#define <DEVNAME>_REG_DATAY0		0x0A
#define <DEVNAME>_REG_DATAZ0		0x0C
#define <DEVNAME>_REG_TEMP		0x0E
#define <DEVNAME>_REG_POWER_CTL		0x10
#define <DEVNAME>_REG_FILTER		0x11

#define <DEVNAME>_WHO_AM_I_VAL		0xAD

/* ------------------------------------------------------------------ */
/* Device State                                                        */
/* ------------------------------------------------------------------ */

enum <devname>_scan_index {
	<DEVNAME>_IDX_X,
	<DEVNAME>_IDX_Y,
	<DEVNAME>_IDX_Z,
	<DEVNAME>_IDX_TEMP,	/* optional */
	<DEVNAME>_IDX_TS,
};

struct <devname>_chip_info {
	const char		*name;
	unsigned int		chip_id;
};

struct <devname>_state {
	struct spi_device	*spi;
	const struct <devname>_chip_info *chip_info;
	struct mutex		lock;	/* Protect device state */
	struct iio_mount_matrix	orientation;
	struct iio_trigger	*dready_trig;
	/*
	 * DMA-safe buffer for SPI transfers + naturally aligned
	 * timestamp.  Must be at the end of the struct and must
	 * be separately cache-line aligned.
	 */
	struct {
		s16 channels[4];	/* X, Y, Z, TEMP */
		s64 ts __aligned(8);
	} buffer __aligned(IIO_DMA_MINALIGN);
};

/* ------------------------------------------------------------------ */
/* Register Access Helpers                                             */
/* ------------------------------------------------------------------ */

static int <devname>_read_reg(struct <devname>_state *st, u8 reg,
			      unsigned int *val)
{
	/* Implement SPI read (device-specific protocol) */
}

static int <devname>_write_reg(struct <devname>_state *st, u8 reg,
			       unsigned int val)
{
	/* Implement SPI write (device-specific protocol) */
}

/* ------------------------------------------------------------------ */
/* WHO_AM_I Verification                                               */
/* ------------------------------------------------------------------ */

static int <devname>_verify_chip_id(struct <devname>_state *st)
{
	struct device *dev = &st->spi->dev;
	unsigned int val;
	int ret;

	ret = <devname>_read_reg(st, <DEVNAME>_REG_WHO_AM_I, &val);
	if (ret)
		return ret;

	if (val != st->chip_info->chip_id)
		return dev_err_probe(dev, -ENODEV,
				     "Wrong WHO_AM_I 0x%02x, expected 0x%02x\n",
				     val, st->chip_info->chip_id);

	return 0;
}

/* ------------------------------------------------------------------ */
/* Mount Matrix                                                        */
/* ------------------------------------------------------------------ */

static const struct iio_mount_matrix *
<devname>_get_mount_matrix(const struct iio_dev *indio_dev,
			   const struct iio_chan_spec *chan)
{
	struct <devname>_state *st = iio_priv(indio_dev);

	return &st->orientation;
}

static const struct iio_chan_spec_ext_info <devname>_ext_info[] = {
	IIO_MOUNT_MATRIX(IIO_SHARED_BY_DIR, <devname>_get_mount_matrix),
	{ }
};

/* ------------------------------------------------------------------ */
/* IIO Channel Specification                                           */
/* ------------------------------------------------------------------ */

/*
 * IIO_ANGL_VEL channels with axis modifiers.  Each axis produces a
 * signed 16-bit value.  Scale converts raw to rad/s.
 */
#define <DEVNAME>_GYRO_CHANNEL(_reg, _axis) {				\
	.type = IIO_ANGL_VEL,						\
	.address = (_reg),						\
	.modified = 1,							\
	.channel2 = IIO_MOD_##_axis,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
		BIT(IIO_CHAN_INFO_CALIBBIAS),				\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |		\
		BIT(IIO_CHAN_INFO_SAMP_FREQ),				\
	.scan_index = <DEVNAME>_IDX_##_axis,				\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = 16,						\
		.storagebits = 16,					\
		.endianness = IIO_LE,					\
	},								\
	.ext_info = <devname>_ext_info,					\
}

static const struct iio_chan_spec <devname>_channels[] = {
	<DEVNAME>_GYRO_CHANNEL(<DEVNAME>_REG_DATAX0, X),
	<DEVNAME>_GYRO_CHANNEL(<DEVNAME>_REG_DATAY0, Y),
	<DEVNAME>_GYRO_CHANNEL(<DEVNAME>_REG_DATAZ0, Z),
	/* Optional die temperature channel */
	{
		.type = IIO_TEMP,
		.address = <DEVNAME>_REG_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
			BIT(IIO_CHAN_INFO_SCALE),
		.scan_index = <DEVNAME>_IDX_TEMP,
		.scan_type = {
			.sign = 's',
			.realbits = 12,
			.storagebits = 16,
			.endianness = IIO_LE,
		},
	},
	IIO_CHAN_SOFT_TIMESTAMP(<DEVNAME>_IDX_TS),
};

/*
 * Available scan masks.  Require all three axes (and optionally
 * temperature) to be captured together via a single burst read.
 */
static const unsigned long <devname>_avail_scan_masks[] = {
	BIT(<DEVNAME>_IDX_X) | BIT(<DEVNAME>_IDX_Y) | BIT(<DEVNAME>_IDX_Z),
	BIT(<DEVNAME>_IDX_X) | BIT(<DEVNAME>_IDX_Y) | BIT(<DEVNAME>_IDX_Z) |
		BIT(<DEVNAME>_IDX_TEMP),
	0
};

/* ------------------------------------------------------------------ */
/* read_raw / write_raw Callbacks                                      */
/* ------------------------------------------------------------------ */

static int <devname>_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int *val, int *val2, long mask)
{
	struct <devname>_state *st = iio_priv(indio_dev);
	unsigned int regval;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (!iio_device_claim_direct(indio_dev))
			return -EBUSY;

		mutex_lock(&st->lock);
		ret = <devname>_read_reg(st, chan->address, &regval);
		mutex_unlock(&st->lock);

		iio_device_release_direct(indio_dev);

		if (ret)
			return ret;

		switch (chan->type) {
		case IIO_ANGL_VEL:
			*val = sign_extend32(regval, 15);
			return IIO_VAL_INT;
		case IIO_TEMP:
			/* Extract valid bits for temperature */
			*val = sign_extend32(regval, 11);
			return IIO_VAL_INT;
		default:
			return -EINVAL;
		}

	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_ANGL_VEL:
			/*
			 * Scale in rad/s.  Example: 1 LSB = 0.005 deg/s
			 * 0.005 * pi / 180 = 0.000087266 rad/s
			 * Return as IIO_VAL_INT_PLUS_NANO: 0 + 87266 nano
			 */
			*val = 0;
			*val2 = 87266;
			return IIO_VAL_INT_PLUS_NANO;
		case IIO_TEMP:
			/*
			 * Temperature scale in millidegrees C per LSB.
			 * Example: 1 LSB = 0.1 deg C = 100 mdeg C
			 */
			*val = 100;
			return IIO_VAL_INT;
		default:
			return -EINVAL;
		}

	case IIO_CHAN_INFO_CALIBBIAS:
		/* Read per-axis calibration bias from hardware */
		guard(mutex)(&st->lock);
		/* ... read calibbias register for chan->channel2 ... */
		*val = 0;  /* placeholder */
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SAMP_FREQ:
		guard(mutex)(&st->lock);
		/* Read ODR from hardware */
		*val = 100;  /* placeholder: 100 Hz */
		*val2 = 0;
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

static int <devname>_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val, int val2, long mask)
{
	struct <devname>_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (val <= 0 || val2 != 0)
			return -EINVAL;

		guard(mutex)(&st->lock);
		/* Configure hardware sample rate */
		return <devname>_write_reg(st, <DEVNAME>_REG_FILTER, val);

	case IIO_CHAN_INFO_CALIBBIAS:
		guard(mutex)(&st->lock);
		/* Write per-axis calibration bias to hardware */
		return <devname>_write_reg(st, /* calibbias reg */, val);

	default:
		return -EINVAL;
	}
}

/* ------------------------------------------------------------------ */
/* debugfs Register Access                                             */
/* ------------------------------------------------------------------ */

static int <devname>_reg_access(struct iio_dev *indio_dev,
				unsigned int reg, unsigned int writeval,
				unsigned int *readval)
{
	struct <devname>_state *st = iio_priv(indio_dev);

	if (readval)
		return <devname>_read_reg(st, reg, readval);

	return <devname>_write_reg(st, reg, writeval);
}

/* ------------------------------------------------------------------ */
/* iio_info                                                            */
/* ------------------------------------------------------------------ */

static const struct iio_info <devname>_info = {
	.read_raw = <devname>_read_raw,
	.write_raw = <devname>_write_raw,
	.debugfs_reg_access = <devname>_reg_access,
};

/* ------------------------------------------------------------------ */
/* Probe                                                               */
/* ------------------------------------------------------------------ */

static int <devname>_probe(struct spi_device *spi)
{
	const struct <devname>_chip_info *info;
	struct device *dev = &spi->dev;
	struct <devname>_state *st;
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

	/* Power supply */
	ret = devm_regulator_get_enable(dev, "vdd");
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to enable vdd supply\n");

	/* Verify chip identity */
	ret = <devname>_verify_chip_id(st);
	if (ret)
		return ret;

	/* Read mount matrix from DT */
	ret = iio_read_mount_matrix(dev, &st->orientation);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to read mount matrix\n");

	indio_dev->name = info->name;
	indio_dev->info = &<devname>_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = <devname>_channels;
	indio_dev->num_channels = ARRAY_SIZE(<devname>_channels);
	indio_dev->available_scan_masks = <devname>_avail_scan_masks;

	/* Hardware init: reset, configure default ODR and range */
	/* ... */

	/* Set up triggered buffer (see Section 7) */
	ret = devm_iio_triggered_buffer_setup(dev, indio_dev,
					      &iio_pollfunc_store_time,
					      <devname>_trigger_handler,
					      NULL);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to setup triggered buffer\n");

	/* Set up data-ready trigger if IRQ is available */
	/* ... see Section 7 ... */

	return devm_iio_device_register(dev, indio_dev);
}

/* ------------------------------------------------------------------ */
/* Match Tables                                                        */
/* ------------------------------------------------------------------ */

static const struct <devname>_chip_info <devname>_chip_info = {
	.name = "<devname>",
	.chip_id = <DEVNAME>_WHO_AM_I_VAL,
};

static const struct of_device_id <devname>_of_match[] = {
	{ .compatible = "adi,<devname>", .data = &<devname>_chip_info },
	{ }
};
MODULE_DEVICE_TABLE(of, <devname>_of_match);

static const struct spi_device_id <devname>_ids[] = {
	{ "<devname>", (kernel_ulong_t)&<devname>_chip_info },
	{ }
};
MODULE_DEVICE_TABLE(spi, <devname>_ids);

static struct spi_driver <devname>_driver = {
	.driver = {
		.name = "<devname>",
		.of_match_table = <devname>_of_match,
	},
	.probe = <devname>_probe,
	.id_table = <devname>_ids,
};
module_spi_driver(<devname>_driver);

MODULE_AUTHOR("First Last <first.last@analog.com>");
MODULE_DESCRIPTION("Analog Devices <DEVNAME> Gyroscope driver");
MODULE_LICENSE("GPL");
```

### I2C Driver Variant

For I2C devices, replace the SPI-specific parts:

```c
#include <linux/i2c.h>
#include <linux/regmap.h>

struct <devname>_state {
	struct regmap		*regmap;
	/* ... same fields minus spi pointer ... */
};

static const struct regmap_config <devname>_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xFF,
};

static int <devname>_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct <devname>_state *st;
	struct iio_dev *indio_dev;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->regmap = devm_regmap_init_i2c(client, &<devname>_regmap_config);
	if (IS_ERR(st->regmap))
		return PTR_ERR(st->regmap);

	/* ... same init as SPI variant ... */

	return devm_iio_device_register(dev, indio_dev);
}

static const struct of_device_id <devname>_of_match[] = {
	{ .compatible = "adi,<devname>", .data = &<devname>_chip_info },
	{ }
};
MODULE_DEVICE_TABLE(of, <devname>_of_match);

static const struct i2c_device_id <devname>_ids[] = {
	{ "<devname>" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, <devname>_ids);

static struct i2c_driver <devname>_driver = {
	.driver = {
		.name = "<devname>",
		.of_match_table = <devname>_of_match,
	},
	.probe = <devname>_probe,
	.id_table = <devname>_ids,
};
module_i2c_driver(<devname>_driver);
```

---

## 7. Buffer & Trigger

### Triggered Buffer Setup

Called from `probe()`:

```c
ret = devm_iio_triggered_buffer_setup(dev, indio_dev,
				      &iio_pollfunc_store_time,
				      <devname>_trigger_handler,
				      NULL);
```

### Trigger Handler (3-Axis + Optional Temperature)

```c
static irqreturn_t <devname>_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct <devname>_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&st->lock);

	/*
	 * Burst-read all axis registers (and optionally temperature).
	 * For SPI, this is typically a single multi-byte transaction
	 * starting from the X data register.
	 *
	 * Buffer layout: [X:16][Y:16][Z:16][TEMP:16][PAD][TS:64]
	 */
	ret = spi_write_then_read(st->spi,
				  /* burst read command */,
				  /* cmd len */,
				  st->buffer.channels,
				  sizeof(st->buffer.channels));
	if (ret)
		goto done;

	iio_push_to_buffers_with_timestamp(indio_dev, &st->buffer,
					   pf->timestamp);

done:
	mutex_unlock(&st->lock);
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}
```

### Data-Ready Trigger (Using Device IRQ)

```c
static int <devname>_data_rdy_trigger_set_state(struct iio_trigger *trig,
						bool state)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct <devname>_state *st = iio_priv(indio_dev);

	/* Enable/disable data-ready interrupt output on the device */
	return <devname>_write_reg(st, <DEVNAME>_REG_INT_EN, state ? 1 : 0);
}

static const struct iio_trigger_ops <devname>_trigger_ops = {
	.set_trigger_state = &<devname>_data_rdy_trigger_set_state,
	.validate_device = &iio_trigger_validate_own_device,
};

static int <devname>_probe_trigger(struct iio_dev *indio_dev)
{
	struct <devname>_state *st = iio_priv(indio_dev);
	struct device *dev = &st->spi->dev;
	int ret;

	if (!st->spi->irq)
		return 0;  /* No IRQ, polling only */

	st->dready_trig = devm_iio_trigger_alloc(dev, "%s-dev%d",
						  indio_dev->name,
						  iio_device_id(indio_dev));
	if (!st->dready_trig)
		return -ENOMEM;

	st->dready_trig->ops = &<devname>_trigger_ops;
	iio_trigger_set_drvdata(st->dready_trig, indio_dev);

	ret = devm_request_irq(dev, st->spi->irq,
			       &iio_trigger_generic_data_rdy_poll,
			       IRQF_ONESHOT, "<devname>_drdy",
			       st->dready_trig);
	if (ret)
		return dev_err_probe(dev, ret, "request irq %d failed\n",
				     st->spi->irq);

	ret = devm_iio_trigger_register(dev, st->dready_trig);
	if (ret)
		return dev_err_probe(dev, ret,
				     "iio trigger register failed\n");

	indio_dev->trig = iio_trigger_get(st->dready_trig);

	return 0;
}
```

### scan_type for Gyroscope Channels

```c
.scan_type = {
	.sign = 's',          /* Gyro data is signed */
	.realbits = 16,       /* Actual ADC resolution */
	.storagebits = 16,    /* Buffer storage width */
	.shift = 0,
	.endianness = IIO_LE, /* or IIO_BE depending on device */
},
```

### Kconfig Selects for Buffer Support

```kconfig
	select IIO_BUFFER
	select IIO_TRIGGERED_BUFFER
```

---

## 8. Mount Matrix

Gyroscopes measure angular velocity relative to the sensor's package axes.
When the sensor is mounted on a board in a non-standard orientation, the
`mount-matrix` devicetree property provides a 3x3 rotation matrix that maps
sensor axes to board axes.

### Reading the Mount Matrix in probe()

```c
ret = iio_read_mount_matrix(dev, &st->orientation);
if (ret)
	return dev_err_probe(dev, ret,
			     "Failed to read mount matrix\n");
```

If the DT property is absent, `iio_read_mount_matrix()` returns the
identity matrix by default.

### Exposing the Mount Matrix via ext_info

Attach `ext_info` to each `IIO_ANGL_VEL` channel:

```c
static const struct iio_mount_matrix *
<devname>_get_mount_matrix(const struct iio_dev *indio_dev,
			   const struct iio_chan_spec *chan)
{
	struct <devname>_state *st = iio_priv(indio_dev);

	return &st->orientation;
}

static const struct iio_chan_spec_ext_info <devname>_ext_info[] = {
	IIO_MOUNT_MATRIX(IIO_SHARED_BY_DIR, <devname>_get_mount_matrix),
	{ }
};

/* In the channel macro: */
	.ext_info = <devname>_ext_info,
```

### Sysfs Result

```
/sys/bus/iio/devices/iio:device0/
    in_anglvel_mount_matrix        # "0, 1, 0; 1, 0, 0; 0, 0, -1"
```

Userspace libraries (e.g. `iio-sensor-proxy`) apply this matrix to rotate
sensor data into the board coordinate frame.

---

## 9. Test & Debug

### Sysfs Interface

Every IIO gyroscope exposes channels through sysfs:

```
/sys/bus/iio/devices/iio:device0/
    name                                 # Device name (e.g. "adxrs290")
    in_anglvel_x_raw                     # Raw X-axis angular velocity
    in_anglvel_y_raw                     # Raw Y-axis angular velocity
    in_anglvel_z_raw                     # Raw Z-axis angular velocity
    in_anglvel_scale                     # Scale factor (rad/s per LSB)
    in_anglvel_x_calibbias               # X-axis calibration offset
    in_anglvel_y_calibbias               # Y-axis calibration offset
    in_anglvel_z_calibbias               # Z-axis calibration offset
    in_anglvel_sampling_frequency        # Output data rate (Hz)
    in_anglvel_mount_matrix              # Mount matrix (if ext_info set)
    in_temp_raw                          # Die temperature raw value
    in_temp_scale                        # Temperature scale (mdeg C/LSB)
```

### Converting Raw to Engineering Units

```sh
# Angular velocity in rad/s:
raw=$(cat in_anglvel_x_raw)
scale=$(cat in_anglvel_scale)
# result = raw * scale  (in rad/s)

# Die temperature in degrees Celsius:
raw=$(cat in_temp_raw)
scale=$(cat in_temp_scale)
# result = raw * scale / 1000  (scale is in mdeg C)
```

### debugfs Register Access

```sh
# Read register 0x00 (WHO_AM_I)
echo 0x00 > /sys/kernel/debug/iio/iio:device0/direct_reg_access
cat /sys/kernel/debug/iio/iio:device0/direct_reg_access

# Write 0x03 to register 0x10
echo 0x10 0x03 > /sys/kernel/debug/iio/iio:device0/direct_reg_access
```

### Buffered Data Capture

```sh
# Enable channels
echo 1 > /sys/bus/iio/devices/iio:device0/scan_elements/in_anglvel_x_en
echo 1 > /sys/bus/iio/devices/iio:device0/scan_elements/in_anglvel_y_en
echo 1 > /sys/bus/iio/devices/iio:device0/scan_elements/in_anglvel_z_en

# Set buffer length and enable
echo 128 > /sys/bus/iio/devices/iio:device0/buffer/length
echo 1 > /sys/bus/iio/devices/iio:device0/buffer/enable

# Read data
cat /dev/iio:device0 | hexdump -C | head
```

### IIO Userspace Tools

```sh
# List all IIO devices and channels
iio_info

# Continuous buffered read
iio_readdev -b 256 -s 1024 iio:device0

# Read a specific attribute
iio_attr -d iio:device0 -c in_anglvel_x_raw
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

### Units

| Channel Type   | Unit                    | IIO Standard                     |
|----------------|-------------------------|----------------------------------|
| `IIO_ANGL_VEL` | rad/s                  | Hardware deg/s must be converted |
| `IIO_TEMP`     | millidegrees Celsius    | Per IIO ABI                      |

### Scale Conversion Reference

| Hardware Sensitivity | Computation                    | `*val` | `*val2` | Return Type               |
|----------------------|--------------------------------|--------|---------|---------------------------|
| 0.005 deg/s per LSB  | 0.005 * pi / 180 = 87.266e-6 | 0      | 87266   | `IIO_VAL_INT_PLUS_NANO`   |
| 0.001 deg/s per LSB  | 0.001 * pi / 180 = 17.453e-6 | 0      | 17453   | `IIO_VAL_INT_PLUS_NANO`   |
| 70 mdps per LSB      | 0.070 * pi / 180 = 1221.7e-6 | 0      | 1221730 | `IIO_VAL_INT_PLUS_NANO`   |
| Range / 2^bits       | range_dps * pi / (180 * 2^n)  | -      | -       | `IIO_VAL_FRACTIONAL_LOG2` |

### Memory Management

Use `devm_*` (device-managed) allocations exclusively:

| Function                             | Purpose                              |
|--------------------------------------|--------------------------------------|
| `devm_iio_device_alloc()`            | Allocate IIO device + private data   |
| `devm_iio_device_register()`         | Register IIO device (auto-unregister)|
| `devm_iio_triggered_buffer_setup()`  | Set up triggered buffer              |
| `devm_iio_trigger_alloc()`           | Allocate IIO trigger                 |
| `devm_iio_trigger_register()`        | Register IIO trigger                 |
| `devm_regulator_get_enable()`        | Get and enable a regulator           |
| `devm_request_irq()`                 | Request interrupt with auto-free     |
| `devm_mutex_init()`                  | Initialize mutex with devm cleanup   |
| `devm_add_action_or_reset()`         | Register custom cleanup callback     |

### Mutex / Locking

- Use `struct mutex` for protecting device state and bus transactions.
- Initialize with `devm_mutex_init()` in `probe()`.
- Use `guard(mutex)(&st->lock)` (scoped lock, auto-release) where possible.
- Use `mutex_lock()` / `mutex_unlock()` when scoped locking is not suitable.
- Use `iio_device_claim_direct()` / `iio_device_release_direct()` to prevent
  raw reads while the buffer is active.

### Coding Style

- Follow kernel coding style (`Documentation/process/coding-style.rst`).
- Run `scripts/checkpatch.pl` before submitting.
- Use `FIELD_GET()` / `FIELD_PREP()` with `GENMASK()` for register bitfields.
- Use `BIT()` for single-bit definitions.
- Use `dev_err_probe()` for all error returns in `probe()`.
- Include headers in alphabetical order within each group.

---

## 11. Commit Message Format

### Subject Line

```
iio: gyro: <devname>: <brief description>
```

Examples:

```
iio: gyro: adxrs290: add support for ADXRS290 dual-axis gyroscope
iio: gyro: adxrs290: add triggered buffer support
iio: gyro: adxrs290: fix scale factor for 16-bit mode
```

### Commit Body

- Wrap at 75 characters.
- Explain **why** the change is made, not just what.
- Reference datasheets or errata when relevant.
- Include `Signed-off-by:` (use `git commit -s`).

### Patch Series for a New Gyroscope Driver

A typical new driver submission is a patch series:

1. `dt-bindings: iio: gyroscope: add adi,<devname>.yaml` -- DT binding
2. `iio: gyro: <devname>: add support for <DEVNAME>` -- Driver source
3. `MAINTAINERS: add entry for <DEVNAME> IIO driver` -- Maintainer entry

### Full Example

```
iio: gyro: adxrs290: add support for ADXRS290 dual-axis gyroscope

The ADXRS290 is a high performance, low noise, low drift, dual-axis
MEMS gyroscope. It provides digital output via SPI with 16-bit
resolution.

This driver supports:
  - X and Y axis angular velocity measurement
  - Die temperature readout
  - Configurable low-pass and high-pass filter frequencies
  - Triggered buffer for continuous data capture
  - Data-ready interrupt

Signed-off-by: First Last <first.last@analog.com>
```
