# Linux IIO Digital Potentiometer Driver Template

This template covers all files needed to write a Linux kernel IIO driver for
a digital potentiometer or rheostat (e.g., AD5272, AD5110).  Digital
potentiometers expose `IIO_RESISTANCE` channels with `info_mask` bits RAW and
SCALE.  RAW is the wiper position (0 to max_pos), and SCALE converts it to
ohms per step.

Reference drivers:
- `drivers/iio/potentiometer/ad5272.c` -- I2C, 1 wiper, 1024 positions
- `drivers/iio/potentiometer/ad5110.c` -- I2C, 1 wiper, EEPROM, tolerance
- `drivers/iio/potentiometer/mcp41010.c` -- SPI, 1-2 wipers, 256 positions
- `drivers/iio/potentiometer/x9250.c` -- SPI, 4 wipers, write-protect GPIO

---

## 1. Purpose

IIO Potentiometer drivers model digitally-controlled variable resistors.  A
digital potentiometer has a resistive element with two end terminals (A, B)
and a wiper that slides between them.  The driver controls the wiper position
through I2C or SPI commands.

**Channel type:** `IIO_RESISTANCE`

**Direction:** Output (`.output = 1`) -- the wiper position is a controllable
output that sets the resistance.

**info_mask bits:**

| Bit                        | Meaning                                                  |
|----------------------------|----------------------------------------------------------|
| `IIO_CHAN_INFO_RAW`        | Wiper position (integer, 0 to max_pos - 1)              |
| `IIO_CHAN_INFO_SCALE`      | Ohms per wiper step = R_AB_ohms / max_pos                |
| `IIO_CHAN_INFO_OFFSET`     | Wiper resistance offset (optional, e.g., AD5110: 70 ohm) |
| `IIO_CHAN_INFO_ENABLE`     | Shutdown control (optional, e.g., AD5110)                |

**Typical ADI parts:** AD5272, AD5274, AD5110, AD5112, AD5114.

---

## 2. File Checklist

| File                                                                          | Action   | Required |
|-------------------------------------------------------------------------------|----------|----------|
| `drivers/iio/potentiometer/<devname>.c`                                       | Create   | Yes      |
| `drivers/iio/potentiometer/Kconfig`                                           | Modify   | Yes      |
| `drivers/iio/potentiometer/Makefile`                                          | Modify   | Yes      |
| `Documentation/devicetree/bindings/iio/potentiometer/adi,<devname>.yaml`      | Create   | Yes      |

No header file is typically needed -- potentiometer drivers are self-contained
single-file drivers.

---

## 3. DT Binding (`adi,<devname>.yaml`)

Digital potentiometer bindings encode the resistance variant in the compatible
string (e.g., `adi,ad5272-020` for 20 kOhm).  Key properties:

- **Bus:** `reg` for I2C address or SPI chip-select.
- **Resistance variant:** Encoded in the compatible string suffix (kohms).
- **Reset GPIO:** Optional hardware reset line.
- **Write-protect GPIO:** Optional WP pin for NVM protection.

### I2C Bus Example (AD5272)

```yaml
# SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)
%YAML 1.2
---
$id: http://devicetree.org/schemas/iio/potentiometer/adi,ad5272.yaml#
$schema: http://devicetree.org/meta-schemas/core.yaml#

title: Analog Devices AD5272/AD5274 Digital Potentiometer

maintainers:
  - First Last <first.last@analog.com>

description: |
  The AD5272/AD5274 are 1024/256-position digital potentiometers with
  I2C interface and 20/50/100 kOhm end-to-end resistance options.

  Datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/AD5272_5274.pdf

properties:
  compatible:
    enum:
      - adi,ad5272-020
      - adi,ad5272-050
      - adi,ad5272-100
      - adi,ad5274-020
      - adi,ad5274-100

  reg:
    maxItems: 1

  reset-gpios:
    maxItems: 1
    description:
      Active low signal to the RESET input.

additionalProperties: false

required:
  - compatible
  - reg

examples:
  - |
    #include <dt-bindings/gpio/gpio.h>
    i2c {
        #address-cells = <1>;
        #size-cells = <0>;

        potentiometer@2f {
            compatible = "adi,ad5272-020";
            reg = <0x2f>;
            reset-gpios = <&gpio3 6 GPIO_ACTIVE_LOW>;
        };
    };
```

### SPI Bus Example

```yaml
properties:
  compatible:
    enum:
      - adi,adxxxx-010
      - adi,adxxxx-050
      - adi,adxxxx-100

  reg:
    maxItems: 1

  spi-max-frequency:
    maximum: 10000000

  wp-gpios:
    maxItems: 1
    description:
      Write-protect GPIO for NVM/EEPROM protection.

allOf:
  - $ref: /schemas/spi/spi-peripheral-props.yaml#

unevaluatedProperties: false

required:
  - compatible
  - reg

examples:
  - |
    spi {
        #address-cells = <1>;
        #size-cells = <0>;

        potentiometer@0 {
            compatible = "adi,adxxxx-010";
            reg = <0>;
            spi-max-frequency = <10000000>;
        };
    };
```

### Binding Convention

The resistance variant is encoded as a suffix on the compatible string rather
than as a separate devicetree property.  The driver uses the compatible match
data to look up `max_pos` and `kohms`:

- `adi,ad5272-020` => 1024 positions, 20 kOhm
- `adi,ad5272-100` => 1024 positions, 100 kOhm
- `adi,ad5274-020` => 256 positions, 20 kOhm

---

## 4. Kconfig

Add the entry to `drivers/iio/potentiometer/Kconfig` in **alphabetical
order**.

### I2C Device

```kconfig
config AD5272
	tristate "Analog Devices AD5272 and similar Digital Potentiometer driver"
	depends on I2C
	help
	  Say yes here to build support for the Analog Devices AD5272 and AD5274
	  digital potentiometer chip.

	  To compile this driver as a module, choose M here: the
	  module will be called ad5272.
```

### SPI Device

```kconfig
config ADXXXX
	tristate "Analog Devices ADXXXX Digital Potentiometer driver"
	depends on SPI
	help
	  Say yes here to build support for the Analog Devices ADXXXX
	  digital potentiometer chip.

	  To compile this driver as a module, choose M here: the
	  module will be called adxxxx.
```

### Notes

- I2C potentiometers use `depends on I2C`.
- SPI potentiometers use `depends on SPI`.
- `select REGMAP_I2C` or `select REGMAP_SPI` only if the driver uses regmap
  (most potentiometer drivers use direct I2C/SPI transfers instead).

---

## 5. Makefile

Add the entry to `drivers/iio/potentiometer/Makefile` in **alphabetical
order**.

```makefile
obj-$(CONFIG_AD5272) += ad5272.o
```

Potentiometer drivers are always single-file; multi-file builds are not
needed.

---

## 6. Driver Source (`<devname>.c`)

### Complete I2C Skeleton (AD5272-style)

```c
// SPDX-License-Identifier: GPL-2.0+
/*
 * Analog Devices ADXXXX digital potentiometer driver
 *
 * Copyright (C) 2024 Analog Devices Inc.
 *
 * Datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/ADXXXX.pdf
 *
 * DEVID	#Wipers	#Positions	Resistor Opts (kOhm)	i2c address
 * adxxxx	1	1024		20, 50, 100		01011xx
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>

#include <linux/iio/iio.h>

/* ------------------------------------------------------------------ */
/* Command Codes                                                       */
/* ------------------------------------------------------------------ */

#define ADXXXX_RDAC_WR		1
#define ADXXXX_RDAC_RD		2
#define ADXXXX_RESET		4
#define ADXXXX_CTL		7

#define ADXXXX_RDAC_WR_EN	BIT(1)

/* ------------------------------------------------------------------ */
/* Chip Configuration Table                                            */
/* ------------------------------------------------------------------ */

/*
 * Per-variant parameters.  The compatible string encodes the resistance
 * option, and the driver looks up max_pos, kohms, and shift from here.
 *
 * max_pos: number of wiper positions (e.g. 1024 for 10-bit, 256 for 8-bit)
 * kohms:   end-to-end resistance in kilo-ohms
 * shift:   left-shift applied to the wiper value for hardware alignment
 *          (used when the DAC register is wider than the position resolution)
 */
struct adxxxx_cfg {
	int max_pos;
	int kohms;
	int shift;
};

enum adxxxx_type {
	ADXXXX_020,
	ADXXXX_050,
	ADXXXX_100,
};

static const struct adxxxx_cfg adxxxx_cfg[] = {
	[ADXXXX_020] = { .max_pos = 1024, .kohms = 20 },
	[ADXXXX_050] = { .max_pos = 1024, .kohms = 50 },
	[ADXXXX_100] = { .max_pos = 1024, .kohms = 100 },
};

/* ------------------------------------------------------------------ */
/* Device State                                                        */
/* ------------------------------------------------------------------ */

struct adxxxx_data {
	struct i2c_client       *client;
	struct mutex            lock;   /* Protect device state */
	const struct adxxxx_cfg *cfg;
	u8                      buf[2] __aligned(IIO_DMA_MINALIGN);
};

/* ------------------------------------------------------------------ */
/* I2C Helpers                                                         */
/* ------------------------------------------------------------------ */

static int adxxxx_write(struct adxxxx_data *data, int reg, int val)
{
	int ret;

	data->buf[0] = (reg << 2) | ((val >> 8) & 0x3);
	data->buf[1] = (u8)val;

	mutex_lock(&data->lock);
	ret = i2c_master_send(data->client, data->buf, sizeof(data->buf));
	mutex_unlock(&data->lock);

	return ret < 0 ? ret : 0;
}

static int adxxxx_read(struct adxxxx_data *data, int reg, int *val)
{
	int ret;

	data->buf[0] = reg << 2;
	data->buf[1] = 0;

	mutex_lock(&data->lock);
	ret = i2c_master_send(data->client, data->buf, sizeof(data->buf));
	if (ret < 0)
		goto error;

	ret = i2c_master_recv(data->client, data->buf, sizeof(data->buf));
	if (ret < 0)
		goto error;

	*val = ((data->buf[0] & 0x3) << 8) | data->buf[1];
	ret = 0;
error:
	mutex_unlock(&data->lock);
	return ret;
}

/* ------------------------------------------------------------------ */
/* IIO Channel Specification                                           */
/* ------------------------------------------------------------------ */

/*
 * Single resistance output channel.
 *
 * .output = 1: the wiper position is a controllable output.
 * RAW:  wiper position (0 to max_pos - 1), read/write.
 * SCALE: shared by type, ohms per step = (kohms * 1000) / max_pos.
 */
static const struct iio_chan_spec adxxxx_channel = {
	.type = IIO_RESISTANCE,
	.output = 1,
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),
};

/*
 * For multi-wiper devices, use an indexed channel macro:
 *
 * #define ADXXXX_CHANNEL(ch) {                                        \
 *         .type = IIO_RESISTANCE,                                     \
 *         .indexed = 1,                                               \
 *         .output = 1,                                                \
 *         .channel = (ch),                                            \
 *         .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),               \
 *         .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),       \
 * }
 *
 * static const struct iio_chan_spec adxxxx_channels[] = {
 *         ADXXXX_CHANNEL(0),
 *         ADXXXX_CHANNEL(1),
 * };
 */

/* ------------------------------------------------------------------ */
/* read_raw / write_raw Callbacks                                      */
/* ------------------------------------------------------------------ */

static int adxxxx_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	struct adxxxx_data *data = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = adxxxx_read(data, ADXXXX_RDAC_RD, val);
		if (ret)
			return ret;

		*val = *val >> data->cfg->shift;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		/*
		 * Scale = total resistance / number of positions
		 *       = (kohms * 1000) / max_pos  [ohms per step]
		 *
		 * Returned as IIO_VAL_FRACTIONAL so userspace computes:
		 *   resistance = raw * val / val2  [ohms]
		 */
		*val = 1000 * data->cfg->kohms;
		*val2 = data->cfg->max_pos;
		return IIO_VAL_FRACTIONAL;
	}

	return -EINVAL;
}

static int adxxxx_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long mask)
{
	struct adxxxx_data *data = iio_priv(indio_dev);

	if (mask != IIO_CHAN_INFO_RAW)
		return -EINVAL;

	if (val >= data->cfg->max_pos || val < 0 || val2)
		return -EINVAL;

	return adxxxx_write(data, ADXXXX_RDAC_WR, val << data->cfg->shift);
}

static const struct iio_info adxxxx_info = {
	.read_raw = adxxxx_read_raw,
	.write_raw = adxxxx_write_raw,
};

/* ------------------------------------------------------------------ */
/* Reset                                                               */
/* ------------------------------------------------------------------ */

static int adxxxx_reset(struct adxxxx_data *data)
{
	struct gpio_desc *reset_gpio;

	reset_gpio = devm_gpiod_get_optional(&data->client->dev, "reset",
					     GPIOD_OUT_HIGH);
	if (IS_ERR(reset_gpio))
		return PTR_ERR(reset_gpio);

	if (reset_gpio) {
		udelay(1);
		gpiod_set_value(reset_gpio, 0);
	} else {
		adxxxx_write(data, ADXXXX_RESET, 0);
	}
	usleep_range(1000, 2000);

	return 0;
}

/* ------------------------------------------------------------------ */
/* Probe                                                               */
/* ------------------------------------------------------------------ */

static int adxxxx_probe(struct i2c_client *client)
{
	const struct i2c_device_id *id = i2c_client_get_device_id(client);
	struct device *dev = &client->dev;
	struct iio_dev *indio_dev;
	struct adxxxx_data *data;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	data->client = client;
	mutex_init(&data->lock);
	data->cfg = &adxxxx_cfg[id->driver_data];

	ret = adxxxx_reset(data);
	if (ret)
		return ret;

	/* Enable RDAC writes */
	ret = adxxxx_write(data, ADXXXX_CTL, ADXXXX_RDAC_WR_EN);
	if (ret < 0)
		return -ENODEV;

	indio_dev->info = &adxxxx_info;
	indio_dev->channels = &adxxxx_channel;
	indio_dev->num_channels = 1;
	indio_dev->name = client->name;

	return devm_iio_device_register(dev, indio_dev);
}

/* ------------------------------------------------------------------ */
/* Match Tables                                                        */
/* ------------------------------------------------------------------ */

static const struct of_device_id adxxxx_of_match[] = {
	{ .compatible = "adi,adxxxx-020", .data = (void *)ADXXXX_020 },
	{ .compatible = "adi,adxxxx-050", .data = (void *)ADXXXX_050 },
	{ .compatible = "adi,adxxxx-100", .data = (void *)ADXXXX_100 },
	{}
};
MODULE_DEVICE_TABLE(of, adxxxx_of_match);

static const struct i2c_device_id adxxxx_id[] = {
	{ "adxxxx-020", ADXXXX_020 },
	{ "adxxxx-050", ADXXXX_050 },
	{ "adxxxx-100", ADXXXX_100 },
	{}
};
MODULE_DEVICE_TABLE(i2c, adxxxx_id);

static struct i2c_driver adxxxx_driver = {
	.driver = {
		.name = "adxxxx",
		.of_match_table = adxxxx_of_match,
	},
	.probe = adxxxx_probe,
	.id_table = adxxxx_id,
};
module_i2c_driver(adxxxx_driver);

MODULE_AUTHOR("First Last <first.last@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADXXXX digital potentiometer");
MODULE_LICENSE("GPL");
```

### SPI Variant

For SPI-connected potentiometers, replace the I2C-specific parts:

```c
#include <linux/spi/spi.h>

struct adxxxx_data {
	struct spi_device *spi;
	const struct adxxxx_cfg *cfg;
	struct mutex lock;
	unsigned int value[MAX_WIPERS];  /* Cache wiper values */
	u8 buf[2] __aligned(IIO_DMA_MINALIGN);
};

static int adxxxx_write_spi(struct adxxxx_data *data, int ch, int val)
{
	data->buf[0] = ADXXXX_WRITE | (ch & 0x0F);
	data->buf[1] = val & 0xFF;

	return spi_write(data->spi, data->buf, sizeof(data->buf));
}

static int adxxxx_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct adxxxx_data *data;
	struct iio_dev *indio_dev;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	data->spi = spi;
	data->cfg = spi_get_device_match_data(spi);

	mutex_init(&data->lock);

	indio_dev->info = &adxxxx_info;
	indio_dev->channels = adxxxx_channels;
	indio_dev->num_channels = data->cfg->wipers;
	indio_dev->name = data->cfg->name;

	return devm_iio_device_register(dev, indio_dev);
}

static const struct spi_device_id adxxxx_id[] = {
	{ "adxxxx-010", (kernel_ulong_t)&adxxxx_cfg[ADXXXX_010] },
	{ "adxxxx-050", (kernel_ulong_t)&adxxxx_cfg[ADXXXX_050] },
	{ }
};
MODULE_DEVICE_TABLE(spi, adxxxx_id);

static struct spi_driver adxxxx_driver = {
	.driver = {
		.name = "adxxxx",
		.of_match_table = adxxxx_of_match,
	},
	.probe = adxxxx_probe,
	.id_table = adxxxx_id,
};
module_spi_driver(adxxxx_driver);
```

### Channel Patterns

**Single wiper (most ADI pots):**

```c
static const struct iio_chan_spec adxxxx_channel = {
	.type = IIO_RESISTANCE,
	.output = 1,
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),
};
```

**Multi-wiper with indexed channels:**

```c
#define ADXXXX_CHANNEL(ch) {                                    \
	.type = IIO_RESISTANCE,                                 \
	.indexed = 1,                                           \
	.output = 1,                                            \
	.channel = (ch),                                        \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),            \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),    \
}

static const struct iio_chan_spec adxxxx_channels[] = {
	ADXXXX_CHANNEL(0),
	ADXXXX_CHANNEL(1),
};
```

**With available range (x9250-style):**

```c
#define ADXXXX_CHANNEL(ch) {                                            \
	.type = IIO_RESISTANCE,                                         \
	.indexed = 1,                                                   \
	.output = 1,                                                    \
	.channel = (ch),                                                \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),                    \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),            \
	.info_mask_shared_by_type_available = BIT(IIO_CHAN_INFO_RAW),    \
}

static int adxxxx_read_avail(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     const int **vals, int *type,
			     int *length, long mask)
{
	static const int range[] = {0, 1, 255}; /* min, step, max */

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		*length = ARRAY_SIZE(range);
		*vals = range;
		*type = IIO_VAL_INT;
		return IIO_AVAIL_RANGE;
	}

	return -EINVAL;
}
```

### Scale Calculation

The scale converts the raw wiper position to resistance in ohms:

```
resistance_ohms = raw * scale
scale = R_AB_ohms / max_pos
      = (kohms * 1000) / max_pos
```

For example, AD5272-020 (20 kOhm, 1024 positions):
```
scale = 20000 / 1024 = 19.53125 ohms/step
```

In `read_raw`:

```c
case IIO_CHAN_INFO_SCALE:
	*val = 1000 * data->cfg->kohms;   /* R_AB in ohms */
	*val2 = data->cfg->max_pos;       /* number of positions */
	return IIO_VAL_FRACTIONAL;
```

### Offset for Wiper Resistance

Some potentiometers have a fixed wiper resistance (R_W) that adds to the
measured resistance.  The AD5110 models this as an offset:

```c
case IIO_CHAN_INFO_OFFSET:
	/* offset = R_W * max_pos / R_AB
	 * so that: resistance = (raw + offset) * scale
	 *        = raw * scale + R_W
	 */
	*val = AD5110_WIPER_RESISTANCE * data->cfg->max_pos;
	*val2 = 1000 * data->cfg->kohms + data->tol;
	return IIO_VAL_FRACTIONAL;
```

---

## 7. EEPROM / NVM

Many digital potentiometers include non-volatile memory (EEPROM) to store
the wiper position so it persists across power cycles.

### EEPROM Store/Recall

The wiper position is saved to EEPROM with a store command and automatically
restored at power-on or software reset.

### ext_info for Store Command (AD5110-style)

Use `IIO_DEVICE_ATTR_RW` to expose a `store_eeprom` attribute through
`iio_info.attrs`.  This is the standard pattern for commands that are not
naturally channel attributes:

```c
static ssize_t store_eeprom_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adxxxx_data *data = iio_priv(indio_dev);
	int val = ADXXXX_EEPROM_WIPER_POS;
	int ret;

	ret = adxxxx_read(data, ADXXXX_EEPROM_RD, &val);
	if (ret)
		return ret;

	val = val >> data->cfg->shift;
	return iio_format_value(buf, IIO_VAL_INT, 1, &val);
}

static ssize_t store_eeprom_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adxxxx_data *data = iio_priv(indio_dev);
	int ret;

	ret = adxxxx_write(data, ADXXXX_EEPROM_WR, 0);
	if (ret) {
		dev_err(&data->client->dev,
			"RDAC to EEPROM write failed\n");
		return ret;
	}

	/* EEPROM write takes approximately 18 ms */
	msleep(20);

	return len;
}

static IIO_DEVICE_ATTR_RW(store_eeprom, 0);

static struct attribute *adxxxx_attributes[] = {
	&iio_dev_attr_store_eeprom.dev_attr.attr,
	NULL
};

static const struct attribute_group adxxxx_attribute_group = {
	.attrs = adxxxx_attributes,
};

static const struct iio_info adxxxx_info = {
	.read_raw = adxxxx_read_raw,
	.write_raw = adxxxx_write_raw,
	.attrs = &adxxxx_attribute_group,
};
```

### Tolerance Readback

Factory-programmed resistance tolerance can be read from EEPROM.  The AD5110
stores a signed tolerance value:

```c
static int adxxxx_resistor_tol(struct adxxxx_data *data, u8 cmd, int val)
{
	int ret;

	ret = adxxxx_read(data, cmd, &val);
	if (ret)
		return ret;

	/* Bits [6:0] = magnitude, bit 7 = sign (1 = positive) */
	data->tol = data->cfg->kohms * (val & GENMASK(6, 0)) * 10 / 8;
	if (!(val & BIT(7)))
		data->tol *= -1;

	return 0;
}
```

The tolerance is then folded into the scale and offset calculations so
userspace sees calibrated values.

---

## 8. DT Parsing

Most potentiometer drivers do not require complex DT parsing.  The resistance
variant (and hence max_pos, kohms) is encoded in the compatible string.  The
driver retrieves it from match data in `probe()`.

### I2C Match Data

```c
static int adxxxx_probe(struct i2c_client *client)
{
	const struct i2c_device_id *id = i2c_client_get_device_id(client);

	/* ... */

	data->cfg = &adxxxx_cfg[id->driver_data];

	/* ... */
}
```

Or with `i2c_get_match_data()` when using `of_device_id.data` pointers:

```c
data->cfg = i2c_get_match_data(client);
```

### SPI Match Data

```c
data->cfg = spi_get_device_match_data(spi);
```

### Optional DT Properties

If the potentiometer has devicetree-configurable properties beyond the
compatible string:

| Property               | API                              | Purpose                          |
|------------------------|----------------------------------|----------------------------------|
| `reset-gpios`          | `devm_gpiod_get_optional()`      | Hardware reset line              |
| `wp-gpios`             | `devm_gpiod_get_optional()`      | Write-protect for NVM            |
| `vdd-supply`           | `devm_regulator_get_enable()`    | Power supply                     |

---

## 9. Test & Debug

### Sysfs Interface

Digital potentiometers expose resistance output channels through sysfs:

```
/sys/bus/iio/devices/iio:device0/
    name                           # Device name (e.g., "ad5272-020")
    out_resistance0_raw            # Read/write wiper position (0 to max_pos-1)
    out_resistance0_scale          # Ohms per step (e.g., 19.531250)
```

Channels use the `out_` prefix because `.output = 1`.  The resistance type
produces `resistance` in the attribute name.

### Reading and Writing the Wiper Position

```sh
# Read current wiper position
cat /sys/bus/iio/devices/iio:device0/out_resistance0_raw

# Set wiper to position 512 (midscale for 1024-position pot)
echo 512 > /sys/bus/iio/devices/iio:device0/out_resistance0_raw

# Read scale (ohms per step)
cat /sys/bus/iio/devices/iio:device0/out_resistance0_scale

# Compute actual resistance:
#   resistance = raw * scale
#   512 * 19.531250 = 10000 ohms = 10 kOhm (midscale of 20 kOhm pot)
```

### EEPROM Attribute (if implemented)

```sh
# Read stored EEPROM wiper position
cat /sys/bus/iio/devices/iio:device0/store_eeprom

# Store current RDAC position to EEPROM (write any value to trigger)
echo 1 > /sys/bus/iio/devices/iio:device0/store_eeprom
```

### iio_info Tool

```sh
# List IIO devices and channels
iio_info

# Read a single attribute
iio_attr -d iio:device0 -c out_resistance0_raw
```

### debugfs Register Access

Most potentiometer drivers do not implement `debugfs_reg_access` because the
device command protocol is not a simple register map.  If implemented:

```sh
echo 0x02 > /sys/kernel/debug/iio/iio:device0/direct_reg_access
cat /sys/kernel/debug/iio/iio:device0/direct_reg_access
```

### Kernel Debugging Tips

- Use `dev_dbg()` / `dev_info()` for debug prints, never `printk()`.
- For I2C bus debugging, enable `CONFIG_I2C_DEBUG_CORE`.
- For SPI bus debugging, enable `CONFIG_SPI_DEBUG`.

---

## 10. Key Conventions

### License

All new ADI IIO drivers must use GPL-2.0 or GPL-2.0+:

```c
// SPDX-License-Identifier: GPL-2.0+
```

The SPDX tag goes on the very first line.  `MODULE_LICENSE("GPL")` at the
bottom must match.

### Units

- **RAW:** Dimensionless wiper position (integer, 0 to max_pos - 1).
- **SCALE:** Ohms per step.  `IIO_VAL_FRACTIONAL` with numerator = R_AB in
  ohms and denominator = max_pos.
- **OFFSET:** Dimensionless.  Converts wiper resistance to equivalent steps.
- The total resistance seen at the output is `(raw + offset) * scale` ohms.

### Resistance Variant Encoding

The end-to-end resistance and number of wiper positions are encoded as a
`struct` lookup table indexed by variant enum.  The compatible string suffix
selects the variant.  Do not use separate DT properties for max_pos or kohms.

### Memory Management

- Use `devm_*` allocations exclusively.  `devm_iio_device_alloc()` for the
  IIO device, `devm_iio_device_register()` for registration.
- The DMA-safe I2C/SPI transfer buffer (`u8 buf[]`) must be at the end of the
  state struct with `__aligned(IIO_DMA_MINALIGN)`.

### Mutex / Locking

- Use `struct mutex` for protecting I2C/SPI transactions and device state.
- Initialize with `mutex_init()` in `probe()` (or `devm_mutex_init()` in
  newer drivers).
- Hold the lock around the send/receive pair in read operations to prevent
  interleaving.

### Coding Style

- Follow the kernel coding style (`Documentation/process/coding-style.rst`).
- Run `scripts/checkpatch.pl` before submitting.
- Use `BIT()` for single-bit definitions, `GENMASK()` for multi-bit fields.
- Use `FIELD_GET()` / `FIELD_PREP()` for register bitfield access.
- Use `dev_err_probe()` for error returns in `probe()`.

---

## 11. Commit Format

### Subject Line

```
iio: potentiometer: <devname>: <brief description>
```

Examples:
```
iio: potentiometer: ad5272: add support for AD5272 and AD5274
iio: potentiometer: ad5110: fix wiper position write validation
iio: potentiometer: ad5110: add shutdown mode support
```

### Commit Body

- Wrap at 75 characters.
- Explain **why** the change is made, not just what.
- Reference datasheets or errata when relevant.
- Include `Signed-off-by:` (use `git commit -s`).

### DT Binding Prefix

```
dt-bindings: iio: potentiometer: add adi,<devname>.yaml
```

### Patch Series for a New Driver

A typical new driver submission is a patch series:

1. `dt-bindings: iio: potentiometer: add adi,<devname>.yaml` -- DT binding
2. `iio: potentiometer: <devname>: add support for <DEVNAME>` -- Driver source
3. `MAINTAINERS: add entry for <DEVNAME> IIO driver` -- Maintainer entry

### Full Example

```
iio: potentiometer: ad5272: add support for AD5272 and AD5274

The AD5272/AD5274 are single-channel digital potentiometers with 1024
and 256 wiper positions respectively.  They feature I2C interface,
20/50/100 kOhm resistance options, and an optional hardware reset
input.

This driver supports:
  - Reading and writing the volatile wiper position (RDAC)
  - Hardware and software reset
  - Multiple resistance variants via devicetree compatible strings

Signed-off-by: First Last <first.last@analog.com>
```
