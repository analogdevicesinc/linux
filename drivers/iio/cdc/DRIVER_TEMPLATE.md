# Linux IIO Capacitance-to-Digital Converter (CDC) Driver Template

This template covers every file needed to add a new Linux kernel IIO CDC
driver for an Analog Devices capacitance-to-digital converter. Reference
drivers: `drivers/iio/cdc/ad7150.c` (threshold events, proximity),
`drivers/iio/cdc/ad7746.c` (differential inputs, excitation config,
calibration).

Replace `<devname>` with the part number (e.g., `ad7150`) and `<DEVNAME>`
with its uppercase form (e.g., `AD7150`) throughout.

---

## 1. Purpose

IIO CDC drivers expose capacitance-to-digital converters (e.g., AD7150,
AD7746) to userspace through the Linux IIO subsystem.

| Property | Value |
|---|---|
| IIO subdirectory | `iio/cdc/` |
| Channel type | `IIO_CAPACITANCE` |
| Typical info_mask bits | RAW, SCALE, OFFSET, SAMP_FREQ, CALIBSCALE, CALIBBIAS |
| Bus | I2C (all current CDC parts) |
| Scale unit | Picofarads (pF); IIO base unit for capacitance is nanofarads, so scale must convert to nF |
| Reference parts | AD7150, AD7151, AD7156, AD7745, AD7746, AD7747 |

CDC drivers typically provide:
- Single-ended and/or differential capacitance input channels.
- Excitation source configuration (output pins, voltage level).
- Gain calibration (`CALIBSCALE`) and offset calibration (`CALIBBIAS`).
- Capacitance threshold events for proximity/touch detection.
- Configurable sample rate via `SAMP_FREQ`.

---

## 2. File Checklist

| File | Action | Required |
|---|---|---|
| `drivers/iio/cdc/<devname>.c` | Create | Yes |
| `drivers/iio/cdc/Kconfig` | Modify | Yes |
| `drivers/iio/cdc/Makefile` | Modify | Yes |
| `Documentation/devicetree/bindings/iio/cdc/adi,<devname>.yaml` | Create | Yes |
| `MAINTAINERS` | Modify | Yes |

---

## 3. DT Binding

Bindings live under `Documentation/devicetree/bindings/iio/cdc/adi,<devname>.yaml`.

CDC parts are I2C devices. The binding should include excitation source
configuration properties when the part supports configurable excitation
outputs.

```yaml
# SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)
%YAML 1.2
---
$id: http://devicetree.org/schemas/iio/cdc/adi,<devname>.yaml#
$schema: http://devicetree.org/meta-schemas/core.yaml#

title: Analog Devices <DEVNAME> Capacitance-to-Digital Converter

maintainers:
  - First Last <first.last@analog.com>

description: |
  The <DEVNAME> is a <resolution>-bit capacitance-to-digital converter
  with <key feature, e.g., dual-channel input, on-chip temperature sensor>.

properties:
  compatible:
    enum:
      - adi,<devname>
      # - adi,<devname_variant>

  reg:
    maxItems: 1

  vdd-supply: true

  interrupts: true
    # Use minItems/maxItems or conditional allOf blocks when the
    # number of interrupts varies by compatible (see ad7150 binding).

  # --- Excitation configuration (AD7746-style) ---
  # Include these when the part has configurable excitation outputs.

  adi,exca-output-en:
    description: Enables the EXCA pin as the excitation output.
    type: boolean

  adi,exca-output-invert:
    description: Inverts the excitation output on the EXCA pin.
    type: boolean

  adi,excb-output-en:
    description: Enables the EXCB pin as the excitation output.
    type: boolean

  adi,excb-output-invert:
    description: Inverts the excitation output on the EXCB pin.
    type: boolean

  adi,excitation-vdd-permille:
    description: |
      Excitation voltage as a fraction of VDD in permille.
    $ref: /schemas/types.yaml#/definitions/uint32
    enum: [125, 250, 375, 500]

required:
  - compatible
  - reg

additionalProperties: false

examples:
  - |
    i2c {
        #address-cells = <1>;
        #size-cells = <0>;

        cdc@48 {
            compatible = "adi,<devname>";
            reg = <0x48>;
            interrupts = <25 2>, <26 2>;
            interrupt-parent = <&gpio>;
            vdd-supply = <&vdd_3v3>;

            /* Excitation config (if applicable) */
            adi,exca-output-en;
            adi,excitation-vdd-permille = <125>;
        };
    };
```

### Interrupt Handling by Compatible

When the number of interrupt lines varies by compatible (e.g., AD7150 has
two threshold output pins, AD7151 has one), use `allOf` conditional blocks:

```yaml
allOf:
  - if:
      properties:
        compatible:
          contains:
            enum:
              - adi,ad7150
              - adi,ad7156
    then:
      properties:
        interrupts:
          minItems: 2
          maxItems: 2
  - if:
      properties:
        compatible:
          contains:
            const: adi,ad7151
    then:
      properties:
        interrupts:
          minItems: 1
          maxItems: 1
```

---

## 4. Kconfig

Add the entry to `drivers/iio/cdc/Kconfig` in **alphabetical order** within
the `menu "Capacitance to digital converters"` block.

```kconfig
config <DEVNAME_UPPER>
	tristate "Analog Devices <DEVNAME> capacitive sensor driver"
	depends on I2C
	help
	  Say yes here to build support for Analog Devices <DEVNAME>
	  capacitance-to-digital converter. Provides direct access via
	  sysfs.

	  To compile this driver as a module, choose M here: the
	  module will be called <devname>.
```

### Common Optional Selects for CDC

| Select | When to use |
|---|---|
| `REGMAP_I2C` | When using regmap for register access |
| `IIO_BUFFER` | When implementing continuous capture |
| `IIO_TRIGGERED_BUFFER` | When implementing triggered buffer |

Note: Current CDC drivers (ad7150, ad7746) use direct `i2c_smbus_*` calls
rather than regmap, so `REGMAP_I2C` is not always needed.

---

## 5. Makefile

Add the entry to `drivers/iio/cdc/Makefile` in **alphabetical order**:

```makefile
obj-$(CONFIG_<DEVNAME_UPPER>) += <devname>.o
```

---

## 6. Driver Source (`<devname>.c`)

### Complete Skeleton (I2C CDC)

```c
// SPDX-License-Identifier: GPL-2.0+
/*
 * <DEVNAME> capacitance-to-digital converter driver
 *
 * Copyright 20XX Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/events.h>

/* ------------------------------------------------------------------ */
/* Register Definitions                                                */
/* ------------------------------------------------------------------ */

#define <DEVNAME>_REG_STATUS		0x00
#define <DEVNAME>_REG_CAP_DATA_HIGH	0x01
#define <DEVNAME>_REG_CAP_SETUP		0x07
#define <DEVNAME>_REG_EXC_SETUP		0x09
#define <DEVNAME>_REG_CFG		0x0A
#define <DEVNAME>_REG_CAPDAC		0x0B
#define <DEVNAME>_REG_CAP_OFFH		0x0D
#define <DEVNAME>_REG_CAP_GAINH		0x0F

/* Status register bits */
#define <DEVNAME>_STATUS_RDY		BIT(2)
#define <DEVNAME>_STATUS_RDYCAP		BIT(0)

/* Capacitive setup register bits */
#define <DEVNAME>_CAPSETUP_CAPEN	BIT(7)
#define <DEVNAME>_CAPSETUP_CIN2		BIT(6)
#define <DEVNAME>_CAPSETUP_CAPDIFF	BIT(5)

/* Excitation setup register bits */
#define <DEVNAME>_EXCSETUP_EXCON	BIT(6)
#define <DEVNAME>_EXCSETUP_EXCA		BIT(3)
#define <DEVNAME>_EXCSETUP_NEXCA	BIT(2)
#define <DEVNAME>_EXCSETUP_EXCB		BIT(5)
#define <DEVNAME>_EXCSETUP_NEXCB	BIT(4)
#define <DEVNAME>_EXCSETUP_EXCLVL_MASK	GENMASK(1, 0)

/* Config register fields */
#define <DEVNAME>_CONF_CAPFS_MASK	GENMASK(5, 3)
#define <DEVNAME>_CONF_MODE_MASK	GENMASK(2, 0)
#define <DEVNAME>_CONF_MODE_SINGLE_CONV	2
#define <DEVNAME>_CONF_MODE_OFFS_CAL	5
#define <DEVNAME>_CONF_MODE_GAIN_CAL	6

/* CAPDAC register fields */
#define <DEVNAME>_CAPDAC_DACEN		BIT(7)
#define <DEVNAME>_CAPDAC_DACP_MASK	GENMASK(6, 0)

/* ------------------------------------------------------------------ */
/* Device State                                                        */
/* ------------------------------------------------------------------ */

struct <devname>_chip_info {
	struct i2c_client	*client;
	struct mutex		lock;	/* Protect device state */
	u8			config;
	u8			cap_setup;
	u8			capdac[2][2];
	s8			capdac_set;
};

/* ------------------------------------------------------------------ */
/* IIO Channel Specification                                           */
/* ------------------------------------------------------------------ */

/*
 * Single-ended capacitance channel.
 *
 * info_mask_separate: RAW, CALIBSCALE, OFFSET
 *   - RAW: raw capacitance reading
 *   - CALIBSCALE: per-channel gain calibration factor
 *   - OFFSET: CAPDAC offset (per-channel, per-mode)
 *
 * info_mask_shared_by_type: CALIBBIAS, SCALE, SAMP_FREQ
 *   - CALIBBIAS: offset calibration register (shared across cap channels)
 *   - SCALE: capacitance scale in nanofarads per LSB
 *   - SAMP_FREQ: capacitance conversion rate
 */
#define <DEVNAME>_CAP_CHANNEL(_idx) {					\
	.type = IIO_CAPACITANCE,					\
	.indexed = 1,							\
	.channel = (_idx),						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
		BIT(IIO_CHAN_INFO_CALIBSCALE) |				\
		BIT(IIO_CHAN_INFO_OFFSET),				\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_CALIBBIAS) |	\
		BIT(IIO_CHAN_INFO_SCALE) |				\
		BIT(IIO_CHAN_INFO_SAMP_FREQ),				\
	.info_mask_shared_by_type_available =				\
		BIT(IIO_CHAN_INFO_SAMP_FREQ),				\
	.address = (_idx),						\
}

/*
 * Differential capacitance channel.
 *
 * Measures CINx(+) - CINx(-). Uses ZEROPOINT for the CAPDAC offset
 * instead of OFFSET (OFFSET is reserved for single-ended channels in
 * the AD7746 convention).
 */
#define <DEVNAME>_CAP_DIFF_CHANNEL(_idx, _pos, _neg) {			\
	.type = IIO_CAPACITANCE,					\
	.differential = 1,						\
	.indexed = 1,							\
	.channel = (_pos),						\
	.channel2 = (_neg),						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
		BIT(IIO_CHAN_INFO_CALIBSCALE) |				\
		BIT(IIO_CHAN_INFO_ZEROPOINT),				\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_CALIBBIAS) |	\
		BIT(IIO_CHAN_INFO_SCALE) |				\
		BIT(IIO_CHAN_INFO_SAMP_FREQ),				\
	.info_mask_shared_by_type_available =				\
		BIT(IIO_CHAN_INFO_SAMP_FREQ),				\
	.address = (_idx),						\
}

static const struct iio_chan_spec <devname>_channels[] = {
	<DEVNAME>_CAP_CHANNEL(0),
	<DEVNAME>_CAP_DIFF_CHANNEL(1, 0, 2),
	/* Add more channels as needed */
};

/* ------------------------------------------------------------------ */
/* Sample Rate Tables                                                  */
/* ------------------------------------------------------------------ */

/* Values: { update_rate_hz, conversion_time_ms + 1 } */
static const unsigned char <devname>_cap_filter_rate_table[][2] = {
	{ 91, 12 }, { 84, 13 }, { 50, 21 }, { 26, 39 },
	{ 16, 63 }, { 13, 78 }, { 11, 93 }, { 9, 111 },
};

static const int <devname>_cap_samp_freq[] = {
	91, 84, 50, 26, 16, 13, 11, 9,
};

/* ------------------------------------------------------------------ */
/* read_raw / write_raw Callbacks                                      */
/* ------------------------------------------------------------------ */

static int <devname>_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int *val, int *val2, long mask)
{
	struct <devname>_chip_info *chip = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		/*
		 * Trigger a single conversion, wait for completion,
		 * then read the 24-bit result.
		 */
		mutex_lock(&chip->lock);
		/* ... trigger conversion, read data register ... */
		mutex_unlock(&chip->lock);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		/*
		 * Capacitance scale: convert to nanofarads.
		 * Example for AD7746: 8.192 pF full scale / 2^24
		 * = 0.000488 nF/LSB = 488 pNF/LSB
		 * Expressed as IIO_VAL_INT_PLUS_NANO: 0 + 488
		 */
		*val = 0;
		*val2 = 488;   /* picofarads expressed in nanofarad scale */
		return IIO_VAL_INT_PLUS_NANO;

	case IIO_CHAN_INFO_OFFSET:
	case IIO_CHAN_INFO_ZEROPOINT:
		/*
		 * CAPDAC offset: each CAPDAC step corresponds to
		 * a fixed capacitance offset.
		 * CAPDAC_Scale = 21pF_typ / 127
		 * CIN_Scale = 8.192pF / 2^24
		 * Offset_Scale = CAPDAC_Scale / CIN_Scale = 338646
		 */
		*val = FIELD_GET(<DEVNAME>_CAPDAC_DACP_MASK,
				 chip->capdac[chan->channel][chan->differential])
			* 338646;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_CALIBSCALE:
		/*
		 * Gain calibration register. Read the 16-bit gain
		 * register and express as 1 + gain_val/2^16.
		 * IIO_VAL_INT_PLUS_MICRO: val=1, val2=gain*15625/1024
		 */
		mutex_lock(&chip->lock);
		/* ret = i2c_smbus_read_word_swapped(..., CAP_GAINH); */
		mutex_unlock(&chip->lock);
		*val = 1;
		*val2 = 0; /* (15625 * regval) / 1024 */
		return IIO_VAL_INT_PLUS_MICRO;

	case IIO_CHAN_INFO_CALIBBIAS:
		/*
		 * Offset calibration register (16-bit).
		 * Shared across all capacitance channels.
		 */
		mutex_lock(&chip->lock);
		/* ret = i2c_smbus_read_word_swapped(..., CAP_OFFH); */
		mutex_unlock(&chip->lock);
		*val = 0; /* regval */
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SAMP_FREQ: {
		int idx;

		idx = FIELD_GET(<DEVNAME>_CONF_CAPFS_MASK, chip->config);
		*val = <devname>_cap_filter_rate_table[idx][0];
		return IIO_VAL_INT;
	}
	default:
		return -EINVAL;
	}
}

static int <devname>_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val, int val2, long mask)
{
	struct <devname>_chip_info *chip = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_CALIBSCALE:
		if (val != 1)
			return -EINVAL;
		/* gain_reg = (val2 * 1024) / 15625 */
		guard(mutex)(&chip->lock);
		/* i2c_smbus_write_word_swapped(..., CAP_GAINH, ...); */
		return 0;

	case IIO_CHAN_INFO_CALIBBIAS:
		if (val < 0 || val > 0xFFFF)
			return -EINVAL;
		guard(mutex)(&chip->lock);
		/* i2c_smbus_write_word_swapped(..., CAP_OFFH, val); */
		return 0;

	case IIO_CHAN_INFO_OFFSET:
	case IIO_CHAN_INFO_ZEROPOINT:
		/*
		 * Set CAPDAC value. Convert from CIN LSBs to CAPDAC steps.
		 * val / 338646 = CAPDAC code
		 */
		guard(mutex)(&chip->lock);
		/* ... write CAPDAC register ... */
		return 0;

	case IIO_CHAN_INFO_SAMP_FREQ: {
		int i;

		if (val2)
			return -EINVAL;
		for (i = 0; i < ARRAY_SIZE(<devname>_cap_filter_rate_table); i++)
			if (val >= <devname>_cap_filter_rate_table[i][0])
				break;
		if (i >= ARRAY_SIZE(<devname>_cap_filter_rate_table))
			i = ARRAY_SIZE(<devname>_cap_filter_rate_table) - 1;

		guard(mutex)(&chip->lock);
		chip->config &= ~<DEVNAME>_CONF_CAPFS_MASK;
		chip->config |= FIELD_PREP(<DEVNAME>_CONF_CAPFS_MASK, i);
		return 0;
	}
	default:
		return -EINVAL;
	}
}

static int <devname>_read_avail(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				const int **vals, int *type,
				int *length, long mask)
{
	if (mask != IIO_CHAN_INFO_SAMP_FREQ)
		return -EINVAL;

	*vals = <devname>_cap_samp_freq;
	*length = ARRAY_SIZE(<devname>_cap_samp_freq);
	*type = IIO_VAL_INT;
	return IIO_AVAIL_LIST;
}

/* ------------------------------------------------------------------ */
/* iio_info                                                            */
/* ------------------------------------------------------------------ */

static const struct iio_info <devname>_info = {
	.read_raw = <devname>_read_raw,
	.write_raw = <devname>_write_raw,
	.read_avail = <devname>_read_avail,
};

/* ------------------------------------------------------------------ */
/* Probe                                                               */
/* ------------------------------------------------------------------ */

static int <devname>_probe(struct i2c_client *client)
{
	const struct i2c_device_id *id = i2c_client_get_device_id(client);
	struct device *dev = &client->dev;
	struct <devname>_chip_info *chip;
	struct iio_dev *indio_dev;
	unsigned char regval = 0;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*chip));
	if (!indio_dev)
		return -ENOMEM;

	chip = iio_priv(indio_dev);

	ret = devm_mutex_init(dev, &chip->lock);
	if (ret)
		return ret;

	chip->client = client;
	chip->capdac_set = -1;

	indio_dev->name = id->name;
	indio_dev->info = &<devname>_info;
	indio_dev->channels = <devname>_channels;
	indio_dev->num_channels = ARRAY_SIZE(<devname>_channels);
	indio_dev->modes = INDIO_DIRECT_MODE;

	/* Power supply */
	ret = devm_regulator_get_enable(dev, "vdd");
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to enable vdd supply\n");

	/* Parse excitation configuration from DT */
	if (device_property_read_bool(dev, "adi,exca-output-en")) {
		if (device_property_read_bool(dev, "adi,exca-output-invert"))
			regval |= <DEVNAME>_EXCSETUP_NEXCA;
		else
			regval |= <DEVNAME>_EXCSETUP_EXCA;
	}

	if (device_property_read_bool(dev, "adi,excb-output-en")) {
		if (device_property_read_bool(dev, "adi,excb-output-invert"))
			regval |= <DEVNAME>_EXCSETUP_NEXCB;
		else
			regval |= <DEVNAME>_EXCSETUP_EXCB;
	}

	/* Parse excitation voltage level */
	/* ... device_property_read_u32(dev, "adi,excitation-vdd-permille", ...); */

	ret = i2c_smbus_write_byte_data(chip->client,
					<DEVNAME>_REG_EXC_SETUP, regval);
	if (ret < 0)
		return dev_err_probe(dev, ret,
				     "Failed to configure excitation\n");

	return devm_iio_device_register(dev, indio_dev);
}

/* ------------------------------------------------------------------ */
/* Match Tables                                                        */
/* ------------------------------------------------------------------ */

static const struct i2c_device_id <devname>_ids[] = {
	{ "<devname>" },
	/* { "<devname_variant>" }, */
	{ }
};
MODULE_DEVICE_TABLE(i2c, <devname>_ids);

static const struct of_device_id <devname>_of_match[] = {
	{ .compatible = "adi,<devname>" },
	/* { .compatible = "adi,<devname_variant>" }, */
	{ }
};
MODULE_DEVICE_TABLE(of, <devname>_of_match);

static struct i2c_driver <devname>_driver = {
	.driver = {
		.name = "<devname>",
		.of_match_table = <devname>_of_match,
	},
	.probe = <devname>_probe,
	.id_table = <devname>_ids,
};
module_i2c_driver(<devname>_driver);

MODULE_AUTHOR("First Last <first.last@analog.com>");
MODULE_DESCRIPTION("Analog Devices <DEVNAME> capacitive sensor driver");
MODULE_LICENSE("GPL");
```

---

## 7. Threshold Events

CDC parts frequently support capacitance threshold events for proximity
and touch detection. The AD7150 is the canonical example: it generates
interrupts when capacitance crosses a programmed threshold.

### Event Specification

```c
#include <linux/iio/events.h>

static const struct iio_event_spec <devname>_events[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
			BIT(IIO_EV_INFO_ENABLE),
	}, {
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
			BIT(IIO_EV_INFO_ENABLE),
	}, {
		/* Adaptive threshold: sensitivity-based, auto-adjusting */
		.type = IIO_EV_TYPE_THRESH_ADAPTIVE,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
			BIT(IIO_EV_INFO_ENABLE) |
			BIT(IIO_EV_INFO_TIMEOUT),
	}, {
		.type = IIO_EV_TYPE_THRESH_ADAPTIVE,
		.dir = IIO_EV_DIR_FALLING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
			BIT(IIO_EV_INFO_ENABLE) |
			BIT(IIO_EV_INFO_TIMEOUT),
	},
};
```

### Channel with Events

```c
#define <DEVNAME>_CAP_CHANNEL_EV(_chan) {			\
	.type = IIO_CAPACITANCE,				\
	.indexed = 1,						\
	.channel = _chan,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
		BIT(IIO_CHAN_INFO_AVERAGE_RAW),			\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |	\
		BIT(IIO_CHAN_INFO_OFFSET),			\
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),\
	.event_spec = <devname>_events,				\
	.num_event_specs = ARRAY_SIZE(<devname>_events),	\
}
```

### Event Handler (IRQ)

```c
static irqreturn_t <devname>_event_handler(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct <devname>_chip_info *chip = iio_priv(indio_dev);
	s64 timestamp = iio_get_time_ns(indio_dev);
	int status;

	status = i2c_smbus_read_byte_data(chip->client,
					  <DEVNAME>_REG_STATUS);
	if (status < 0)
		return IRQ_HANDLED;

	if (status & <DEVNAME>_STATUS_OUT1)
		iio_push_event(indio_dev,
			       IIO_UNMOD_EVENT_CODE(IIO_CAPACITANCE, 0,
						    chip->type, chip->dir),
			       timestamp);

	return IRQ_HANDLED;
}
```

### iio_info with Event Callbacks

```c
static const struct iio_info <devname>_info_ev = {
	.read_raw = <devname>_read_raw,
	.read_event_config = <devname>_read_event_config,
	.write_event_config = <devname>_write_event_config,
	.read_event_value = <devname>_read_event_value,
	.write_event_value = <devname>_write_event_value,
};
```

### IRQ Request in probe()

```c
if (client->irq > 0) {
	irq_set_status_flags(client->irq, IRQ_NOAUTOEN);
	ret = devm_request_threaded_irq(dev, client->irq,
					NULL,
					<devname>_event_handler,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					"<devname>_irq",
					indio_dev);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to request IRQ\n");

	indio_dev->info = &<devname>_info_ev;
} else {
	/* No IRQ: use info struct without event callbacks */
	indio_dev->info = &<devname>_info;
}
```

---

## 8. DT Parsing

### Excitation Configuration

Parse excitation output enable/invert and voltage level from DT
properties. This follows the AD7746 pattern:

```c
static int <devname>_parse_excitation(struct device *dev,
				      struct <devname>_chip_info *chip)
{
	unsigned char regval = 0;
	unsigned int vdd_permille;
	int ret;

	/* Excitation output A */
	if (device_property_read_bool(dev, "adi,exca-output-en")) {
		if (device_property_read_bool(dev, "adi,exca-output-invert"))
			regval |= <DEVNAME>_EXCSETUP_NEXCA;
		else
			regval |= <DEVNAME>_EXCSETUP_EXCA;
	}

	/* Excitation output B */
	if (device_property_read_bool(dev, "adi,excb-output-en")) {
		if (device_property_read_bool(dev, "adi,excb-output-invert"))
			regval |= <DEVNAME>_EXCSETUP_NEXCB;
		else
			regval |= <DEVNAME>_EXCSETUP_EXCB;
	}

	/* Excitation voltage level (fraction of VDD) */
	ret = device_property_read_u32(dev, "adi,excitation-vdd-permille",
				       &vdd_permille);
	if (!ret) {
		switch (vdd_permille) {
		case 125:
			regval |= FIELD_PREP(<DEVNAME>_EXCSETUP_EXCLVL_MASK, 0);
			break;
		case 250:
			regval |= FIELD_PREP(<DEVNAME>_EXCSETUP_EXCLVL_MASK, 1);
			break;
		case 375:
			regval |= FIELD_PREP(<DEVNAME>_EXCSETUP_EXCLVL_MASK, 2);
			break;
		case 500:
			regval |= FIELD_PREP(<DEVNAME>_EXCSETUP_EXCLVL_MASK, 3);
			break;
		default:
			return dev_err_probe(dev, -EINVAL,
					     "Invalid excitation level %u\n",
					     vdd_permille);
		}
	}

	return i2c_smbus_write_byte_data(chip->client,
					 <DEVNAME>_REG_EXC_SETUP, regval);
}
```

### Input Mode Properties

| DT Property | API | Purpose |
|---|---|---|
| `adi,exca-output-en` | `device_property_read_bool()` | Enable EXCA excitation pin |
| `adi,exca-output-invert` | `device_property_read_bool()` | Invert EXCA output |
| `adi,excb-output-en` | `device_property_read_bool()` | Enable EXCB excitation pin |
| `adi,excb-output-invert` | `device_property_read_bool()` | Invert EXCB output |
| `adi,excitation-vdd-permille` | `device_property_read_u32()` | Excitation voltage as VDD permille |
| `vdd-supply` | `devm_regulator_get_enable()` | Power supply |

---

## 9. Test & Debug

### Sysfs Interface

Every IIO CDC device exposes capacitance channels through sysfs:

```
/sys/bus/iio/devices/iio:device0/
    name                                    # Device name (e.g., "ad7746")
    in_capacitance0_raw                     # Raw capacitance reading, channel 0
    in_capacitance0_calibscale              # Per-channel gain calibration
    in_capacitance0_offset                  # CAPDAC offset (single-ended)
    in_capacitance0-capacitance2_raw        # Differential (ch0 - ch2)
    in_capacitance0-capacitance2_zeropoint  # CAPDAC offset (differential)
    in_capacitance_scale                    # Shared scale (nF per LSB)
    in_capacitance_calibbias                # Shared offset calibration
    in_capacitance_sampling_frequency       # Conversion rate
    in_capacitance_sampling_frequency_available  # Available rates
```

### Reading Capacitance

```sh
# Read raw capacitance value
cat /sys/bus/iio/devices/iio:device0/in_capacitance0_raw

# Read scale (nanofarads per LSB)
cat /sys/bus/iio/devices/iio:device0/in_capacitance_scale

# Compute capacitance in nF:
#   cap_nF = raw * scale + offset * scale

# Read available sample rates
cat /sys/bus/iio/devices/iio:device0/in_capacitance_sampling_frequency_available

# Set sample rate
echo 50 > /sys/bus/iio/devices/iio:device0/in_capacitance_sampling_frequency
```

### Threshold Events

```sh
# Monitor capacitance threshold events
iio_event_monitor iio:device0

# Enable threshold event (via sysfs)
echo 1 > /sys/bus/iio/devices/iio:device0/events/in_capacitance0_thresh_rising_en

# Set threshold value
echo 5000 > /sys/bus/iio/devices/iio:device0/events/in_capacitance0_thresh_rising_value

# Adaptive threshold timeout (AD7150-style)
echo 0.01 > /sys/bus/iio/devices/iio:device0/events/in_capacitance0_thresh_adaptive_rising_timeout
```

### Calibration

```sh
# Read/write gain calibration
cat /sys/bus/iio/devices/iio:device0/in_capacitance0_calibscale
echo 1.015625 > /sys/bus/iio/devices/iio:device0/in_capacitance0_calibscale

# Read/write offset calibration (shared)
cat /sys/bus/iio/devices/iio:device0/in_capacitance_calibbias
echo 32768 > /sys/bus/iio/devices/iio:device0/in_capacitance_calibbias
```

### debugfs Register Access

```sh
# Read register 0x07
echo 0x07 > /sys/kernel/debug/iio/iio:device0/direct_reg_access
cat /sys/kernel/debug/iio/iio:device0/direct_reg_access

# Write 0x80 to register 0x07
echo 0x07 0x80 > /sys/kernel/debug/iio/iio:device0/direct_reg_access
```

### IIO Userspace Tools

```sh
# List all IIO devices and channels
iio_info

# Read a single attribute
iio_attr -d iio:device0 -c in_capacitance0_raw
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

### Scale Units

IIO base unit for capacitance is **nanofarads** (nF). CDC datasheets
typically specify resolution in **picofarads** (pF), so the scale value
must include the pF-to-nF conversion:

```c
/* Example: AD7746 full-scale 8.192 pF / 2^24 = 488 pF/LSB */
/* In nanofarads: 0.000488 nF/LSB */
*val = 0;
*val2 = 488;
return IIO_VAL_INT_PLUS_NANO;

/* Example: AD7150 */
/* Base formula gives picofarads, multiply by 1000 for nanofarad scale */
*val = 1000;
*val2 = 40944 >> 4;
return IIO_VAL_FRACTIONAL;
```

### Memory Management

Use `devm_*` (device-managed) allocations exclusively:

| Function | Purpose |
|---|---|
| `devm_iio_device_alloc()` | Allocate IIO device + private data |
| `devm_iio_device_register()` | Register IIO device (auto-unregister) |
| `devm_regulator_get_enable()` | Get and enable a regulator |
| `devm_mutex_init()` | Initialize a mutex with devm cleanup |
| `devm_request_threaded_irq()` | Request threaded IRQ for events |

### Locking

- Use `struct mutex` for protecting device state and I2C transactions.
- Initialize with `devm_mutex_init()` in `probe()`.
- Use `guard(mutex)(&chip->lock)` (scoped lock) where possible.
- Use explicit `mutex_lock()`/`mutex_unlock()` when scoped locking is
  not suitable (e.g., when goto-based error handling is needed).

### Coding Style

- Use `FIELD_GET()` / `FIELD_PREP()` with `GENMASK()` for register fields.
- Use `BIT()` for single-bit definitions.
- Use `dev_err_probe()` for all error returns in `probe()`.
- Include headers in alphabetical order within each group.
- Run `scripts/checkpatch.pl` before submitting.

---

## 11. Commit Format

### Subject Line

```
iio: cdc: <devname>: <brief description>
```

Examples:
```
iio: cdc: ad7150: add support for AD7150/1/6 capacitive sensors
iio: cdc: ad7746: fix capacitance channel scale calculation
iio: cdc: ad7746: add configurable excitation voltage support
```

### DT Binding Commit

```
dt-bindings: iio: cdc: add adi,<devname>.yaml
```

### Patch Series for a New Driver

A typical new CDC driver submission is a patch series:

1. `dt-bindings: iio: cdc: add adi,<devname>.yaml` -- DT binding
2. `iio: cdc: <devname>: add support for <DEVNAME>` -- Driver source
3. `MAINTAINERS: add entry for <DEVNAME> IIO driver` -- Maintainer entry

### Full Example

```
iio: cdc: ad7746: add support for AD7745, AD7746, and AD7747

The AD7745/AD7746/AD7747 are 24-bit capacitance-to-digital converters
with on-chip temperature sensor and voltage input monitoring. They
feature configurable excitation sources, hardware gain and offset
calibration, and programmable conversion rates.

This driver supports:
  - Single-ended and differential capacitance channels
  - Configurable excitation output pins and voltage levels
  - Hardware gain calibration (CALIBSCALE) and offset calibration (CALIBBIAS)
  - CAPDAC offset adjustment per channel
  - Programmable sample rate with available rate enumeration
  - Voltage and temperature auxiliary channels

Signed-off-by: First Last <first.last@analog.com>
```

### Commit Body Guidelines

- Wrap at 75 characters.
- Explain **why** the change is made, not just what.
- Reference datasheets when relevant.
- Include `Signed-off-by:` (use `git commit -s`).
