# Linux IIO Resolver Driver Template

Template for writing Linux kernel IIO drivers under `drivers/iio/resolver/`.
Based on the consolidated IIO template and real resolver drivers (AD2S1210,
AD2S1200, AD2S90).

---

## 1. Purpose

**IIO Resolver** drivers handle resolver-to-digital converters (RDCs) such as
the AD2S1210, AD2S1200, and AD2S90. These devices convert the analog output of
a resolver (a type of rotary position sensor) into digital position and velocity
data.

### Channel Types

| Channel Type     | IIO Constant    | Meaning                                |
|------------------|-----------------|----------------------------------------|
| Position (angle) | `IIO_ANGL`      | Angular position from the resolver     |
| Velocity         | `IIO_ANGL_VEL`  | Angular velocity (rate of rotation)    |
| Excitation       | `IIO_ALTVOLTAGE`| Excitation frequency output / monitor  |
| Phase            | `IIO_PHASE`     | Phase lock error monitoring            |

### Units

- **Position**: Radians. Scale converts raw to radians.
  `in_angl0_raw * in_angl0_scale = angle in radians`
- **Velocity**: Radians per second. Scale converts raw to rad/s.
  `in_anglvel0_raw * in_anglvel0_scale = velocity in rad/s`
- **Excitation frequency**: Hz. Exposed via `out_altvoltage0_frequency`.

### Typical info_mask Bits

| Channel       | Separate Mask Bits                                   |
|---------------|------------------------------------------------------|
| `IIO_ANGL`    | `RAW`, `SCALE`, `HYSTERESIS`                         |
| `IIO_ANGL_VEL`| `RAW`, `SCALE`                                       |
| `IIO_ALTVOLTAGE` (output) | `FREQUENCY`                             |

---

## 2. File Checklist

| File                                                                  | Action   | Required |
|-----------------------------------------------------------------------|----------|----------|
| `drivers/iio/resolver/<devname>.c`                                    | Create   | Yes      |
| `drivers/iio/resolver/Kconfig`                                        | Modify   | Yes      |
| `drivers/iio/resolver/Makefile`                                       | Modify   | Yes      |
| `Documentation/devicetree/bindings/iio/resolver/adi,<devname>.yaml`   | Create   | Yes      |
| `MAINTAINERS`                                                         | Modify   | Yes      |

### Notes

- No header file is typically needed; resolver drivers are self-contained.
- If the driver exports symbols (e.g. a shared library for a chip family),
  a header under `include/linux/iio/resolver/` may be added.

---

## 3. Devicetree Binding

Resolver-to-digital converters are always SPI bus devices. Bindings live under
`Documentation/devicetree/bindings/iio/resolver/adi,<devname>.yaml`.

Key resolver-specific properties beyond standard SPI:

| Property                    | Type          | Purpose                                       |
|-----------------------------|---------------|-----------------------------------------------|
| `clocks`                    | phandle       | External oscillator clock (CLKIN)              |
| `assigned-resolution-bits`  | u32 enum      | Output resolution: 10, 12, 14, or 16 bits     |
| `sample-gpios`              | GPIO          | GPIO to `/SAMPLE` pin (trigger data latch)     |
| `mode-gpios`                | GPIO array    | GPIOs to A0/A1 mode select pins (optional)     |
| `resolution-gpios`          | GPIO array    | GPIOs to RES0/RES1 pins (optional)             |
| `reset-gpios`               | GPIO          | GPIO to `/RESET` pin (optional)                |
| `fault-gpios`               | GPIO array    | GPIOs to LOT/DOS fault output pins (optional)  |
| `adi,fixed-mode`            | string enum   | Mode when A0/A1 are hardwired (alternative to mode-gpios) |
| `avdd-supply`               | phandle       | Analog supply regulator                        |
| `dvdd-supply`               | phandle       | Digital supply regulator                       |
| `vdrive-supply`             | phandle       | Logic I/O supply regulator                     |

### Example Binding

```yaml
# SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)
%YAML 1.2
---
$id: http://devicetree.org/schemas/iio/resolver/adi,adXXXX.yaml#
$schema: http://devicetree.org/meta-schemas/core.yaml#

title: Analog Devices ADXXXX Resolver-to-Digital Converter

maintainers:
  - First Last <first.last@analog.com>

description: |
  The ADXXXX is a 10-bit to 16-bit resolution tracking resolver-to-digital
  converter, integrating an on-board programmable sinusoidal oscillator
  that provides sine wave excitation for resolvers.

properties:
  compatible:
    const: adi,adXXXX

  reg:
    maxItems: 1

  spi-max-frequency:
    maximum: 25000000

  spi-cpha: true

  avdd-supply:
    description: Analog supply voltage.

  dvdd-supply:
    description: Digital supply voltage.

  vdrive-supply:
    description: Logic power supply input.

  clocks:
    maxItems: 1
    description: External oscillator clock (CLKIN).

  reset-gpios:
    description: GPIO connected to /RESET pin.
    maxItems: 1

  sample-gpios:
    description:
      GPIO connected to /SAMPLE pin. Active low triggers a data latch.
    maxItems: 1

  mode-gpios:
    description: GPIOs connected to A0 and A1 mode select pins.
    minItems: 2
    maxItems: 2

  resolution-gpios:
    description:
      GPIOs connected to RES0 and RES1 resolution select pins.
      If omitted, pins are assumed hard-wired to match
      assigned-resolution-bits.
    minItems: 2
    maxItems: 2

  fault-gpios:
    description:
      GPIOs connected to LOT and DOS fault output pins.
    minItems: 2
    maxItems: 2

  assigned-resolution-bits:
    description: Output resolution in bits.
    enum: [10, 12, 14, 16]

required:
  - compatible
  - reg
  - avdd-supply
  - dvdd-supply
  - vdrive-supply
  - clocks
  - sample-gpios
  - assigned-resolution-bits

allOf:
  - $ref: /schemas/spi/spi-peripheral-props.yaml#

unevaluatedProperties: false

examples:
  - |
    #include <dt-bindings/gpio/gpio.h>

    spi {
        #address-cells = <1>;
        #size-cells = <0>;

        resolver@0 {
            compatible = "adi,adXXXX";
            reg = <0>;
            spi-max-frequency = <20000000>;
            spi-cpha;
            avdd-supply = <&avdd_regulator>;
            dvdd-supply = <&dvdd_regulator>;
            vdrive-supply = <&vdrive_regulator>;
            clocks = <&ext_osc>;
            sample-gpios = <&gpio0 90 GPIO_ACTIVE_LOW>;
            mode-gpios = <&gpio0 86 0>, <&gpio0 87 0>;
            resolution-gpios = <&gpio0 88 0>, <&gpio0 89 0>;
            assigned-resolution-bits = <16>;
        };
    };
```

---

## 4. Kconfig

Add the entry to `drivers/iio/resolver/Kconfig` in **alphabetical order**,
inside the `menu "Resolver to digital converters"` block.

```kconfig
config ADXXXX
	tristate "Analog Devices ADXXXX resolver-to-digital converter driver"
	depends on SPI
	depends on COMMON_CLK
	depends on GPIOLIB || COMPILE_TEST
	select REGMAP
	select IIO_BUFFER
	select IIO_TRIGGERED_BUFFER
	help
	  Say yes here to build support for the Analog Devices ADXXXX
	  resolver-to-digital converter. This provides angular position
	  and velocity measurement with 10-bit to 16-bit resolution.

	  To compile this driver as a module, choose M here: the
	  module will be called adXXXX.
```

### Common Resolver Dependencies/Selects

| Entry                  | Purpose                                              |
|------------------------|------------------------------------------------------|
| `depends on SPI`       | All resolver RDCs use SPI                            |
| `depends on COMMON_CLK`| External oscillator clock framework                  |
| `depends on GPIOLIB`   | SAMPLE, mode, resolution, fault GPIOs                |
| `select REGMAP`        | Register map abstraction                             |
| `select IIO_BUFFER`    | Buffered capture of position+velocity                |
| `select IIO_TRIGGERED_BUFFER` | Triggered buffer for continuous sampling      |

---

## 5. Makefile

Add the entry to `drivers/iio/resolver/Makefile` in **alphabetical order**.

```makefile
# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile for Resolver/Synchro drivers
#

obj-$(CONFIG_AD2S90) += ad2s90.o
obj-$(CONFIG_AD2S1200) += ad2s1200.o
obj-$(CONFIG_AD2S1210) += ad2s1210.o
obj-$(CONFIG_ADXXXX) += adXXXX.o
```

---

## 6. Driver Source

### Complete Skeleton

```c
// SPDX-License-Identifier: GPL-2.0
/*
 * ADXXXX Resolver-to-Digital Converter driver
 *
 * Copyright (c) 2024 Analog Devices Inc.
 *
 * Device register to IIO ABI mapping:
 *
 * Register                    | Addr | IIO ABI (sysfs)
 * ----------------------------|------|-------------------------------------------
 * Position MSB/LSB            | 0x80 | in_angl0_raw
 * Velocity MSB/LSB            | 0x82 | in_anglvel0_raw
 * DOS Overrange Threshold     | 0x89 | events/in_altvoltage0_thresh_rising_value
 * DOS Mismatch Threshold      | 0x8A | events/in_altvoltage0_mag_rising_value
 * LOT High Threshold          | 0x8D | events/in_angl1_thresh_rising_value
 * LOT Low Threshold           | 0x8E | events/in_angl1_thresh_rising_hysteresis
 * Excitation Frequency        | 0x91 | out_altvoltage0_frequency
 * Control                     | 0x92 | (bit fields)
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>
#include <linux/types.h>

#include <linux/iio/buffer.h>
#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

/* ------------------------------------------------------------------ */
/* Register Definitions                                                */
/* ------------------------------------------------------------------ */

#define ADXXXX_REG_POSITION_MSB		0x80
#define ADXXXX_REG_POSITION_LSB		0x81
#define ADXXXX_REG_VELOCITY_MSB		0x82
#define ADXXXX_REG_VELOCITY_LSB		0x83
#define ADXXXX_REG_LOS_THRD		0x88
#define ADXXXX_REG_DOS_OVR_THRD	0x89
#define ADXXXX_REG_DOS_MIS_THRD	0x8A
#define ADXXXX_REG_DOS_RST_MAX_THRD	0x8B
#define ADXXXX_REG_DOS_RST_MIN_THRD	0x8C
#define ADXXXX_REG_LOT_HIGH_THRD	0x8D
#define ADXXXX_REG_LOT_LOW_THRD	0x8E
#define ADXXXX_REG_EXCIT_FREQ		0x91
#define ADXXXX_REG_CONTROL		0x92
#define ADXXXX_REG_SOFT_RESET		0xF0
#define ADXXXX_REG_FAULT		0xFF

/* Control register fields */
#define ADXXXX_PHASE_LOCK_RANGE_44	BIT(5)
#define ADXXXX_ENABLE_HYSTERESIS	BIT(4)
#define ADXXXX_SET_ENRES		GENMASK(3, 2)
#define ADXXXX_SET_RES			GENMASK(1, 0)

/* Fault register flags */
#define ADXXXX_FAULT_CLIP		BIT(7)
#define ADXXXX_FAULT_LOS		BIT(6)
#define ADXXXX_FAULT_DOS_OVR		BIT(5)
#define ADXXXX_FAULT_DOS_MIS		BIT(4)
#define ADXXXX_FAULT_LOT		BIT(3)
#define ADXXXX_FAULT_VELOCITY		BIT(2)
#define ADXXXX_FAULT_PHASE		BIT(1)
#define ADXXXX_FAULT_CONFIG_PARITY	BIT(0)

/* Excitation frequency constraints */
#define ADXXXX_MIN_EXCIT	2000
#define ADXXXX_DEF_EXCIT	10000
#define ADXXXX_MAX_EXCIT	20000

/* Resolution enumeration (maps to control register bits) */
enum adxxxx_resolution {
	ADXXXX_RES_10 = 0b00,
	ADXXXX_RES_12 = 0b01,
	ADXXXX_RES_14 = 0b10,
	ADXXXX_RES_16 = 0b11,
};

/* ------------------------------------------------------------------ */
/* Device State                                                        */
/* ------------------------------------------------------------------ */

struct adxxxx_state {
	struct mutex lock;
	struct spi_device *sdev;
	/** GPIO pin connected to /SAMPLE line. */
	struct gpio_desc *sample_gpio;
	/** GPIO pins connected to A0 and A1 mode lines (optional). */
	struct gpio_descs *mode_gpios;
	/** Used to access config registers. */
	struct regmap *regmap;
	/** External oscillator frequency in Hz. */
	unsigned long clkin_hz;
	/** Available raw hysteresis values based on resolution. */
	int hysteresis_available[2];
	/** Selected resolution. */
	enum adxxxx_resolution resolution;
	/** Copy of fault register from previous read. */
	u8 prev_fault_flags;
	/** For reading raw sample value via SPI. */
	struct {
		__be16 raw;
		u8 fault;
	} sample __aligned(IIO_DMA_MINALIGN);
	/** Scan buffer for triggered buffer. */
	struct {
		__be16 chan[2];
		s64 timestamp __aligned(8);
	} scan;
	/** SPI transmit/receive buffers. */
	u8 rx[2];
	u8 tx[2];
};

/* ------------------------------------------------------------------ */
/* SAMPLE Line Control                                                 */
/* ------------------------------------------------------------------ */

/*
 * Toggle the SAMPLE line to latch in current position, velocity, and faults.
 *
 * Must be called with lock held.
 */
static void adxxxx_toggle_sample_line(struct adxxxx_state *st)
{
	gpiod_set_value(st->sample_gpio, 1);
	ndelay(350);
	gpiod_set_value(st->sample_gpio, 0);
	ndelay(350);
}

/* ------------------------------------------------------------------ */
/* Excitation Frequency Configuration                                  */
/* ------------------------------------------------------------------ */

static int adxxxx_reinit_excitation_frequency(struct adxxxx_state *st,
					      u16 fexcit)
{
	unsigned int ignored;
	int ret;
	u8 fcw;

	fcw = fexcit * (1 << 15) / st->clkin_hz;
	/* ... validate FCW range ... */

	ret = regmap_write(st->regmap, ADXXXX_REG_EXCIT_FREQ, fcw);
	if (ret < 0)
		return ret;

	/* Software reset reinitializes the excitation frequency output. */
	ret = regmap_write(st->regmap, ADXXXX_REG_SOFT_RESET, 0);
	if (ret < 0)
		return ret;

	/* Delay for tracking to settle, then clear faults. */
	msleep(60);

	ret = regmap_read(st->regmap, ADXXXX_REG_FAULT, &ignored);
	if (ret < 0)
		return ret;

	adxxxx_toggle_sample_line(st);
	return 0;
}

/* ------------------------------------------------------------------ */
/* IIO Channel Specification                                           */
/* ------------------------------------------------------------------ */

/*
 * Position channel: IIO_ANGL, channel 0.
 *   - Raw value: 16-bit unsigned, represents 0 to 2*pi radians.
 *   - Scale: 2*pi / 2^16 ~= 0.000095874 radians/LSB (as nano: 95874).
 *
 * Velocity channel: IIO_ANGL_VEL, channel 0.
 *   - Raw value: 16-bit signed, represents angular velocity.
 *   - Scale: depends on clkin_hz and resolution.
 */

static const struct iio_event_spec adxxxx_velocity_event_spec[] = {
	{
		/* Velocity exceeds maximum tracking rate. */
		.type = IIO_EV_TYPE_MAG,
		.dir = IIO_EV_DIR_RISING,
	},
};

static const struct iio_event_spec adxxxx_position_event_spec[] = {
	{
		/* Tracking error exceeds LOT threshold. */
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate =
			BIT(IIO_EV_INFO_VALUE) |
			BIT(IIO_EV_INFO_HYSTERESIS),
	},
};

static const struct iio_event_spec adxxxx_phase_event_spec[] = {
	{
		/* Phase error exceeds phase lock range. */
		.type = IIO_EV_TYPE_MAG,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE),
	},
};

static const struct iio_event_spec adxxxx_monitor_signal_event_spec[] = {
	{
		/* Sine/cosine below LOS threshold. */
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE),
	},
	{
		/* Sine/cosine DOS overrange. */
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE),
	},
	{
		/* Sine/cosine DOS mismatch. */
		.type = IIO_EV_TYPE_MAG,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE),
	},
};

static const struct iio_event_spec adxxxx_sin_cos_event_spec[] = {
	{
		/* Sine/cosine clipping fault. */
		.type = IIO_EV_TYPE_MAG,
		.dir = IIO_EV_DIR_EITHER,
	},
};

static const struct iio_chan_spec adxxxx_channels[] = {
	{
		/* Position (angle) output. */
		.type = IIO_ANGL,
		.indexed = 1,
		.channel = 0,
		.scan_index = 0,
		.scan_type = {
			.sign = 'u',
			.realbits = 16,
			.storagebits = 16,
			.endianness = IIO_BE,
		},
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE) |
				      BIT(IIO_CHAN_INFO_HYSTERESIS),
		.info_mask_separate_available =
					BIT(IIO_CHAN_INFO_HYSTERESIS),
	}, {
		/* Velocity (angular rate) output. */
		.type = IIO_ANGL_VEL,
		.indexed = 1,
		.channel = 0,
		.scan_index = 1,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.endianness = IIO_BE,
		},
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
		.event_spec = adxxxx_velocity_event_spec,
		.num_event_specs = ARRAY_SIZE(adxxxx_velocity_event_spec),
	},
	IIO_CHAN_SOFT_TIMESTAMP(2),
	{
		/* Used to configure LOT thresholds / tracking error. */
		.type = IIO_ANGL,
		.indexed = 1,
		.channel = 1,
		.scan_index = -1,
		.event_spec = adxxxx_position_event_spec,
		.num_event_specs = ARRAY_SIZE(adxxxx_position_event_spec),
	},
	{
		/* Phase lock error monitoring. */
		.type = IIO_PHASE,
		.indexed = 1,
		.channel = 0,
		.scan_index = -1,
		.event_spec = adxxxx_phase_event_spec,
		.num_event_specs = ARRAY_SIZE(adxxxx_phase_event_spec),
	}, {
		/* Excitation frequency output. */
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.channel = 0,
		.output = 1,
		.scan_index = -1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_FREQUENCY),
		.info_mask_separate_available = BIT(IIO_CHAN_INFO_FREQUENCY),
	}, {
		/* Monitor signal (sine/cosine combined). */
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.channel = 0,
		.scan_index = -1,
		.event_spec = adxxxx_monitor_signal_event_spec,
		.num_event_specs = ARRAY_SIZE(adxxxx_monitor_signal_event_spec),
	}, {
		/* Sine input. */
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.channel = 1,
		.scan_index = -1,
		.event_spec = adxxxx_sin_cos_event_spec,
		.num_event_specs = ARRAY_SIZE(adxxxx_sin_cos_event_spec),
	}, {
		/* Cosine input. */
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.channel = 2,
		.scan_index = -1,
		.event_spec = adxxxx_sin_cos_event_spec,
		.num_event_specs = ARRAY_SIZE(adxxxx_sin_cos_event_spec),
	},
};

/* ------------------------------------------------------------------ */
/* read_raw / write_raw / read_avail                                   */
/* ------------------------------------------------------------------ */

static int adxxxx_single_conversion(struct iio_dev *indio_dev,
				    struct iio_chan_spec const *chan,
				    int *val)
{
	struct adxxxx_state *st = iio_priv(indio_dev);

	guard(mutex)(&st->lock);

	adxxxx_toggle_sample_line(st);

	switch (chan->type) {
	case IIO_ANGL:
		/* Read position register pair. */
		/* ... regmap_bulk_read or spi_read depending on mode ... */
		*val = be16_to_cpu(st->sample.raw);
		return IIO_VAL_INT;
	case IIO_ANGL_VEL:
		/* Read velocity register pair. */
		/* ... regmap_bulk_read or spi_read depending on mode ... */
		*val = (s16)be16_to_cpu(st->sample.raw);
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int adxxxx_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	struct adxxxx_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		return adxxxx_single_conversion(indio_dev, chan, val);
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_ANGL:
			/*
			 * Scale: 2*pi / 2^16 radians per LSB.
			 * Expressed as IIO_VAL_INT_PLUS_NANO:
			 *   0.000095874 rad/LSB -> val=0, val2=95874.
			 */
			*val = 0;
			*val2 = 95874;
			return IIO_VAL_INT_PLUS_NANO;
		case IIO_ANGL_VEL:
			/*
			 * Scale depends on clkin_hz and resolution.
			 * clkin / velocity_divider -> rad/s per LSB.
			 */
			*val = st->clkin_hz;
			*val2 = /* velocity scale factor */;
			return IIO_VAL_FRACTIONAL;
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_FREQUENCY:
		switch (chan->type) {
		case IIO_ALTVOLTAGE:
			return adxxxx_get_excitation_frequency(st, val);
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_HYSTERESIS:
		switch (chan->type) {
		case IIO_ANGL:
			return adxxxx_get_hysteresis(st, val);
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
	struct adxxxx_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_FREQUENCY:
		switch (chan->type) {
		case IIO_ALTVOLTAGE:
			return adxxxx_set_excitation_frequency(st, val);
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_HYSTERESIS:
		switch (chan->type) {
		case IIO_ANGL:
			return adxxxx_set_hysteresis(st, val);
		default:
			return -EINVAL;
		}
	default:
		return -EINVAL;
	}
}

static int adxxxx_read_avail(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     const int **vals, int *type,
			     int *length, long mask)
{
	static const int excitation_frequency_available[] = {
		ADXXXX_MIN_EXCIT,
		250, /* step */
		ADXXXX_MAX_EXCIT,
	};
	struct adxxxx_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_FREQUENCY:
		*type = IIO_VAL_INT;
		*vals = excitation_frequency_available;
		return IIO_AVAIL_RANGE;
	case IIO_CHAN_INFO_HYSTERESIS:
		*vals = st->hysteresis_available;
		*type = IIO_VAL_INT;
		*length = ARRAY_SIZE(st->hysteresis_available);
		return IIO_AVAIL_LIST;
	default:
		return -EINVAL;
	}
}

/* ------------------------------------------------------------------ */
/* Channel Labels                                                      */
/* ------------------------------------------------------------------ */

static int adxxxx_read_label(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     char *label)
{
	if (chan->type == IIO_ANGL) {
		if (chan->channel == 0)
			return sprintf(label, "position\n");
		if (chan->channel == 1)
			return sprintf(label, "tracking error\n");
	}
	if (chan->type == IIO_ANGL_VEL)
		return sprintf(label, "velocity\n");
	if (chan->type == IIO_PHASE)
		return sprintf(label, "synthetic reference\n");
	if (chan->type == IIO_ALTVOLTAGE) {
		if (chan->output)
			return sprintf(label, "excitation\n");
		if (chan->channel == 0)
			return sprintf(label, "monitor signal\n");
		if (chan->channel == 1)
			return sprintf(label, "cosine\n");
		if (chan->channel == 2)
			return sprintf(label, "sine\n");
	}

	return -EINVAL;
}

/* ------------------------------------------------------------------ */
/* IIO Info & Probe                                                    */
/* ------------------------------------------------------------------ */

static const struct iio_info adxxxx_info = {
	.event_attrs = &adxxxx_event_attribute_group,
	.read_raw = adxxxx_read_raw,
	.read_avail = adxxxx_read_avail,
	.write_raw = adxxxx_write_raw,
	.read_label = adxxxx_read_label,
	.read_event_value = adxxxx_read_event_value,
	.write_event_value = adxxxx_write_event_value,
	.read_event_label = adxxxx_read_event_label,
	.debugfs_reg_access = adxxxx_debugfs_reg_access,
};

static int adxxxx_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct adxxxx_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	ret = devm_mutex_init(&spi->dev, &st->lock);
	if (ret)
		return ret;

	st->sdev = spi;

	/* Parse devicetree properties. */
	ret = adxxxx_setup_properties(st);
	if (ret < 0)
		return ret;

	/* Get and validate external oscillator clock. */
	ret = adxxxx_setup_clocks(st);
	if (ret < 0)
		return ret;

	/* Request and configure GPIOs (sample, mode, resolution, reset). */
	ret = adxxxx_setup_gpios(st);
	if (ret < 0)
		return ret;

	/* Initialize regmap for register access. */
	ret = adxxxx_setup_regmap(st);
	if (ret < 0)
		return ret;

	/* Hardware init: set resolution and excitation frequency. */
	ret = adxxxx_initial(st);
	if (ret < 0)
		return ret;

	indio_dev->info = &adxxxx_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = adxxxx_channels;
	indio_dev->num_channels = ARRAY_SIZE(adxxxx_channels);
	indio_dev->name = spi_get_device_id(spi)->name;

	ret = devm_iio_triggered_buffer_setup(&spi->dev, indio_dev,
					      &iio_pollfunc_store_time,
					      &adxxxx_trigger_handler, NULL);
	if (ret < 0)
		return dev_err_probe(&spi->dev, ret,
				     "iio triggered buffer setup failed\n");

	return devm_iio_device_register(&spi->dev, indio_dev);
}

/* ------------------------------------------------------------------ */
/* Match Tables                                                        */
/* ------------------------------------------------------------------ */

static const struct of_device_id adxxxx_of_match[] = {
	{ .compatible = "adi,adXXXX" },
	{ }
};
MODULE_DEVICE_TABLE(of, adxxxx_of_match);

static const struct spi_device_id adxxxx_id[] = {
	{ "adXXXX" },
	{ }
};
MODULE_DEVICE_TABLE(spi, adxxxx_id);

static struct spi_driver adxxxx_driver = {
	.driver = {
		.name = "adXXXX",
		.of_match_table = adxxxx_of_match,
	},
	.probe = adxxxx_probe,
	.id_table = adxxxx_id,
};
module_spi_driver(adxxxx_driver);

MODULE_AUTHOR("First Last <first.last@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADXXXX Resolver to Digital SPI driver");
MODULE_LICENSE("GPL v2");
```

### Key Resolver-Specific Patterns

**Position channel (IIO_ANGL)**:
- Unsigned 16-bit raw value representing 0 to 2*pi radians.
- Scale converts to radians: `raw * scale = angle_in_radians`.
- `scan_type.sign = 'u'` -- position is always unsigned (0 to 360 degrees).
- Supports hysteresis (position jitter reduction).

**Velocity channel (IIO_ANGL_VEL)**:
- Signed 16-bit raw value representing angular velocity.
- `scan_type.sign = 's'` -- velocity can be positive or negative.
- Scale converts to rad/s: `raw * scale = velocity_in_rad_per_s`.
- Scale depends on `clkin_hz` and resolution.

**Excitation frequency (IIO_ALTVOLTAGE, output)**:
- An output channel (`.output = 1`) used to configure the excitation sine wave.
- Exposed via `IIO_CHAN_INFO_FREQUENCY` as `out_altvoltage0_frequency`.
- Written via `write_raw`, read via `read_raw`.
- Available range exposed via `read_avail` as `IIO_AVAIL_RANGE`.

**Resolution**:
- Configured via devicetree `assigned-resolution-bits` property.
- Affects position precision, velocity scale, and tracking performance.
- Stored as an enum: `RES_10`, `RES_12`, `RES_14`, `RES_16`.

---

## 7. Fault Detection

Resolver drivers expose extensive fault detection through IIO events. Faults
are read from the fault register after each sample and pushed as IIO events.

### Fault Register Bits

| Bit | Fault                              | IIO Channel     | Event Type | Direction |
|-----|------------------------------------|-----------------|------------|-----------|
| D7  | Sine/cosine clipping               | altvoltage1/2   | MAG        | EITHER    |
| D6  | Sine/cosine below LOS threshold    | altvoltage0     | THRESH     | FALLING   |
| D5  | Sine/cosine DOS overrange          | altvoltage0     | THRESH     | RISING    |
| D4  | Sine/cosine DOS mismatch           | altvoltage0     | MAG        | RISING    |
| D3  | Tracking error exceeds LOT         | angl1           | THRESH     | RISING    |
| D2  | Velocity exceeds max tracking rate | anglvel0        | MAG        | RISING    |
| D1  | Phase error exceeds lock range     | phase0          | MAG        | RISING    |
| D0  | Configuration parity error         | (kernel log)    | --         | --        |

### Pushing Fault Events

```c
#define FAULT_ONESHOT(bit, new, old) (new & bit && !(old & bit))

static void adxxxx_push_events(struct iio_dev *indio_dev,
			       u8 flags, s64 timestamp)
{
	struct adxxxx_state *st = iio_priv(indio_dev);

	/* Sine/cosine below LOS threshold */
	if (FAULT_ONESHOT(ADXXXX_FAULT_LOS, flags, st->prev_fault_flags))
		iio_push_event(indio_dev,
			       IIO_UNMOD_EVENT_CODE(IIO_ALTVOLTAGE, 0,
						    IIO_EV_TYPE_THRESH,
						    IIO_EV_DIR_FALLING),
			       timestamp);

	/* Sine/cosine DOS overrange */
	if (FAULT_ONESHOT(ADXXXX_FAULT_DOS_OVR, flags, st->prev_fault_flags))
		iio_push_event(indio_dev,
			       IIO_UNMOD_EVENT_CODE(IIO_ALTVOLTAGE, 0,
						    IIO_EV_TYPE_THRESH,
						    IIO_EV_DIR_RISING),
			       timestamp);

	/* Sine/cosine DOS mismatch */
	if (FAULT_ONESHOT(ADXXXX_FAULT_DOS_MIS, flags, st->prev_fault_flags))
		iio_push_event(indio_dev,
			       IIO_UNMOD_EVENT_CODE(IIO_ALTVOLTAGE, 0,
						    IIO_EV_TYPE_MAG,
						    IIO_EV_DIR_RISING),
			       timestamp);

	/* Tracking error exceeds LOT threshold */
	if (FAULT_ONESHOT(ADXXXX_FAULT_LOT, flags, st->prev_fault_flags))
		iio_push_event(indio_dev,
			       IIO_UNMOD_EVENT_CODE(IIO_ANGL, 1,
						    IIO_EV_TYPE_THRESH,
						    IIO_EV_DIR_RISING),
			       timestamp);

	/* Velocity exceeds maximum tracking rate */
	if (FAULT_ONESHOT(ADXXXX_FAULT_VELOCITY, flags, st->prev_fault_flags))
		iio_push_event(indio_dev,
			       IIO_UNMOD_EVENT_CODE(IIO_ANGL_VEL, 0,
						    IIO_EV_TYPE_THRESH,
						    IIO_EV_DIR_RISING),
			       timestamp);

	/* Phase error exceeds phase lock range */
	if (FAULT_ONESHOT(ADXXXX_FAULT_PHASE, flags, st->prev_fault_flags))
		iio_push_event(indio_dev,
			       IIO_UNMOD_EVENT_CODE(IIO_PHASE, 0,
						    IIO_EV_TYPE_MAG,
						    IIO_EV_DIR_RISING),
			       timestamp);

	/* Configuration parity error */
	if (FAULT_ONESHOT(ADXXXX_FAULT_CONFIG_PARITY, flags,
			  st->prev_fault_flags))
		dev_err_ratelimited(&indio_dev->dev,
				    "Configuration parity error\n");

	st->prev_fault_flags = flags;
}
```

### Threshold Configuration via IIO Events

Fault thresholds are exposed through `read_event_value` / `write_event_value`
callbacks rather than regular `read_raw` / `write_raw`. This is because they
configure event detection thresholds, not measurement data.

| sysfs path                                       | Register             | Unit |
|--------------------------------------------------|----------------------|------|
| `events/in_altvoltage0_thresh_falling_value`     | LOS threshold        | mV   |
| `events/in_altvoltage0_thresh_rising_value`      | DOS overrange thresh | mV   |
| `events/in_altvoltage0_mag_rising_value`         | DOS mismatch thresh  | mV   |
| `events/in_angl1_thresh_rising_value`            | LOT high threshold   | urad |
| `events/in_angl1_thresh_rising_hysteresis`       | LOT low threshold    | urad |
| `events/in_phase0_mag_rising_value`              | Phase lock range     | rad  |

### Custom Event Attributes (DOS Reset Thresholds)

Some threshold registers (like DOS reset max/min) do not map to standard IIO
event info types. These use custom `IIO_DEVICE_ATTR`:

```c
static IIO_DEVICE_ATTR(in_altvoltage0_mag_rising_reset_max, 0644,
		       event_attr_voltage_reg_show, event_attr_voltage_reg_store,
		       ADXXXX_REG_DOS_RST_MAX_THRD);
static IIO_DEVICE_ATTR(in_altvoltage0_mag_rising_reset_min, 0644,
		       event_attr_voltage_reg_show, event_attr_voltage_reg_store,
		       ADXXXX_REG_DOS_RST_MIN_THRD);
```

---

## 8. Devicetree Parsing

### Resolution

```c
static int adxxxx_setup_properties(struct adxxxx_state *st)
{
	struct device *dev = &st->sdev->dev;
	u32 val;
	int ret;

	ret = device_property_read_u32(dev, "assigned-resolution-bits", &val);
	if (ret < 0)
		return dev_err_probe(dev, ret,
			"failed to read assigned-resolution-bits property\n");

	if (val < 10 || val > 16)
		return dev_err_probe(dev, -EINVAL,
				     "resolution out of range: %u\n", val);

	/* Map 10/12/14/16 to enum 0/1/2/3. */
	st->resolution = (val - 10) >> 1;

	/*
	 * Hysteresis available values: 0 (disabled), 1 (enabled).
	 * When enabled, actual hysteresis is +/- 1 LSB where LSB
	 * depends on resolution.
	 */
	st->hysteresis_available[0] = 0;
	st->hysteresis_available[1] = 1 << (2 * (ADXXXX_RES_16 -
						  st->resolution));

	return 0;
}
```

### Excitation Clock

```c
static int adxxxx_setup_clocks(struct adxxxx_state *st)
{
	struct device *dev = &st->sdev->dev;
	struct clk *clk;

	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk),
				     "failed to get clock\n");

	st->clkin_hz = clk_get_rate(clk);
	if (st->clkin_hz < ADXXXX_MIN_CLKIN ||
	    st->clkin_hz > ADXXXX_MAX_CLKIN)
		return dev_err_probe(dev, -EINVAL,
				     "clock frequency out of range: %lu\n",
				     st->clkin_hz);

	return 0;
}
```

### GPIO Setup (Sample, Mode, Resolution, Reset, Fault)

```c
static int adxxxx_setup_gpios(struct adxxxx_state *st)
{
	struct device *dev = &st->sdev->dev;

	/* /SAMPLE pin -- mandatory, active low. */
	st->sample_gpio = devm_gpiod_get(dev, "sample", GPIOD_OUT_LOW);
	if (IS_ERR(st->sample_gpio))
		return dev_err_probe(dev, PTR_ERR(st->sample_gpio),
				     "failed to request sample GPIO\n");

	/* A0/A1 mode pins -- optional (alternative: adi,fixed-mode). */
	st->mode_gpios = devm_gpiod_get_array_optional(dev, "mode",
							GPIOD_OUT_HIGH);
	if (IS_ERR(st->mode_gpios))
		return dev_err_probe(dev, PTR_ERR(st->mode_gpios),
				     "failed to request mode GPIOs\n");

	/* RES0/RES1 resolution pins -- optional (may be hard-wired). */
	/* Set to match assigned-resolution-bits if provided. */

	/* /RESET pin -- optional, active low. */
	/* Toggle high then low to perform hardware reset. */

	return 0;
}
```

### Key DT Property to Driver API Mapping

| DT Property                 | Driver API                              | Notes                  |
|-----------------------------|-----------------------------------------|------------------------|
| `assigned-resolution-bits`  | `device_property_read_u32()`            | Required, 10/12/14/16  |
| `clocks`                    | `devm_clk_get_enabled()`                | Required, ext osc      |
| `sample-gpios`              | `devm_gpiod_get(dev, "sample", ...)`    | Required, /SAMPLE      |
| `mode-gpios`                | `devm_gpiod_get_array_optional()`       | Optional, A0+A1        |
| `resolution-gpios`          | `devm_gpiod_get_array_optional()`       | Optional, RES0+RES1    |
| `reset-gpios`               | `devm_gpiod_get_optional()`             | Optional, /RESET       |
| `fault-gpios`               | `devm_gpiod_get_array_optional()`       | Optional, LOT+DOS      |
| `adi,fixed-mode`            | `device_property_read_string()`         | Alternative to mode-gpios |
| `avdd-supply`               | `devm_regulator_get_enable()`           | Required               |
| `dvdd-supply`               | `devm_regulator_get_enable()`           | Required               |
| `vdrive-supply`             | `devm_regulator_get_enable()`           | Required               |

---

## 9. Test & Debug

### sysfs Interface (Resolver-Specific)

```
/sys/bus/iio/devices/iio:device0/
    name                                # e.g. "ad2s1210"
    in_angl0_raw                        # Raw position (0 .. 65535)
    in_angl0_scale                      # Scale to radians
    in_angl0_hysteresis                 # Position hysteresis (0 or LSB count)
    in_angl0_hysteresis_available       # "0 <lsb_value>"
    in_angl0_label                      # "position"
    in_anglvel0_raw                     # Raw velocity (signed, -32768 .. 32767)
    in_anglvel0_scale                   # Scale to rad/s
    in_anglvel0_label                   # "velocity"
    out_altvoltage0_frequency           # Excitation frequency in Hz
    out_altvoltage0_frequency_available # "[2000 250 20000]"
    out_altvoltage0_label               # "excitation"
```

### Reading Position and Velocity

```sh
# Read current angular position
cat /sys/bus/iio/devices/iio:device0/in_angl0_raw
# => 32768 (represents 180 degrees = pi radians)

cat /sys/bus/iio/devices/iio:device0/in_angl0_scale
# => 0.000095874

# Compute position in radians:
# 32768 * 0.000095874 = 3.14159 radians (pi)

# Read current angular velocity
cat /sys/bus/iio/devices/iio:device0/in_anglvel0_raw
# => 1000 (positive = clockwise)

cat /sys/bus/iio/devices/iio:device0/in_anglvel0_scale
# => (depends on clkin and resolution)
```

### Configuring Excitation Frequency

```sh
# Read current excitation frequency
cat /sys/bus/iio/devices/iio:device0/out_altvoltage0_frequency
# => 10000

# Set excitation frequency to 15 kHz
echo 15000 > /sys/bus/iio/devices/iio:device0/out_altvoltage0_frequency

# Check available range
cat /sys/bus/iio/devices/iio:device0/out_altvoltage0_frequency_available
# => [2000 250 20000]
```

### Monitoring Fault Events

```sh
# Use iio_event_monitor to watch for faults
iio_event_monitor /dev/iio:device0

# Example output when a fault occurs:
# Event: time: 123456789, type: thresh, chan: angl1, dir: rising
#   => LOT (loss of tracking) fault detected
# Event: time: 123456790, type: mag, chan: anglvel0, dir: rising
#   => Velocity exceeded maximum tracking rate
```

### Fault Threshold Configuration

```sh
# Read/write LOS threshold (mV)
cat /sys/bus/iio/devices/iio:device0/events/in_altvoltage0_thresh_falling_value
echo 760 > /sys/bus/iio/devices/iio:device0/events/in_altvoltage0_thresh_falling_value

# Read/write DOS overrange threshold (mV)
cat /sys/bus/iio/devices/iio:device0/events/in_altvoltage0_thresh_rising_value

# Read/write LOT thresholds (microradians)
cat /sys/bus/iio/devices/iio:device0/events/in_angl1_thresh_rising_value
cat /sys/bus/iio/devices/iio:device0/events/in_angl1_thresh_rising_hysteresis
```

### Buffered Capture

```sh
# Enable position and velocity channels in the scan
echo 1 > /sys/bus/iio/devices/iio:device0/scan_elements/in_angl0_en
echo 1 > /sys/bus/iio/devices/iio:device0/scan_elements/in_anglvel0_en
echo 1 > /sys/bus/iio/devices/iio:device0/scan_elements/in_timestamp_en

# Set buffer length and enable
echo 128 > /sys/bus/iio/devices/iio:device0/buffer0/length
echo 1 > /sys/bus/iio/devices/iio:device0/buffer0/enable

# Read buffered data
iio_readdev -b 128 iio:device0
```

### debugfs Register Access

```sh
# Read register 0x92 (control register)
echo 0x92 > /sys/kernel/debug/iio/iio:device0/direct_reg_access
cat /sys/kernel/debug/iio/iio:device0/direct_reg_access

# Read fault register (also clears faults)
echo 0xFF > /sys/kernel/debug/iio/iio:device0/direct_reg_access
cat /sys/kernel/debug/iio/iio:device0/direct_reg_access
```

---

## 10. Key Conventions

### License

All new resolver drivers must use GPL-2.0:

```c
// SPDX-License-Identifier: GPL-2.0
```

The SPDX tag goes on the very first line. `MODULE_LICENSE("GPL v2")` at
the bottom must match.

### Units

| Measurement | IIO Unit   | Conversion                          |
|-------------|------------|-------------------------------------|
| Position    | radians    | `raw * scale` where scale = 2*pi / 2^N |
| Velocity    | rad/s      | `raw * scale` (depends on clkin and resolution) |
| Excitation  | Hz         | Direct integer value                |
| Thresholds  | mV or urad | Depends on threshold type           |
| Phase       | radians    | INT_PLUS_MICRO representation       |

### Memory Management

- Use `devm_*` functions exclusively for all resource allocation.
- DMA-safe buffers must be `__aligned(IIO_DMA_MINALIGN)` and placed at the
  end of the state struct.
- Scan buffer must have naturally aligned `s64 timestamp` field:
  `s64 timestamp __aligned(8)`.

### Locking

- Use `struct mutex` to protect device state and SPI bus transactions.
- Initialize with `devm_mutex_init()` in `probe()`.
- Use `guard(mutex)(&st->lock)` (scoped lock, auto-release) where possible.
- Use `iio_device_claim_direct()` / `iio_device_release_direct()` in
  `read_raw` to prevent raw reads while the buffer is active.

### Coding Style

- Follow the kernel coding style (`Documentation/process/coding-style.rst`).
- Run `scripts/checkpatch.pl` before submitting.
- Use `FIELD_GET()` / `FIELD_PREP()` with `GENMASK()` for register bitfields.
- Use `BIT()` for single-bit flag definitions.
- Use `dev_err_probe()` for all error returns in `probe()`.
- Include headers in alphabetical order within each group (kernel headers,
  then IIO headers).

---

## 11. Commit Message Format

### Subject Line Prefix

```
iio: resolver: <devname>: <brief description>
```

### Examples

```
iio: resolver: ad2s1210: add support for AD2S1210
iio: resolver: ad2s1210: add triggered buffer support
iio: resolver: ad2s1210: fix velocity scale calculation
iio: resolver: ad2s1210: use devm_mutex_init
```

### DT Binding Commit

```
dt-bindings: iio: resolver: add adi,adXXXX.yaml
```

### Patch Series for a New Resolver Driver

A typical new resolver driver submission is a 3-patch series:

1. `dt-bindings: iio: resolver: add adi,<devname>.yaml` -- DT binding
2. `iio: resolver: <devname>: add support for <DEVNAME>` -- Driver source
3. `MAINTAINERS: add entry for <DEVNAME> IIO resolver driver` -- Maintainer entry

### Full Commit Example

```
iio: resolver: ad2s1210: add support for AD2S1210

The AD2S1210 is a complete 10-bit to 16-bit resolution tracking
resolver-to-digital converter. It integrates an on-board programmable
sinusoidal oscillator that provides sine wave excitation for resolvers.

This driver supports:
  - Angular position reading (IIO_ANGL channel)
  - Angular velocity reading (IIO_ANGL_VEL channel)
  - Configurable excitation frequency (2-20 kHz)
  - Selectable resolution (10/12/14/16 bit)
  - Fault detection via IIO events (LOT, LOS, DOS, phase, velocity)
  - Triggered buffer for continuous position/velocity capture

Signed-off-by: First Last <first.last@analog.com>
```
