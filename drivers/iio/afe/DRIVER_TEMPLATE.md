# Linux IIO AFE (Analog Front-End) Driver Template

This template covers the `iio_rescale` pattern used by drivers under
`drivers/iio/afe/`. AFE drivers are virtual IIO devices that sit between a
physical ADC (or any IIO channel provider) and userspace. They apply a
mathematical rescale -- voltage divider ratio, current sense resistor
conversion, RTD temperature linearisation -- so that sysfs exposes the
real-world quantity instead of the raw ADC voltage.

---

## 1. Purpose

IIO AFE drivers expose `IIO_VOLTAGE`, `IIO_CURRENT`, or `IIO_TEMP` channels
that **rescale an underlying IIO device's output**. They do not talk to
hardware directly; instead they consume a backend IIO channel through the
`iio_channel` consumer API and transform its readings with a numerator /
denominator scale factor and an optional offset.

The canonical implementation is `iio-rescale.c`, which handles five
compatible strings through a single platform driver:

| Compatible                  | Output Type    | Key Properties                          |
|-----------------------------|----------------|-----------------------------------------|
| `voltage-divider`           | `IIO_VOLTAGE`  | `output-ohms`, `full-ohms`              |
| `current-sense-shunt`       | `IIO_CURRENT`  | `shunt-resistor-micro-ohms`             |
| `current-sense-amplifier`   | `IIO_CURRENT`  | `sense-resistor-micro-ohms`, gain-mult/div |
| `temperature-sense-rtd`     | `IIO_TEMP`     | `excitation-current-microamp`, `alpha-ppm-per-celsius`, `r-naught-ohms` |
| `temperature-transducer`    | `IIO_TEMP`     | `alpha-ppm-per-celsius`, `sense-resistor-ohms` |

All variants follow the same architecture: read raw (or processed) from the
source channel, then multiply by `numerator / denominator` for the scale.

---

## 2. File Checklist

| File                                                                    | Action   | Required |
|-------------------------------------------------------------------------|----------|----------|
| `drivers/iio/afe/<devname>.c`                                           | Create   | Yes      |
| `drivers/iio/afe/Kconfig`                                               | Modify   | Yes      |
| `drivers/iio/afe/Makefile`                                              | Modify   | Yes      |
| `Documentation/devicetree/bindings/iio/afe/<compatible>.yaml`           | Create   | Yes      |
| `include/linux/iio/afe/<devname>.h`                                     | Create   | Optional |

### Notes

- If the new AFE type fits within the existing `iio-rescale` framework (a
  simple numerator/denominator transformation), add a new `rescale_variant`
  enum entry, a `rescale_cfg` element, and a `*_props()` function to
  `iio-rescale.c` rather than creating a new file.
- A separate source file is only warranted when the transformation is complex
  enough that it cannot be expressed as a simple scale + offset (e.g.,
  non-linear correction, lookup tables, or multi-channel interplay).
- The header in `include/linux/iio/afe/` is only needed if the driver exports
  symbols consumed by other modules (as `rescale.h` exports
  `rescale_process_scale()` and `rescale_process_offset()`).

---

## 3. DT Binding (.yaml)

AFE bindings live under `Documentation/devicetree/bindings/iio/afe/`.
The critical property is `io-channels`, which references the backend ADC
channel. Device-specific properties describe the analog front-end circuit.

### Voltage Divider Example

```yaml
# SPDX-License-Identifier: (GPL-2.0 OR BSD-2-Clause)
%YAML 1.2
---
$id: http://devicetree.org/schemas/iio/afe/voltage-divider.yaml#
$schema: http://devicetree.org/meta-schemas/core.yaml#

title: Voltage divider

maintainers:
  - First Last <first.last@analog.com>

description: |
  When an io-channel measures the midpoint of a voltage divider, the
  interesting voltage is the voltage over the full resistance of the
  divider. This binding describes such a circuit.

    Vin ----.
            |
         .-----.
         |  R  |
         '-----'
            |
            +---- Vout (to ADC)
            |
         .-----.
         | Rout|
         '-----'
            |
           GND

properties:
  compatible:
    const: voltage-divider

  io-channels:
    maxItems: 1
    description: |
      Channel node of a voltage io-channel (the backend ADC).

  '#io-channel-cells':
    const: 1

  output-ohms:
    description:
      Resistance Rout over which the output voltage is measured.

  full-ohms:
    description:
      Total resistance R + Rout. The io-channel reading is scaled by
      full-ohms / output-ohms.

required:
  - compatible
  - io-channels
  - output-ohms
  - full-ohms

additionalProperties: false

examples:
  - |
    spi {
        #address-cells = <1>;
        #size-cells = <0>;
        maxadc: adc@0 {
            compatible = "maxim,max1027";
            reg = <0>;
            #io-channel-cells = <1>;
            spi-max-frequency = <1000000>;
        };
    };
    sysv {
        compatible = "voltage-divider";
        io-channels = <&maxadc 1>;

        output-ohms = <22>;
        full-ohms = <222>; /* 200 + 22 */
    };
```

### Current Sense Shunt Example

```yaml
properties:
  compatible:
    const: current-sense-shunt

  io-channels:
    maxItems: 1

  shunt-resistor-micro-ohms:
    description: The shunt resistance in micro-ohms.

required:
  - compatible
  - io-channels
  - shunt-resistor-micro-ohms
```

### DT Node Example (combined)

```dts
i2c {
    #address-cells = <1>;
    #size-cells = <0>;

    backend_adc: adc@48 {
        compatible = "ti,ads1015";
        reg = <0x48>;
        #io-channel-cells = <1>;
    };
};

/* Voltage divider: scale ADC reading by full/output = 222/22 */
sysv {
    compatible = "voltage-divider";
    io-channels = <&backend_adc 0>;
    output-ohms = <22>;
    full-ohms = <222>;
};

/* Current sense: convert voltage to current via I = V / Rshunt */
sysi {
    compatible = "current-sense-shunt";
    io-channels = <&backend_adc 1>;
    shunt-resistor-micro-ohms = <10000>; /* 10 mOhm */
};
```

### Key Binding Properties by AFE Type

| AFE Type                   | Required DT Properties                               |
|----------------------------|------------------------------------------------------|
| `voltage-divider`          | `output-ohms`, `full-ohms`                           |
| `current-sense-shunt`      | `shunt-resistor-micro-ohms`                          |
| `current-sense-amplifier`  | `sense-resistor-micro-ohms`, opt: `sense-gain-mult`, `sense-gain-div` |
| `temperature-sense-rtd`    | `excitation-current-microamp`, `alpha-ppm-per-celsius`, `r-naught-ohms` |
| `temperature-transducer`   | `alpha-ppm-per-celsius`, opt: `sense-resistor-ohms`, `sense-offset-millicelsius` |

---

## 4. Kconfig

Add the entry to `drivers/iio/afe/Kconfig` in **alphabetical order**.

For a new variant that extends `iio-rescale`, no new Kconfig entry is needed --
the existing `IIO_RESCALE` symbol covers it.

For a standalone AFE driver:

```kconfig
config IIO_RESCALE
	tristate "IIO rescale"
	help
	  Say yes here to build support for the IIO rescaling
	  that handles voltage dividers, current sense shunts and
	  current sense amplifiers.

	  To compile this driver as a module, choose M here: the
	  module will be called iio-rescale.

config <DEVNAME>_AFE
	tristate "Analog Devices <DEVNAME> analog front end driver"
	select IIO_RESCALE
	help
	  Say yes here to build support for the Analog Devices <DEVNAME>
	  analog front end.

	  To compile this driver as a module, choose M here: the module
	  will be called <devname>-afe.
```

### Notes

- Most AFE drivers should `select IIO_RESCALE` rather than duplicate the
  rescale math, since `rescale_process_scale()` and
  `rescale_process_offset()` are exported symbols.
- AFE drivers are platform drivers with no bus dependency (`depends on SPI` /
  `depends on I2C` are not needed).

---

## 5. Makefile

Add the entry to `drivers/iio/afe/Makefile` in **alphabetical order**.

```makefile
# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile for industrial I/O Analog Front Ends (AFE)
#

# When adding new entries keep the list in alphabetical order
obj-$(CONFIG_IIO_RESCALE) += iio-rescale.o
obj-$(CONFIG_<DEVNAME>_AFE) += <devname>-afe.o
```

---

## 6. Driver Source (.c)

### Architecture Overview

An AFE driver is a **platform driver** (not SPI or I2C). It has no direct
hardware access. Instead it:

1. Obtains a reference to a backend IIO channel via `devm_iio_channel_get()`.
2. Reads the backend's raw value with `iio_read_channel_raw()`.
3. Reads the backend's scale with `iio_read_channel_scale()`.
4. Applies a numerator/denominator transformation to produce the rescaled
   value.
5. Exposes the result as a new IIO device with its own sysfs entries.

### Complete Skeleton -- Extending iio-rescale

When the new AFE fits the rescale pattern (numerator / denominator scale),
add it directly to `iio-rescale.c`:

```c
/* --- Add the props function --- */

static int rescale_<afetype>_props(struct device *dev,
                                   struct rescale *rescale)
{
	u32 param;
	u32 factor;
	int ret;

	ret = device_property_read_u32(dev, "<dt-property-name>", &param);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to read <dt-property-name>\n");

	/*
	 * Compute numerator / denominator from the DT parameters.
	 * Reduce with gcd() to avoid overflow in the rescale math.
	 */
	factor = gcd(param, 1000000);
	rescale->numerator = 1000000 / factor;
	rescale->denominator = param / factor;

	return 0;
}

/* --- Add enum entry --- */

enum rescale_variant {
	CURRENT_SENSE_AMPLIFIER,
	CURRENT_SENSE_SHUNT,
	VOLTAGE_DIVIDER,
	TEMP_SENSE_RTD,
	TEMP_TRANSDUCER,
	<AFETYPE>,            /* new */
};

/* --- Add rescale_cfg entry --- */

static const struct rescale_cfg rescale_cfg[] = {
	/* ... existing entries ... */
	[<AFETYPE>] = {
		.type = IIO_VOLTAGE, /* or IIO_CURRENT, IIO_TEMP */
		.props = rescale_<afetype>_props,
	},
};

/* --- Add of_device_id entry --- */

static const struct of_device_id rescale_match[] = {
	/* ... existing entries ... */
	{ .compatible = "<compatible-string>",
	  .data = &rescale_cfg[<AFETYPE>], },
	{ /* sentinel */ }
};
```

### Complete Skeleton -- Standalone AFE Driver

When the AFE requires logic beyond simple rescale (non-linear transforms,
multi-channel, calibration tables):

```c
// SPDX-License-Identifier: GPL-2.0
/*
 * <DEVNAME> analog front end driver
 *
 * Copyright 2024 Analog Devices Inc.
 */

#include <linux/err.h>
#include <linux/gcd.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>

#include <linux/iio/consumer.h>
#include <linux/iio/iio.h>

struct <devname>_afe {
	struct iio_channel	*source;
	struct iio_chan_spec	chan;
	s32			numerator;
	s32			denominator;
	s32			offset;
};

/* ------------------------------------------------------------------ */
/* read_raw: delegate to backend then apply AFE transformation         */
/* ------------------------------------------------------------------ */

static int <devname>_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int *val, int *val2, long mask)
{
	struct <devname>_afe *afe = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		return iio_read_channel_raw(afe->source, val);

	case IIO_CHAN_INFO_SCALE:
		/*
		 * Read the backend scale, then multiply by
		 * numerator / denominator.
		 */
		/* ... rescale_process_scale() or custom math ... */
		return IIO_VAL_FRACTIONAL;

	case IIO_CHAN_INFO_OFFSET:
		*val = afe->offset;
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

static const struct iio_info <devname>_info = {
	.read_raw = <devname>_read_raw,
};

/* ------------------------------------------------------------------ */
/* Probe                                                               */
/* ------------------------------------------------------------------ */

static int <devname>_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iio_dev *indio_dev;
	struct iio_channel *source;
	struct <devname>_afe *afe;
	int ret;

	/* Obtain the backend IIO channel */
	source = devm_iio_channel_get(dev, NULL);
	if (IS_ERR(source))
		return dev_err_probe(dev, PTR_ERR(source),
				     "failed to get source channel\n");

	indio_dev = devm_iio_device_alloc(dev, sizeof(*afe));
	if (!indio_dev)
		return -ENOMEM;

	afe = iio_priv(indio_dev);
	afe->source = source;

	/* Parse DT properties to compute numerator/denominator */
	ret = device_property_read_u32(dev, "output-ohms",
				       &afe->denominator);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to read output-ohms\n");

	ret = device_property_read_u32(dev, "full-ohms",
				       &afe->numerator);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to read full-ohms\n");

	/* Configure the virtual IIO channel */
	afe->chan.type = IIO_VOLTAGE;
	afe->chan.indexed = 1;
	afe->chan.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
		BIT(IIO_CHAN_INFO_SCALE);

	if (afe->offset)
		afe->chan.info_mask_separate |= BIT(IIO_CHAN_INFO_OFFSET);

	indio_dev->name = dev_name(dev);
	indio_dev->info = &<devname>_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = &afe->chan;
	indio_dev->num_channels = 1;

	return devm_iio_device_register(dev, indio_dev);
}

/* ------------------------------------------------------------------ */
/* Match Tables                                                        */
/* ------------------------------------------------------------------ */

static const struct of_device_id <devname>_of_match[] = {
	{ .compatible = "<compatible-string>" },
	{ }
};
MODULE_DEVICE_TABLE(of, <devname>_of_match);

static struct platform_driver <devname>_driver = {
	.probe = <devname>_probe,
	.driver = {
		.name = "<devname>-afe",
		.of_match_table = <devname>_of_match,
	},
};
module_platform_driver(<devname>_driver);

MODULE_AUTHOR("First Last <first.last@analog.com>");
MODULE_DESCRIPTION("Analog Devices <DEVNAME> analog front end driver");
MODULE_LICENSE("GPL");
```

### Voltage Divider Rescale Math

The voltage divider transforms the backend reading with:

```
V_actual = V_measured * (full_ohms / output_ohms)
```

In the driver this becomes `numerator = full_ohms`, `denominator = output_ohms`,
reduced by their GCD:

```c
static int rescale_voltage_divider_props(struct device *dev,
                                         struct rescale *rescale)
{
	u32 factor;
	int ret;

	ret = device_property_read_u32(dev, "output-ohms",
				       &rescale->denominator);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to read output-ohms\n");

	ret = device_property_read_u32(dev, "full-ohms",
				       &rescale->numerator);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to read full-ohms\n");

	factor = gcd(rescale->numerator, rescale->denominator);
	rescale->numerator /= factor;
	rescale->denominator /= factor;

	return 0;
}
```

### Current Sense Shunt Rescale Math

Converts a voltage reading across a shunt resistor to current:

```
I = V / R_shunt
```

The shunt resistance is specified in micro-ohms, so:

```
numerator = 1000000 / gcd(shunt, 1000000)
denominator = shunt / gcd(shunt, 1000000)
```

```c
static int rescale_current_sense_shunt_props(struct device *dev,
                                             struct rescale *rescale)
{
	u32 shunt;
	u32 factor;
	int ret;

	ret = device_property_read_u32(dev, "shunt-resistor-micro-ohms",
				       &shunt);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to read shunt resistance\n");

	factor = gcd(shunt, 1000000);
	rescale->numerator = 1000000 / factor;
	rescale->denominator = shunt / factor;

	return 0;
}
```

---

## 7. Backend IIO Channel

AFE drivers consume another IIO device's channel through the IIO consumer
API. The backend device must declare `#io-channel-cells` in DT; the AFE
references it with the `io-channels` property.

### Key Consumer API Functions

| Function                            | Purpose                                           |
|-------------------------------------|---------------------------------------------------|
| `devm_iio_channel_get(dev, NULL)`   | Obtain a reference to the backend channel         |
| `iio_read_channel_raw(chan, &val)`  | Read the backend's raw ADC code                   |
| `iio_read_channel_scale(chan, &v, &v2)` | Read the backend's scale factor               |
| `iio_read_channel_offset(chan, &v, &v2)` | Read the backend's offset                     |
| `iio_read_channel_processed(chan, &val)` | Read a fully processed value (raw * scale + offset) |
| `iio_convert_raw_to_processed(chan, raw, &processed, scale)` | Convert raw to processed with extra scaling |
| `iio_read_avail_channel_raw(chan, &vals, &len)` | Read available raw values from backend |
| `iio_channel_has_info(chan->channel, info)` | Check if backend exposes a given info mask bit |
| `iio_read_channel_ext_info(chan, name, buf)` | Read extended info attribute from backend |
| `iio_write_channel_ext_info(chan, name, buf, len)` | Write extended info attribute to backend |

### Consumer Header

```c
#include <linux/iio/consumer.h>
```

### Channel Acquisition in Probe

```c
struct iio_channel *source;

source = devm_iio_channel_get(dev, NULL);
if (IS_ERR(source))
	return dev_err_probe(dev, PTR_ERR(source),
			     "failed to get source channel\n");
```

The `NULL` channel name means the driver uses the first (and typically only)
`io-channels` entry in DT. For multiple channels, use named entries:

```c
source_v = devm_iio_channel_get(dev, "voltage");
source_i = devm_iio_channel_get(dev, "current");
```

with DT:

```dts
io-channels = <&adc 0>, <&adc 1>;
io-channel-names = "voltage", "current";
```

### Channel Capability Detection

The rescale driver checks what the backend supports before deciding how to
read it:

```c
if (iio_channel_has_info(schan, IIO_CHAN_INFO_RAW) &&
    (iio_channel_has_info(schan, IIO_CHAN_INFO_SCALE) ||
     iio_channel_has_info(schan, IIO_CHAN_INFO_OFFSET))) {
	/* Use raw + scale path */
} else if (iio_channel_has_info(schan, IIO_CHAN_INFO_PROCESSED)) {
	/* Use processed path */
	rescale->chan_processed = true;
} else {
	return -EINVAL;
}
```

---

## 8. DT Parsing

AFE drivers parse DT properties that describe the analog front-end circuit
parameters. There are no `channel@N` sub-nodes -- the AFE exposes exactly one
virtual channel that wraps the backend.

### Common DT Properties by AFE Type

| DT Property                        | API Call                                  | AFE Type                  |
|------------------------------------|-------------------------------------------|---------------------------|
| `io-channels`                      | `devm_iio_channel_get()`                  | All                       |
| `output-ohms`                      | `device_property_read_u32()`              | `voltage-divider`         |
| `full-ohms`                        | `device_property_read_u32()`              | `voltage-divider`         |
| `shunt-resistor-micro-ohms`        | `device_property_read_u32()`              | `current-sense-shunt`     |
| `sense-resistor-micro-ohms`        | `device_property_read_u32()`              | `current-sense-amplifier` |
| `sense-gain-mult`                  | `device_property_read_u32()`              | `current-sense-amplifier` |
| `sense-gain-div`                   | `device_property_read_u32()`              | `current-sense-amplifier` |
| `excitation-current-microamp`      | `device_property_read_u32()`              | `temperature-sense-rtd`   |
| `alpha-ppm-per-celsius`            | `device_property_read_u32()`              | RTD / transducer          |
| `r-naught-ohms`                    | `device_property_read_u32()`              | `temperature-sense-rtd`   |
| `sense-resistor-ohms`              | `device_property_read_u32()`              | `temperature-transducer`  |
| `sense-offset-millicelsius`        | `device_property_read_u32()`              | `temperature-transducer`  |

### Parsing Pattern

```c
static int rescale_my_afe_props(struct device *dev,
                                struct rescale *rescale)
{
	u32 r_out, r_full, factor;
	int ret;

	ret = device_property_read_u32(dev, "output-ohms", &r_out);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to read output-ohms\n");

	ret = device_property_read_u32(dev, "full-ohms", &r_full);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to read full-ohms\n");

	/* Reduce to avoid overflow in rescale math */
	factor = gcd(r_full, r_out);
	rescale->numerator = r_full / factor;
	rescale->denominator = r_out / factor;

	return 0;
}
```

### Validation

After parsing, validate that the numerator and denominator are non-zero:

```c
if (!rescale->numerator || !rescale->denominator) {
	dev_err(dev, "invalid scaling factor.\n");
	return -EINVAL;
}
```

---

## 9. Test & Debug

### Sysfs Interface

An AFE device appears as a standard IIO device. The sysfs entries show the
**rescaled** values, not the raw backend values:

```sh
# The AFE device shows rescaled voltage/current/temperature
ls /sys/bus/iio/devices/iio:deviceN/
    name                          # e.g. "sysv" (from DT node name)
    in_voltage0_raw               # Raw value read from backend ADC
    in_voltage0_scale             # Scale = backend_scale * (full/output)
    in_voltage0_offset            # Offset (if present)

# For current-sense-shunt
    in_current0_raw               # Raw voltage from ADC
    in_current0_scale             # Scale converts to current (mA)
```

### Verifying Rescale Math

```sh
# Read the raw and scale values
RAW=$(cat /sys/bus/iio/devices/iio:device1/in_voltage0_raw)
SCALE=$(cat /sys/bus/iio/devices/iio:device1/in_voltage0_scale)

# The actual voltage is RAW * SCALE (in millivolts)
# For a 22/222 voltage divider with a 12V system voltage:
#   Backend reads ~1.19V at the divider midpoint
#   AFE reports scale such that RAW * SCALE = ~12V
```

### Comparing Backend vs AFE

```sh
# Backend ADC device (iio:device0)
cat /sys/bus/iio/devices/iio:device0/in_voltage0_raw    # e.g. 1500
cat /sys/bus/iio/devices/iio:device0/in_voltage0_scale  # e.g. 0.805664

# AFE device (iio:device1) -- same raw, different scale
cat /sys/bus/iio/devices/iio:device1/in_voltage0_raw    # e.g. 1500
cat /sys/bus/iio/devices/iio:device1/in_voltage0_scale  # e.g. 8.130859
# 0.805664 * (222/22) = 8.130... (voltage divider applied)
```

### IIO Userspace Tools

```sh
# List all IIO devices (backend ADC + AFE will both appear)
iio_info

# Read a specific attribute
iio_attr -d iio:device1 -c in_voltage0_raw
iio_attr -d iio:device1 -c in_voltage0_scale
```

### Debugging Tips

- If the AFE device does not appear, verify that the backend ADC's
  `#io-channel-cells` is set and that `io-channels` in the AFE node
  references a valid phandle + channel index.
- Use `dev_info()` / `dev_dbg()` in the props function to print the computed
  numerator / denominator during probe.
- Check `dmesg` for the "using raw+scale/offset source channel" or "using
  processed channel" message emitted by `rescale_configure_channel()`.
- If `devm_iio_channel_get()` returns `-EPROBE_DEFER`, the backend ADC has
  not probed yet -- this is normal and the AFE will retry automatically.

---

## 10. Key Conventions

### License

All AFE drivers use GPL-2.0 (or GPL-2.0+):

```c
// SPDX-License-Identifier: GPL-2.0
```

With `MODULE_LICENSE("GPL v2");` at the bottom.

### Platform Driver, Not Bus Driver

AFE drivers are **platform drivers** registered with
`module_platform_driver()`. They have no SPI/I2C bus dependency because they
do not talk to hardware directly -- they only consume another IIO channel.

### Rescale Math -- Overflow Prevention

Always use `gcd()` to reduce numerator/denominator before storing them. The
rescale math in `rescale_process_scale()` multiplies these values with the
backend scale, and overflow is possible with large resistor values:

```c
#include <linux/gcd.h>

factor = gcd(numerator, denominator);
rescale->numerator = numerator / factor;
rescale->denominator = denominator / factor;
```

### Scale Processing

The `rescale_process_scale()` function handles all `IIO_VAL_*` return types
from the backend scale. It multiplies the backend's scale by
`numerator / denominator` while preserving the value type where possible and
falling back to `IIO_VAL_INT_PLUS_NANO` when overflow would occur.

### Offset Processing

The offset computation accounts for the backend's own offset:

```
effective_offset = schan_offset + rescaler_offset / schan_scale
```

This is implemented in `rescale_process_offset()`.

### No Buffer Support

AFE drivers do not implement triggered buffers. They operate in
`INDIO_DIRECT_MODE` only. Each sysfs read triggers a synchronous read of the
backend channel.

### ext_info Passthrough

If the backend channel exposes extended info attributes, the AFE driver
copies and proxies them so they remain accessible through the AFE's sysfs
directory. See the `rescale_read_ext_info()` / `rescale_write_ext_info()`
pattern in `iio-rescale.c`.

### Namespace Symbol Export

Exported symbols use the `IIO_RESCALE` namespace:

```c
EXPORT_SYMBOL_NS_GPL(rescale_process_scale, IIO_RESCALE);
```

Consumers must declare `MODULE_IMPORT_NS(IIO_RESCALE)` to use these symbols.

---

## 11. Commit Format

### Subject Line

```
iio: afe: <devname>: <brief description>
```

### Examples

```
iio: afe: iio-rescale: add support for <new-afe-type>
iio: afe: iio-rescale: fix overflow in current sense scaling
dt-bindings: iio: afe: add <compatible-string>.yaml
```

### New AFE Variant Patch Series (within iio-rescale)

1. `dt-bindings: iio: afe: add <compatible>.yaml` -- DT binding
2. `iio: afe: iio-rescale: add support for <afe-type>` -- Driver change
3. `MAINTAINERS: add entry for <afe-type> binding` -- Maintainer entry (if applicable)

### New Standalone AFE Driver Patch Series

1. `dt-bindings: iio: afe: add <compatible>.yaml` -- DT binding
2. `iio: afe: <devname>: add support for <DEVNAME>` -- Driver source
3. `MAINTAINERS: add entry for <DEVNAME> IIO AFE driver` -- Maintainer entry

### Commit Body

- Wrap at 75 characters.
- Explain the analog front-end circuit and why the rescale parameters are
  computed the way they are.
- Reference the datasheet for the AFE component if applicable.
- Include `Signed-off-by:` (use `git commit -s`).

### Full Example

```
iio: afe: iio-rescale: add support for current-sense-shunt

The current-sense-shunt compatible describes a circuit where a voltage
is measured across a shunt resistor to determine current. The driver
converts the ADC voltage reading to current by dividing by the shunt
resistance (specified in micro-ohms via devicetree).

The rescale factor is computed as 1000000 / shunt_micro_ohms, reduced
by their GCD to prevent overflow in the rescale math.

Signed-off-by: First Last <first.last@analog.com>
```
