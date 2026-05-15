# Linux IIO Amplifier Driver Template

This template covers IIO amplifier/attenuator drivers under `drivers/iio/amplifiers/`.
It is derived from the consolidated IIO driver template and real upstream drivers
(ADA4250, HMC425A, AD8366, ADL5580).

---

## 1. Purpose

IIO amplifier drivers handle programmable gain amplifiers (PGAs), variable gain
amplifiers (VGAs), and digital step attenuators. Representative parts include
ADA4250, HMC425A, AD8366, ADL5240, ADRF5720, and LTC6373.

All amplifier drivers use `IIO_VOLTAGE` channels (typically with `.output = 1`)
and expose gain through the `IIO_CHAN_INFO_HARDWAREGAIN` attribute. Gain values
are expressed in decibels using `IIO_VAL_INT_PLUS_MICRO_DB`, or as linear V/V
multipliers using `IIO_VAL_INT` when the device operates with power-of-two gain
steps (e.g. ADA4250: 1, 2, 4, 8, ... 128).

Two control interfaces exist in practice:

- **GPIO-based** -- Digital attenuators (HMC425A, ADRF5740) set gain via
  parallel GPIO lines. These use `platform_driver` with `ctrl-gpios`.
- **SPI register-based** -- Amplifiers with SPI register maps (ADA4250, AD8366,
  ADL5580) use `spi_driver` and typically `regmap_spi`.

---

## 2. File Checklist

| File                                                                  | Action | Required |
|-----------------------------------------------------------------------|--------|----------|
| `drivers/iio/amplifiers/<devname>.c`                                  | Create | Yes      |
| `drivers/iio/amplifiers/Kconfig`                                      | Modify | Yes      |
| `drivers/iio/amplifiers/Makefile`                                     | Modify | Yes      |
| `Documentation/devicetree/bindings/iio/amplifiers/adi,<devname>.yaml` | Create | Yes      |
| `include/dt-bindings/iio/adi,<devname>.h`                             | Create | Optional |

### Notes

- A `dt-bindings` header is only needed when the binding defines enumerated
  constants shared between the YAML and the driver (e.g. ADL5580 common-mode
  voltage modes).
- GPIO-based attenuators do not have a `reg` property since they are not on
  a bus -- they are matched as platform devices.

---

## 3. Devicetree Binding

Bindings live under `Documentation/devicetree/bindings/iio/amplifiers/adi,<devname>.yaml`.

### SPI Amplifier (ADA4250 pattern)

```yaml
# SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)
%YAML 1.2
---
$id: http://devicetree.org/schemas/iio/amplifiers/adi,ada4250.yaml#
$schema: http://devicetree.org/meta-schemas/core.yaml#

title: Analog Devices ADA4250 Programmable Gain Instrumentation Amplifier

maintainers:
  - First Last <first.last@analog.com>

description: |
  Precision Low Power, 110kHz, 26uA, Programmable Gain Instrumentation
  Amplifier with SPI interface.

properties:
  compatible:
    enum:
      - adi,ada4250

  reg:
    maxItems: 1

  avdd-supply: true

  adi,refbuf-enable:
    description:
      Enable internal buffer to drive the reference pin.
    type: boolean

required:
  - compatible
  - reg
  - avdd-supply

allOf:
  - $ref: /schemas/spi/spi-peripheral-props.yaml#

unevaluatedProperties: false

examples:
  - |
    spi {
        #address-cells = <1>;
        #size-cells = <0>;
        amplifier@0 {
            compatible = "adi,ada4250";
            reg = <0>;
            avdd-supply = <&avdd>;
        };
    };
```

### GPIO Digital Attenuator (HMC425A pattern)

```yaml
# SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)
%YAML 1.2
---
$id: http://devicetree.org/schemas/iio/amplifiers/adi,hmc425a.yaml#
$schema: http://devicetree.org/meta-schemas/core.yaml#

title: Analog Devices HMC425A and similar Digital Step Attenuators

maintainers:
  - First Last <first.last@analog.com>

description: |
  Digital Step Attenuator IIO devices with GPIO interface.
  HMC425A 0.5 dB LSB, 6-BIT DIGITAL POSITIVE CONTROL ATTENUATOR, 2.2 - 8.0 GHz.

properties:
  compatible:
    enum:
      - adi,hmc425a
      - adi,hmc540s
      - adi,adrf5740
      - adi,ltc6373

  vcc-supply: true

  ctrl-gpios:
    description:
      Array of GPIO specifiers connected to the digital control pins.
    minItems: 1
    maxItems: 6

required:
  - compatible
  - ctrl-gpios

additionalProperties: false

examples:
  - |
    #include <dt-bindings/gpio/gpio.h>
    gpio_hmc425a: hmc425a {
        compatible = "adi,hmc425a";
        ctrl-gpios = <&gpio 40 GPIO_ACTIVE_HIGH>,
                     <&gpio 39 GPIO_ACTIVE_HIGH>,
                     <&gpio 38 GPIO_ACTIVE_HIGH>,
                     <&gpio 37 GPIO_ACTIVE_HIGH>,
                     <&gpio 36 GPIO_ACTIVE_HIGH>,
                     <&gpio 35 GPIO_ACTIVE_HIGH>;
        vcc-supply = <&vcc_3v3>;
    };
```

### Key Binding Properties for Amplifiers

| Property            | Bus    | Purpose                                      |
|---------------------|--------|----------------------------------------------|
| `reg`               | SPI    | Chip-select index                            |
| `avdd-supply`       | SPI    | Analog supply regulator                      |
| `vcc-supply`        | GPIO   | Power supply for GPIO-controlled attenuators |
| `ctrl-gpios`        | GPIO   | Gain/attenuation control GPIO array          |
| `adi,refbuf-enable` | SPI    | Device-specific boolean (ADA4250)            |
| `enable-gpios`      | Either | Optional enable/power-down GPIO              |

---

## 4. Kconfig

Add the entry to `drivers/iio/amplifiers/Kconfig` in **alphabetical order**,
inside the `menu "Amplifiers"` block.

### SPI Amplifier with regmap

```kconfig
config ADA4250
	tristate "Analog Devices ADA4250 Instrumentation Amplifier"
	depends on SPI
	select REGMAP_SPI
	help
	  Say yes here to build support for Analog Devices ADA4250
	  SPI Amplifier's support. The driver provides direct access via
	  sysfs.

	  To compile this driver as a module, choose M here: the
	  module will be called ada4250.
```

### SPI Amplifier without regmap (raw SPI writes)

```kconfig
config AD8366
	tristate "Analog Devices AD8366 and similar Gain Amplifiers"
	depends on SPI
	depends on GPIOLIB
	select BITREVERSE
	help
	  Say yes here to build support for Analog Devices AD8366 and similar
	  gain amplifiers. This driver supports the following gain amplifiers
	  from Analog Devices:
	    AD8366 Dual-Digital Variable Gain Amplifier (VGA)
	    ADA4961 BiCMOS RF Digital Gain Amplifier (DGA)
	    ADL5240 Digitally controlled variable gain amplifier (VGA)

	  To compile this driver as a module, choose M here: the
	  module will be called ad8366.
```

### GPIO-Based Attenuator

```kconfig
config HMC425
	tristate "Analog Devices HMC425A and similar GPIO Gain Amplifiers"
	depends on GPIOLIB
	help
	  Say yes here to build support for Analog Devices HMC425A and similar
	  gain amplifiers or step attenuators.

	  To compile this driver as a module, choose M here: the
	  module will be called hmc425a.
```

### Common Dependencies and Selects

| Depends / Select   | When to use                                      |
|--------------------|--------------------------------------------------|
| `depends on SPI`   | SPI-controlled amplifier                         |
| `depends on GPIOLIB` | GPIO-controlled attenuator                     |
| `select REGMAP_SPI` | SPI amplifier using regmap for register access  |
| `select BITREVERSE` | Driver uses `bitrev8()` for SPI data packing    |

---

## 5. Makefile

Add the entry to `drivers/iio/amplifiers/Makefile` in **alphabetical order**.

```makefile
# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile iio/amplifiers
#

# When adding new entries keep the list in alphabetical order
obj-$(CONFIG_AD8366) += ad8366.o
obj-$(CONFIG_ADA4250) += ada4250.o
obj-$(CONFIG_HMC425) += hmc425a.o
```

---

## 6. Driver Source

### Channel Specification

All amplifier drivers use `IIO_VOLTAGE` channels with `.output = 1` and
`IIO_CHAN_INFO_HARDWAREGAIN` in `info_mask_separate`:

```c
#define ADXXXX_AMP_CHAN(_channel) {                              \
        .type = IIO_VOLTAGE,                                    \
        .output = 1,                                            \
        .indexed = 1,                                           \
        .channel = (_channel),                                  \
        .info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN),  \
}

static const struct iio_chan_spec adxxxx_channels[] = {
        ADXXXX_AMP_CHAN(0),
};
```

When `read_avail` is supported (e.g. ADA4250), add the available mask:

```c
static const struct iio_chan_spec ada4250_channels[] = {
        {
                .type = IIO_VOLTAGE,
                .output = 1,
                .indexed = 1,
                .channel = 0,
                .info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
                                     BIT(IIO_CHAN_INFO_OFFSET) |
                                     BIT(IIO_CHAN_INFO_CALIBBIAS) |
                                     BIT(IIO_CHAN_INFO_SCALE),
                .info_mask_separate_available =
                                     BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
                                     BIT(IIO_CHAN_INFO_CALIBBIAS),
        },
};
```

### HARDWAREGAIN read_raw / write_raw (dB convention, SPI)

For devices that report gain in dB (AD8366, HMC425A family):

```c
static int adxxxx_read_raw(struct iio_dev *indio_dev,
                           struct iio_chan_spec const *chan,
                           int *val, int *val2, long m)
{
        struct adxxxx_state *st = iio_priv(indio_dev);
        int code, gain;

        mutex_lock(&st->lock);
        switch (m) {
        case IIO_CHAN_INFO_HARDWAREGAIN:
                code = st->ch[chan->channel];
                gain = st->info->gain_min + st->info->gain_step * code;
                /* gain is in milli-dB internally */
                *val = gain / 1000;
                *val2 = (gain % 1000) * 1000;
                mutex_unlock(&st->lock);
                return IIO_VAL_INT_PLUS_MICRO_DB;
        default:
                mutex_unlock(&st->lock);
                return -EINVAL;
        }
}

static int adxxxx_write_raw(struct iio_dev *indio_dev,
                            struct iio_chan_spec const *chan,
                            int val, int val2, long mask)
{
        struct adxxxx_state *st = iio_priv(indio_dev);
        int gain, code;

        switch (mask) {
        case IIO_CHAN_INFO_HARDWAREGAIN:
                /* Convert dB + micro-dB to milli-dB */
                if (val < 0)
                        gain = (val * 1000) - (val2 / 1000);
                else
                        gain = (val * 1000) + (val2 / 1000);

                if (gain > st->info->gain_max || gain < st->info->gain_min)
                        return -EINVAL;

                code = DIV_ROUND_CLOSEST(gain - st->info->gain_min,
                                         st->info->gain_step);

                mutex_lock(&st->lock);
                st->ch[chan->channel] = code;
                /* Write code to hardware via SPI */
                mutex_unlock(&st->lock);
                return 0;
        default:
                return -EINVAL;
        }
}
```

### HARDWAREGAIN read_raw / write_raw (linear V/V, SPI register)

For devices with power-of-two gain steps (ADA4250):

```c
static int ada4250_read_raw(struct iio_dev *indio_dev,
                            struct iio_chan_spec const *chan,
                            int *val, int *val2, long info)
{
        struct ada4250_state *st = iio_priv(indio_dev);
        int ret;

        switch (info) {
        case IIO_CHAN_INFO_HARDWAREGAIN:
                ret = regmap_read(st->regmap, ADA4250_REG_GAIN_MUX, val);
                if (ret)
                        return ret;
                *val = BIT(*val);       /* Convert code to V/V: 2^code */
                return IIO_VAL_INT;
        default:
                return -EINVAL;
        }
}

static int ada4250_write_raw(struct iio_dev *indio_dev,
                             struct iio_chan_spec const *chan,
                             int val, int val2, long info)
{
        struct ada4250_state *st = iio_priv(indio_dev);

        switch (info) {
        case IIO_CHAN_INFO_HARDWAREGAIN:
                return regmap_write(st->regmap, ADA4250_REG_GAIN_MUX,
                                    FIELD_PREP(ADA4250_GAIN_MUX_MSK,
                                               ilog2(val)));
        default:
                return -EINVAL;
        }
}
```

### write_raw_get_fmt (required for dB-based drivers)

When using `IIO_VAL_INT_PLUS_MICRO_DB`, the driver must implement
`write_raw_get_fmt` so the IIO core knows how to parse userspace writes:

```c
static int adxxxx_write_raw_get_fmt(struct iio_dev *indio_dev,
                                    struct iio_chan_spec const *chan,
                                    long mask)
{
        switch (mask) {
        case IIO_CHAN_INFO_HARDWAREGAIN:
                return IIO_VAL_INT_PLUS_MICRO_DB;
        default:
                return -EINVAL;
        }
}
```

### GPIO-Based Gain Control (HMC425A pattern)

```c
static int hmc425a_write(struct iio_dev *indio_dev, u32 value)
{
        struct hmc425a_state *st = iio_priv(indio_dev);
        DECLARE_BITMAP(values, BITS_PER_TYPE(value));

        values[0] = value;

        gpiod_set_array_value_cansleep(st->gpios->ndescs,
                                       st->gpios->desc,
                                       NULL, values);
        return 0;
}
```

### iio_info Structure

```c
/* dB-based driver (AD8366, HMC425A) */
static const struct iio_info adxxxx_info = {
        .read_raw = &adxxxx_read_raw,
        .write_raw = &adxxxx_write_raw,
        .write_raw_get_fmt = &adxxxx_write_raw_get_fmt,
};

/* Linear V/V driver with read_avail (ADA4250) */
static const struct iio_info adxxxx_info = {
        .read_raw = adxxxx_read_raw,
        .write_raw = adxxxx_write_raw,
        .read_avail = &adxxxx_read_avail,
        .debugfs_reg_access = &adxxxx_reg_access,
};
```

### Probe (SPI amplifier)

```c
static int adxxxx_probe(struct spi_device *spi)
{
        struct device *dev = &spi->dev;
        struct iio_dev *indio_dev;
        struct adxxxx_state *st;
        int ret;

        indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
        if (!indio_dev)
                return -ENOMEM;

        st = iio_priv(indio_dev);
        st->spi = spi;
        st->info = spi_get_device_match_data(spi);

        ret = devm_mutex_init(dev, &st->lock);
        if (ret)
                return ret;

        ret = devm_regulator_get_enable(dev, "vcc");
        if (ret)
                return dev_err_probe(dev, ret, "Failed to get regulator\n");

        indio_dev->name = st->info->name;
        indio_dev->info = &adxxxx_info;
        indio_dev->modes = INDIO_DIRECT_MODE;
        indio_dev->channels = adxxxx_channels;
        indio_dev->num_channels = ARRAY_SIZE(adxxxx_channels);

        /* Hardware init: reset, verify chip ID, set default gain */

        return devm_iio_device_register(dev, indio_dev);
}
```

### Probe (GPIO attenuator)

```c
static int hmc425a_probe(struct platform_device *pdev)
{
        struct device *dev = &pdev->dev;
        struct iio_dev *indio_dev;
        struct hmc425a_state *st;
        int ret;

        indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
        if (!indio_dev)
                return -ENOMEM;

        st = iio_priv(indio_dev);
        st->chip_info = device_get_match_data(dev);
        st->gain = st->chip_info->default_gain;

        st->gpios = devm_gpiod_get_array(dev, "ctrl", GPIOD_OUT_LOW);
        if (IS_ERR(st->gpios))
                return dev_err_probe(dev, PTR_ERR(st->gpios),
                                     "failed to get gpios\n");

        if (st->gpios->ndescs != st->chip_info->num_gpios)
                return dev_err_probe(dev, -ENODEV,
                                     "%d GPIOs needed to operate\n",
                                     st->chip_info->num_gpios);

        ret = devm_regulator_get_enable(dev, "vcc-supply");
        if (ret)
                return ret;

        mutex_init(&st->lock);

        indio_dev->name = st->chip_info->name;
        indio_dev->info = &hmc425a_info;
        indio_dev->modes = INDIO_DIRECT_MODE;
        indio_dev->channels = st->chip_info->channels;
        indio_dev->num_channels = st->chip_info->num_channels;

        /* Set default gain via GPIO */
        hmc425a_write(indio_dev, st->gain);

        return devm_iio_device_register(dev, indio_dev);
}
```

---

## 7. Gain Table

### Available Gain Values via IIO_AVAIL_LIST

When a device supports discrete gain steps, expose them through `read_avail`
and set `info_mask_separate_available` in the channel spec.

```c
/* Linear V/V gain table (ADA4250) */
static const int hwgain_table[] = {1, 2, 4, 8, 16, 32, 64, 128};

static int adxxxx_read_avail(struct iio_dev *indio_dev,
                             struct iio_chan_spec const *chan,
                             const int **vals, int *type, int *length,
                             long mask)
{
        switch (mask) {
        case IIO_CHAN_INFO_HARDWAREGAIN:
                *vals = hwgain_table;
                *type = IIO_VAL_INT;
                *length = ARRAY_SIZE(hwgain_table);
                return IIO_AVAIL_LIST;
        default:
                return -EINVAL;
        }
}
```

### dB-Based Gain Table

For dB-based devices, gain is typically computed from chip_info parameters
rather than an explicit table. The pattern uses `gain_min`, `gain_max`, and
`gain_step` fields in a per-device info structure:

```c
struct adxxxx_chip_info {
        const char      *name;
        int             gain_min;       /* Minimum gain in milli-dB */
        int             gain_max;       /* Maximum gain in milli-dB */
        int             gain_step;      /* Step size in milli-dB */
        unsigned int    num_gpios;      /* For GPIO-based devices */
};

/* Examples from upstream drivers: */
/* AD8366:    gain_min =   4500, gain_max =  20500, gain_step =  253  */
/* HMC425A:   gain_min = -31500, gain_max =      0, gain_step =  500  */
/* ADRF5720:  gain_min = -31500, gain_max =      0, gain_step =  500  */
/* HMC1119:   gain_min = -31750, gain_max =      0, gain_step =  250  */
```

### Gain Code Conversion Functions

Amplifier drivers typically provide two conversion functions per device variant:

```c
/* Convert dB gain (milli-dB) to hardware code */
int (*gain_dB_to_code)(int gain, int *code);

/* Convert hardware code to dB gain */
int (*code_to_gain_dB)(int code, int *val, int *val2);
```

The `code_to_gain_dB` function populates `val` (integer part) and `val2`
(micro part) for `IIO_VAL_INT_PLUS_MICRO_DB` format.

---

## 8. Devicetree Parsing

### GPIO Pins for Digital Attenuators

```c
/* Request control GPIOs as an array */
st->gpios = devm_gpiod_get_array(dev, "ctrl", GPIOD_OUT_LOW);
if (IS_ERR(st->gpios))
        return dev_err_probe(dev, PTR_ERR(st->gpios),
                             "failed to get gpios\n");

/* Validate expected GPIO count */
if (st->gpios->ndescs != st->chip_info->num_gpios)
        return dev_err_probe(dev, -ENODEV,
                             "%d GPIOs needed to operate\n",
                             st->chip_info->num_gpios);
```

DT node uses `ctrl-gpios`:
```dts
ctrl-gpios = <&gpio 40 GPIO_ACTIVE_HIGH>,
             <&gpio 39 GPIO_ACTIVE_HIGH>,
             <&gpio 38 GPIO_ACTIVE_HIGH>;
```

### Optional Enable GPIO (SPI amplifiers)

```c
enable_gpio = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_HIGH);
if (IS_ERR(enable_gpio))
        return dev_err_probe(dev, PTR_ERR(enable_gpio),
                             "Failed to get enable GPIO\n");
```

### Default Gain

Default gain is typically set from `chip_info` constants rather than parsed
from DT. The gain is applied during `probe()`:

```c
st->gain = st->chip_info->default_gain;

/* Write default gain to hardware */
hmc425a_write(indio_dev, st->gain);
```

### Device-Specific Boolean Properties

```c
/* ADA4250: Reference buffer enable */
st->refbuf_en = device_property_read_bool(dev, "adi,refbuf-enable");
```

### Supply Regulators

```c
/* Single supply (GPIO attenuator) */
ret = devm_regulator_get_enable(dev, "vcc-supply");

/* Single supply (SPI amplifier) */
ret = devm_regulator_get_enable(dev, "avdd");

/* Multiple optional supplies (ADL5580) */
static const char * const regulator_names[] = {"avdd", "avcc"};
for (i = 0; i < ARRAY_SIZE(regulator_names); i++) {
        ret = devm_regulator_get_enable_optional(dev, regulator_names[i]);
        if (ret < 0 && ret != -ENODEV)
                return dev_err_probe(dev, ret,
                                     "error enabling regulator %s\n",
                                     regulator_names[i]);
}
```

---

## 9. Test & Debug

### sysfs Interface

Amplifier channels appear as output voltage channels:

```
/sys/bus/iio/devices/iio:device0/
    name                                    # e.g. "hmc425a", "ada4250"
    out_voltage0_hardwaregain               # Current gain (dB or V/V)
    out_voltage0_hardwaregain_available     # Discrete gain values (if supported)
```

### Reading and Writing Gain

```sh
# Read current gain
cat /sys/bus/iio/devices/iio:device0/out_voltage0_hardwaregain

# Write gain in dB (for dB-based drivers like HMC425A)
# Value format: <integer> <micro> dB
echo "-6 0" > /sys/bus/iio/devices/iio:device0/out_voltage0_hardwaregain

# Write gain in V/V (for linear drivers like ADA4250)
echo "8" > /sys/bus/iio/devices/iio:device0/out_voltage0_hardwaregain

# Read available gain values
cat /sys/bus/iio/devices/iio:device0/out_voltage0_hardwaregain_available
# Example output for ADA4250: "1 2 4 8 16 32 64 128"
```

### debugfs Register Access

For SPI-based amplifiers that implement `debugfs_reg_access`:

```sh
# Read register 0x00
echo 0x00 > /sys/kernel/debug/iio/iio:device0/direct_reg_access
cat /sys/kernel/debug/iio/iio:device0/direct_reg_access

# Write 0x03 to register 0x00
echo 0x00 0x03 > /sys/kernel/debug/iio/iio:device0/direct_reg_access
```

### IIO Userspace Tools

```sh
# List devices and channels
iio_info

# Read a specific attribute
iio_attr -d iio:device0 -c out_voltage0_hardwaregain
```

---

## 10. Key Conventions

### License

All new amplifier drivers must use GPL-2.0 or GPL-2.0+:

```c
// SPDX-License-Identifier: GPL-2.0+
```

`MODULE_LICENSE("GPL")` at the bottom must match.

### dB Gain Convention

- Gain is stored internally in **milli-dB** (integer, e.g. -31500 for -31.5 dB).
- Gain is reported to userspace via `IIO_VAL_INT_PLUS_MICRO_DB`:
  - `*val` = integer part in dB (e.g. -31)
  - `*val2` = fractional part in micro-dB (e.g. 500000 for 0.5 dB)
- Negative dB values (attenuation) require care when splitting integer and
  fractional parts:
  ```c
  if (val < 0)
          gain = (val * 1000) - (val2 / 1000);
  else
          gain = (val * 1000) + (val2 / 1000);
  ```

### IIO_VAL_INT_PLUS_MICRO_DB

This return value from `read_raw` causes the IIO core to append ` dB` to the
sysfs output. The driver must also implement `write_raw_get_fmt` returning
`IIO_VAL_INT_PLUS_MICRO_DB` so the core correctly parses writes.

### Linear Gain (V/V)

For devices with power-of-two gain steps (ADA4250), gain is returned as
`IIO_VAL_INT` representing the V/V multiplier. No `write_raw_get_fmt` is
needed for `IIO_VAL_INT`.

### Coding Style

- Use `FIELD_GET()` / `FIELD_PREP()` with `GENMASK()` for register bitfields.
- Use `BIT()` for single-bit definitions.
- Use `dev_err_probe()` for all error returns in `probe()`.
- Use `devm_mutex_init()` (preferred) or `mutex_init()` for lock initialization.
- Use `guard(mutex)(&st->lock)` (scoped lock) where possible.
- GPIO-based drivers use `platform_driver` / `module_platform_driver()`.
- SPI-based drivers use `spi_driver` / `module_spi_driver()`.
- Include headers in alphabetical order within each group.

### Match Table Patterns

For drivers supporting multiple device variants, use a `chip_info` table
indexed by an enum and referenced from `of_device_id.data`:

```c
enum adxxxx_type {
        ID_ADXXXX_A,
        ID_ADXXXX_B,
};

static const struct adxxxx_chip_info adxxxx_chip_info_tbl[] = {
        [ID_ADXXXX_A] = { .name = "adxxxx_a", .gain_min = -31500, ... },
        [ID_ADXXXX_B] = { .name = "adxxxx_b", .gain_min = -15000, ... },
};

static const struct of_device_id adxxxx_of_match[] = {
        { .compatible = "adi,adxxxx-a",
          .data = &adxxxx_chip_info_tbl[ID_ADXXXX_A] },
        { .compatible = "adi,adxxxx-b",
          .data = &adxxxx_chip_info_tbl[ID_ADXXXX_B] },
        { }
};
```

---

## 11. Commit Message Format

### Subject Line

```
iio: amplifiers: <devname>: <brief description>
```

Examples:
```
iio: amplifiers: ada4250: add support for ADA4250
iio: amplifiers: hmc425a: add support for LTC6373
iio: amplifiers: ad8366: fix gain calculation for negative dB values
iio: amplifiers: adl5580: add devicetree binding support
```

### Patch Series for a New Amplifier Driver

1. `dt-bindings: iio: amplifiers: add adi,<devname>.yaml` -- DT binding
2. `iio: amplifiers: <devname>: add support for <DEVNAME>` -- Driver source
3. `MAINTAINERS: add entry for <DEVNAME> IIO driver` -- Maintainer entry

### Full Example

```
iio: amplifiers: ada4250: add support for ADA4250

The ADA4250 is a precision, low power, programmable gain
instrumentation amplifier with SPI interface. It supports
gain settings of 1, 2, 4, 8, 16, 32, 64, and 128 V/V,
along with sensor offset calibration.

This driver supports:
  - Programmable gain via IIO_CHAN_INFO_HARDWAREGAIN
  - Sensor offset calibration
  - Reference buffer enable via devicetree property

Signed-off-by: First Last <first.last@analog.com>
```
