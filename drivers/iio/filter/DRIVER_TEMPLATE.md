# Linux IIO Filter Driver Template

Template for IIO filter subsystem drivers (`drivers/iio/filter/`).
Reference device: ADMV8818 (digitally tunable high-pass and low-pass filter).

---

## 1. Purpose

IIO filter drivers expose tunable or programmable analog filters -- devices
that provide selectable high-pass, low-pass, or bandpass frequency responses.
The canonical example is the ADMV8818, a 2 GHz to 18 GHz digitally tunable
high-pass and low-pass filter.

### Channel Types

| Channel Type      | Usage                                                |
|-------------------|------------------------------------------------------|
| `IIO_VOLTAGE`     | When the filter operates on baseband voltage signals  |
| `IIO_ALTVOLTAGE`  | When the filter operates on RF/carrier-frequency signals (preferred for RF filters) |

### Key `info_mask` Bits

| Info Mask Bit                                    | Purpose                                      |
|--------------------------------------------------|----------------------------------------------|
| `IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY`    | Low-pass filter 3 dB cutoff frequency (Hz)   |
| `IIO_CHAN_INFO_HIGH_PASS_FILTER_3DB_FREQUENCY`   | High-pass filter 3 dB cutoff frequency (Hz)  |

A single channel typically exposes both LPF and HPF cutoff frequencies when the
device supports bandpass operation (independent HPF + LPF). A second channel
with `ext_info` is used for mode selection (auto/manual/bypass) and bandwidth
or center frequency reporting.

---

## 2. File Checklist

| File                                                              | Action | Required |
|-------------------------------------------------------------------|--------|----------|
| `drivers/iio/filter/<devname>.c`                                  | Create | Yes      |
| `drivers/iio/filter/Kconfig`                                      | Modify | Yes      |
| `drivers/iio/filter/Makefile`                                     | Modify | Yes      |
| `Documentation/devicetree/bindings/iio/filter/adi,<devname>.yaml` | Create | Yes      |
| `MAINTAINERS`                                                     | Modify | Yes      |

---

## 3. DT Binding

Bindings live under `Documentation/devicetree/bindings/iio/filter/adi,<devname>.yaml`.

Filter devices are typically SPI-connected and may accept a reference clock
input representing the RF input frequency for automatic filter band tracking.

```yaml
# SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)
%YAML 1.2
---
$id: http://devicetree.org/schemas/iio/filter/adi,admv8818.yaml#
$schema: http://devicetree.org/meta-schemas/core.yaml#

title: ADMV8818 Digitally Tunable, High-Pass and Low-Pass Filter

maintainers:
  - First Last <first.last@analog.com>

description: |
  Fully monolithic microwave integrated circuit (MMIC) that features a
  digitally selectable frequency of operation. The device features four
  independently controlled high-pass filters (HPFs) and four independently
  controlled low-pass filters (LPFs) that span the 2 GHz to 18 GHz
  frequency range.

  https://www.analog.com/en/products/admv8818.html

properties:
  compatible:
    enum:
      - adi,admv8818

  reg:
    maxItems: 1

  spi-max-frequency:
    maximum: 10000000

  clocks:
    description:
      Definition of the external clock (RF input reference).
    minItems: 1

  clock-names:
    items:
      - const: rf_in

  clock-output-names:
    maxItems: 1

  '#clock-cells':
    const: 0

required:
  - compatible
  - reg

additionalProperties: false

examples:
  - |
    spi {
      #address-cells = <1>;
      #size-cells = <0>;
      admv8818@0 {
        compatible = "adi,admv8818";
        reg = <0>;
        spi-max-frequency = <10000000>;
        clocks = <&admv8818_rfin>;
        clock-names = "rf_in";
      };
    };
...
```

### Key Binding Notes

- The `clocks` / `clock-names` properties are optional. When present, the
  driver can operate in **auto** mode, tracking the RF input frequency via
  clock notifiers and automatically adjusting filter bands.
- When clocks are omitted, the driver defaults to **manual** mode where
  filter cutoff frequencies are set explicitly through sysfs.

---

## 4. Kconfig

Add the entry to `drivers/iio/filter/Kconfig` in **alphabetical order**.

```kconfig
config ADMV8818
	tristate "Analog Devices ADMV8818 High-Pass and Low-Pass Filter"
	depends on SPI && COMMON_CLK
	select REGMAP_SPI
	help
	  Say yes here to build support for Analog Devices ADMV8818
	  2 GHz to 18 GHz, Digitally Tunable, High-Pass and Low-Pass Filter.

	  To compile this driver as a module, choose M here: the
	  module will be called admv8818.
```

### Notes

- `COMMON_CLK` dependency is needed when the driver uses the clock framework
  for automatic filter tracking.
- `REGMAP_SPI` is selected when the driver uses regmap for register access.

---

## 5. Makefile

Add the entry to `drivers/iio/filter/Makefile` in **alphabetical order**.

```makefile
# SPDX-License-Identifier: GPL-2.0
#
# Makefile for industrial I/O Filter drivers
#

# When adding new entries keep the list in alphabetical order
obj-$(CONFIG_ADMV8818) += admv8818.o
```

---

## 6. Driver Source

### Includes

```c
// SPDX-License-Identifier: GPL-2.0-only
/*
 * ADMV8818 driver
 *
 * Copyright 2021 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>
#include <linux/units.h>
```

### Register Definitions

Use `GENMASK()` for multi-bit fields and `BIT()` for single-bit fields.
Filter registers typically encode band selection (which physical filter path)
and step selection (where within a band the cutoff is set):

```c
/* Filter register fields */
#define ADMV8818_HPF_WR0_MSK    GENMASK(7, 4)   /* HPF step within band */
#define ADMV8818_LPF_WR0_MSK    GENMASK(3, 0)   /* LPF step within band */

/* Switch register fields */
#define ADMV8818_SW_IN_WR0_MSK  GENMASK(5, 3)   /* HPF band select */
#define ADMV8818_SW_OUT_WR0_MSK GENMASK(2, 0)   /* LPF band select */
```

### Channel Specification

Filter channels use `IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY` and
`IIO_CHAN_INFO_HIGH_PASS_FILTER_3DB_FREQUENCY` to expose cutoff frequencies.
A typical filter has two channels: one for filter frequency control, one for
mode/bandwidth `ext_info`:

```c
#define ADMV8818_CHAN(_channel) {                                \
    .type = IIO_ALTVOLTAGE,                                     \
    .output = 1,                                                \
    .indexed = 1,                                               \
    .channel = _channel,                                        \
    .info_mask_separate =                                       \
        BIT(IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY) |      \
        BIT(IIO_CHAN_INFO_HIGH_PASS_FILTER_3DB_FREQUENCY),      \
}

#define ADMV8818_CHAN_BW_CF(_channel, _ext_info) {               \
    .type = IIO_ALTVOLTAGE,                                     \
    .output = 1,                                                \
    .indexed = 1,                                               \
    .channel = _channel,                                        \
    .ext_info = _ext_info,                                      \
}

static const struct iio_chan_spec admv8818_channels[] = {
    ADMV8818_CHAN(0),
    ADMV8818_CHAN_BW_CF(0, admv8818_ext_info),
};
```

### read_raw / write_raw -- Filter Frequency Attributes

The `read_raw` and `write_raw` callbacks handle the 3 dB cutoff frequency
attributes. Filter frequencies are in Hz. For frequencies exceeding 32-bit
range, use `IIO_VAL_INT_64` and pack as `val = (u32)freq`,
`val2 = (u32)(freq >> 32)`:

```c
static int admv8818_read_raw(struct iio_dev *indio_dev,
                             struct iio_chan_spec const *chan,
                             int *val, int *val2, long info)
{
    struct admv8818_state *st = iio_priv(indio_dev);
    int ret;
    u64 freq;

    switch (info) {
    case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
        ret = admv8818_read_lpf_freq(st, &freq);
        if (ret)
            return ret;

        *val = (u32)freq;
        *val2 = (u32)(freq >> 32);
        return IIO_VAL_INT_64;

    case IIO_CHAN_INFO_HIGH_PASS_FILTER_3DB_FREQUENCY:
        ret = admv8818_read_hpf_freq(st, &freq);
        if (ret)
            return ret;

        *val = (u32)freq;
        *val2 = (u32)(freq >> 32);
        return IIO_VAL_INT_64;

    default:
        return -EINVAL;
    }
}

static int admv8818_write_raw(struct iio_dev *indio_dev,
                              struct iio_chan_spec const *chan,
                              int val, int val2, long info)
{
    struct admv8818_state *st = iio_priv(indio_dev);
    u64 freq = ((u64)val2 << 32 | (u32)val);

    switch (info) {
    case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
        return admv8818_lpf_select(st, freq);
    case IIO_CHAN_INFO_HIGH_PASS_FILTER_3DB_FREQUENCY:
        return admv8818_hpf_select(st, freq);
    default:
        return -EINVAL;
    }
}
```

### Frequency Range Tables and Band Selection

Filter drivers typically store frequency ranges per band and compute the
closest discrete step within a band:

```c
static const unsigned long long freq_range_hpf[4][2] = {
    {1750000000ULL, 3550000000ULL},
    {3400000000ULL, 7250000000ULL},
    {6600000000ULL, 12000000000ULL},
    {12500000000ULL, 19900000000ULL},
};

static int __admv8818_hpf_select(struct admv8818_state *st, u64 freq)
{
    unsigned int hpf_step = 0, hpf_band = 0, i, j;
    u64 freq_step;

    for (i = 0; i < 4; i++) {
        freq_step = div_u64(freq_range_hpf[i][1] -
                            freq_range_hpf[i][0], 15);

        if (freq > freq_range_hpf[i][0] &&
            freq < freq_range_hpf[i][1] + freq_step) {
            hpf_band = i + 1;

            for (j = 1; j <= 16; j++) {
                if (freq < freq_range_hpf[i][0] + freq_step * j) {
                    hpf_step = j - 1;
                    break;
                }
            }
            break;
        }
    }

    /* Write band + step to hardware registers */
    /* ... */
}
```

### read_avail for Available Values

When filter cutoff frequencies have discrete available settings, implement
`read_avail` in `iio_info` or use `info_mask_separate_available` /
`info_mask_shared_by_type_available`:

```c
static int admv8818_read_avail(struct iio_dev *indio_dev,
                               struct iio_chan_spec const *chan,
                               const int **vals, int *type,
                               int *length, long info)
{
    switch (info) {
    case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
        *vals = lpf_avail_freqs;
        *type = IIO_VAL_INT;
        *length = ARRAY_SIZE(lpf_avail_freqs);
        return IIO_AVAIL_LIST;
    case IIO_CHAN_INFO_HIGH_PASS_FILTER_3DB_FREQUENCY:
        *vals = hpf_avail_freqs;
        *type = IIO_VAL_INT;
        *length = ARRAY_SIZE(hpf_avail_freqs);
        return IIO_AVAIL_LIST;
    default:
        return -EINVAL;
    }
}

static const struct iio_info admv8818_info = {
    .read_raw = admv8818_read_raw,
    .write_raw = admv8818_write_raw,
    .read_avail = admv8818_read_avail,
    .debugfs_reg_access = admv8818_reg_access,
};
```

---

## 7. Filter Configuration

### Mode Selection via ext_info

Filter drivers expose an operating mode through `ext_info` on a channel.
Common modes:

| Mode     | Behavior                                                        |
|----------|-----------------------------------------------------------------|
| `auto`   | Filter tracks RF input clock; cutoffs adjusted automatically via clock notifiers |
| `manual` | Cutoff frequencies set explicitly through sysfs                  |
| `bypass` | All filter paths disabled; signal passes through unfiltered      |

Implementation uses `iio_enum` and `iio_chan_spec_ext_info`:

```c
static const char * const admv8818_modes[] = {
    [ADMV8818_AUTO_MODE]   = "auto",
    [ADMV8818_MANUAL_MODE] = "manual",
    [ADMV8818_BYPASS_MODE] = "bypass",
};

static const struct iio_enum admv8818_mode_enum = {
    .items = admv8818_modes,
    .num_items = ARRAY_SIZE(admv8818_modes),
    .get = admv8818_get_mode,
    .set = admv8818_set_mode,
};

static const struct iio_chan_spec_ext_info admv8818_ext_info[] = {
    IIO_ENUM("filter_mode", IIO_SHARED_BY_ALL, &admv8818_mode_enum),
    IIO_ENUM_AVAILABLE("filter_mode", IIO_SHARED_BY_ALL,
                       &admv8818_mode_enum),
    { },
};
```

### Mode Get/Set Callbacks

The `set` callback manages transitions between modes, enabling/disabling the
clock notifier for auto mode and clearing filter settings for bypass mode:

```c
static int admv8818_get_mode(struct iio_dev *indio_dev,
                             const struct iio_chan_spec *chan)
{
    struct admv8818_state *st = iio_priv(indio_dev);

    return st->filter_mode;
}

static int admv8818_set_mode(struct iio_dev *indio_dev,
                             const struct iio_chan_spec *chan,
                             unsigned int mode)
{
    struct admv8818_state *st = iio_priv(indio_dev);

    switch (mode) {
    case ADMV8818_AUTO_MODE:
        /* Enable clock and register notifier for freq tracking */
        /* ... */
        break;
    case ADMV8818_MANUAL_MODE:
        /* Disable auto-tracking, allow manual HPF/LPF writes */
        /* ... */
        break;
    case ADMV8818_BYPASS_MODE:
        /* Set all band/step selects to 0 (bypass) */
        return admv8818_filter_bypass(st);
    default:
        return -EINVAL;
    }

    st->filter_mode = mode;
    return 0;
}
```

### Bandpass / Lowpass / Highpass Configuration

A device like ADMV8818 achieves different filter topologies by combining
independent HPF and LPF paths:

- **Bandpass**: Both HPF and LPF active with HPF cutoff < LPF cutoff.
- **High-pass only**: HPF active, LPF bypassed (band = 0).
- **Low-pass only**: LPF active, HPF bypassed (band = 0).
- **Bypass**: Both HPF and LPF bands set to 0.

The driver selects band and step independently for each filter path. There is
no explicit "filter order" register; the order is fixed by hardware topology
(typically first-order per stage, cascaded).

---

## 8. DT Parsing

### Clock Reference (Auto Mode)

The clock input represents the RF signal frequency. When present, the driver
registers a clock notifier to automatically adjust filter bands on frequency
changes:

```c
static int admv8818_clk_setup(struct admv8818_state *st)
{
    struct spi_device *spi = st->spi;
    int ret;

    st->clkin = devm_clk_get_optional(&spi->dev, "rf_in");
    if (IS_ERR(st->clkin))
        return dev_err_probe(&spi->dev, PTR_ERR(st->clkin),
                             "failed to get the input clock\n");
    if (!st->clkin)
        return 0;

    ret = clk_prepare_enable(st->clkin);
    if (ret)
        return ret;

    ret = devm_add_action_or_reset(&spi->dev, admv8818_clk_disable, st);
    if (ret)
        return ret;

    st->nb.notifier_call = admv8818_freq_change;
    ret = clk_notifier_register(st->clkin, &st->nb);
    if (ret < 0)
        return ret;

    return devm_add_action_or_reset(&spi->dev,
                                    admv8818_clk_notifier_unreg, st);
}
```

### Clock Notifier Callback

```c
static int admv8818_freq_change(struct notifier_block *nb,
                                unsigned long action, void *data)
{
    struct admv8818_state *st = container_of(nb, struct admv8818_state, nb);

    if (action == POST_RATE_CHANGE)
        return notifier_from_errno(admv8818_rfin_band_select(st));

    return NOTIFY_OK;
}
```

### Default Filter Settings

When no clock is provided, the driver starts in manual mode. The `probe()`
function performs a soft reset, verifies the chip ID, and optionally runs
initial band selection if a clock is present:

```c
static int admv8818_init(struct admv8818_state *st)
{
    unsigned int chip_id;
    int ret;

    /* Soft reset */
    ret = regmap_write(st->regmap, ADMV8818_REG_SPI_CONFIG_A,
                       ADMV8818_SOFTRESET_N_MSK | ADMV8818_SOFTRESET_MSK);
    if (ret)
        return ret;

    /* Enable SDO */
    ret = regmap_write(st->regmap, ADMV8818_REG_SPI_CONFIG_A,
                       ADMV8818_SDOACTIVE_N_MSK | ADMV8818_SDOACTIVE_MSK);
    if (ret)
        return ret;

    /* Verify chip ID */
    ret = regmap_read(st->regmap, ADMV8818_REG_CHIPTYPE, &chip_id);
    if (ret)
        return ret;

    if (chip_id != 0x1)
        return -EINVAL;

    /* If clock present, auto-select bands based on RF frequency */
    if (st->clkin)
        return admv8818_rfin_band_select(st);

    return 0;
}
```

---

## 9. Test & Debug

### sysfs Interface

Filter drivers expose the following sysfs attributes:

```
/sys/bus/iio/devices/iio:deviceX/
    name                                                 # "admv8818"
    out_altvoltage0_filter_low_pass_3db_frequency        # LPF 3 dB cutoff (Hz)
    out_altvoltage0_filter_high_pass_3db_frequency       # HPF 3 dB cutoff (Hz)
    out_altvoltage0_filter_mode                          # auto / manual / bypass
    out_altvoltage0_filter_mode_available                # "auto manual bypass"
```

### Reading and Writing Cutoff Frequencies

```sh
# Read current low-pass 3 dB frequency
cat /sys/bus/iio/devices/iio:device0/out_altvoltage0_filter_low_pass_3db_frequency

# Set low-pass 3 dB frequency to 8 GHz
echo 8000000000 > /sys/bus/iio/devices/iio:device0/out_altvoltage0_filter_low_pass_3db_frequency

# Read current high-pass 3 dB frequency
cat /sys/bus/iio/devices/iio:device0/out_altvoltage0_filter_high_pass_3db_frequency

# Set high-pass 3 dB frequency to 3 GHz
echo 3000000000 > /sys/bus/iio/devices/iio:device0/out_altvoltage0_filter_high_pass_3db_frequency
```

### Mode Selection

```sh
# Check available modes
cat /sys/bus/iio/devices/iio:device0/out_altvoltage0_filter_mode_available

# Switch to manual mode
echo manual > /sys/bus/iio/devices/iio:device0/out_altvoltage0_filter_mode

# Switch to bypass (all filters disabled)
echo bypass > /sys/bus/iio/devices/iio:device0/out_altvoltage0_filter_mode
```

### debugfs Register Access

```sh
# Read register 0x20 (WR0_SW -- band selection)
echo 0x20 > /sys/kernel/debug/iio/iio:device0/direct_reg_access
cat /sys/kernel/debug/iio/iio:device0/direct_reg_access

# Read register 0x21 (WR0_FILTER -- HPF/LPF step)
echo 0x21 > /sys/kernel/debug/iio/iio:device0/direct_reg_access
cat /sys/kernel/debug/iio/iio:device0/direct_reg_access
```

### IIO Userspace Tools

```sh
# List device and channel attributes
iio_info
iio_attr -d iio:device0 -c

# Read a specific attribute
iio_attr -d iio:device0 -c out_altvoltage0_filter_low_pass_3db_frequency
```

---

## 10. Key Conventions

### License

All new IIO filter drivers must use GPL-2.0:

```c
// SPDX-License-Identifier: GPL-2.0-only
```

The SPDX tag goes on the very first line. `MODULE_LICENSE("GPL v2")` at the
bottom must match.

### Units

- All filter cutoff frequencies are in **Hz**.
- For RF filters operating in the GHz range, the values will be large integers
  (e.g., 8000000000 for 8 GHz). Use `u64` types internally and
  `IIO_VAL_INT_64` return from `read_raw`.
- Use `HZ_PER_MHZ` and `HZ_PER_GHZ` from `<linux/units.h>` for clarity in
  frequency comparisons.

### Memory Management

- Use `devm_*` allocations exclusively.
- Use `devm_regmap_init_spi()` for SPI register access.
- Use `devm_add_action_or_reset()` for custom cleanup (clock disable,
  notifier unregister).

### Locking

- Protect device state and register access with `struct mutex`.
- Separate locked (`admv8818_hpf_select`) and unlocked
  (`__admv8818_hpf_select`) variants when multiple operations must be
  performed atomically (e.g., setting both HPF and LPF in auto mode).

### Coding Style

- Follow kernel coding style.
- Use `FIELD_GET()` / `FIELD_PREP()` with `GENMASK()` for register bitfields.
- Use `div_u64()` for 64-bit division (not the `/` operator).
- Include headers in alphabetical order.

---

## 11. Commit Format

### Subject Line Prefix

```
iio: filter: <devname>: <brief description>
```

### Examples

```
iio: filter: admv8818: add support for ADMV8818 tunable filter
iio: filter: admv8818: fix HPF band selection for 12 GHz gap
iio: filter: admv8818: add bypass mode support
```

### Patch Series for a New Filter Driver

1. `dt-bindings: iio: filter: add adi,<devname>.yaml`
2. `iio: filter: <devname>: add support for <DEVNAME>`
3. `MAINTAINERS: add entry for <DEVNAME> IIO driver`

### Full Commit Example

```
iio: filter: admv8818: add support for ADMV8818 tunable filter

The ADMV8818 is a fully monolithic microwave integrated circuit (MMIC)
featuring a digitally selectable frequency of operation. It provides
four independently controlled high-pass filters and four independently
controlled low-pass filters spanning 2 GHz to 18 GHz.

This driver supports:
  - Independent HPF and LPF 3 dB cutoff frequency control
  - Auto mode with clock notifier for RF frequency tracking
  - Manual mode for explicit cutoff frequency configuration
  - Bypass mode to disable all filtering

Signed-off-by: First Last <first.last@analog.com>
```
