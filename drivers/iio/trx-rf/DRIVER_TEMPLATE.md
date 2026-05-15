# Linux IIO RF Transceiver Driver Template

Reference drivers: `drivers/iio/trx-rf/adrv903x/`, `drivers/iio/frequency/`

This template covers all files needed to write a Linux kernel IIO driver for
an RF transceiver (AD9361, ADF7023, ADRV9009, ADRV903X). RF transceivers use
`IIO_ALTVOLTAGE` channels for local oscillator (LO) frequency control and
`IIO_VOLTAGE` channels for TX/RX I/Q data paths. `HARDWAREGAIN` controls TX
output power and RX gain, while `SAMP_FREQ` exposes the I/Q data rate.

---

## 1. Purpose -- IIO TRX-RF Subsystem Mapping

RF transceivers integrate complete transmit and receive radio chains. In the
IIO model they expose:

| Function         | IIO Channel Type   | Direction       | Key `info_mask` / `ext_info`                |
|------------------|--------------------|-----------------|---------------------------------------------|
| LO frequency     | `IIO_ALTVOLTAGE`   | `.output = 1`   | `ext_info` "frequency" (u64 Hz)             |
| TX I/Q data path | `IIO_VOLTAGE`      | `.output = 1`   | HARDWAREGAIN, ENABLE; shared SAMP_FREQ      |
| RX I/Q data path | `IIO_VOLTAGE`      | `.output = 0`   | HARDWAREGAIN, ENABLE; shared SAMP_FREQ      |
| Observation RX   | `IIO_VOLTAGE`      | `.output = 0`   | HARDWAREGAIN, ENABLE; shared SAMP_FREQ      |
| Die temperature  | `IIO_TEMP`         | `.output = 0`   | PROCESSED (millidegrees Celsius)            |

**IIO subdirectory:** `drivers/iio/trx-rf/` (preferred) or
`drivers/iio/frequency/` (for simpler transceivers).

**Typical parts:** AD9361, AD9371, ADRV9009, ADRV903X, ADF7023.

### Why IIO_ALTVOLTAGE for the LO

The LO frequency often exceeds 2^32 Hz, so `IIO_CHAN_INFO_FREQUENCY` (which
uses `int *val`) cannot represent the full range. The established pattern is to
use `IIO_ALTVOLTAGE` channels with `ext_info` callbacks that read and write the
frequency as a `u64` string via sysfs.

---

## 2. File Checklist

| File                                                                   | Action   | Required |
|------------------------------------------------------------------------|----------|----------|
| `drivers/iio/trx-rf/<devname>/<devname>.c`                            | Create   | Yes      |
| `drivers/iio/trx-rf/<devname>/<devname>.h`                            | Create   | Yes      |
| `drivers/iio/trx-rf/Kconfig`                                          | Modify   | Yes      |
| `drivers/iio/trx-rf/Makefile`                                         | Modify   | Yes      |
| `Documentation/devicetree/bindings/iio/frequency/adi,<devname>.yaml`  | Create   | Yes      |
| `include/dt-bindings/iio/adc/adi,<devname>.h`                         | Create   | Optional |
| `drivers/iio/trx-rf/<devname>/<devname>_conv.c`                       | Create   | Optional |
| `drivers/iio/trx-rf/<devname>/<devname>_debugfs.c`                    | Create   | Optional |

### Notes

- Multi-file drivers are typical for complex RF transceivers. A `_conv.c`
  file handles JESD204/AXI converter registration, and a `_debugfs.c` file
  provides extended debug attributes.
- A `dt-bindings` header is needed when the DT binding uses enumerations
  shared between the YAML schema and the driver (e.g., gain table channel
  masks).

---

## 3. DT Binding (.yaml)

RF transceiver bindings live under
`Documentation/devicetree/bindings/iio/frequency/adi,<devname>.yaml`.

### SPI Bus Example

```yaml
# SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)
%YAML 1.2
---
$id: http://devicetree.org/schemas/iio/frequency/adi,adxxxx.yaml#
$schema: http://devicetree.org/meta-schemas/core.yaml#

title: Analog Devices ADXXXX RF Transceiver

maintainers:
  - First Last <first.last@analog.com>

description: |
  The ADXXXX is a wideband RF transceiver with integrated ADCs, DACs,
  synthesisers, and digital filtering.

properties:
  compatible:
    enum:
      - adi,adxxxx

  reg:
    maxItems: 1

  spi-max-frequency:
    maximum: 25000000

  clocks:
    items:
      - description: Device reference clock (dev_clk)

  clock-names:
    items:
      - const: dev_clk

  clock-output-names:
    minItems: 3
    maxItems: 3
    description: |
      Clock outputs provided to downstream converters.
      Typically rx_sampl_clk, obs_sampl_clk, tx_sampl_clk.

  reset-gpios:
    maxItems: 1
    description: GPIO connected to the device RESETB pin (active low).

  '#clock-cells':
    const: 1

  adi,device-config-name:
    $ref: /schemas/types.yaml#/definitions/string
    description: Filename of the CPU profile binary in /lib/firmware/.

  adi,arm-firmware-name:
    $ref: /schemas/types.yaml#/definitions/string
    description: Filename of the ARM firmware binary.

  adi,stream-firmware-name:
    $ref: /schemas/types.yaml#/definitions/string
    description: Filename of the stream processor binary.

  adi,rx-gaintable-names:
    $ref: /schemas/types.yaml#/definitions/string-array
    description: Array of RX gain table filenames.

  adi,rx-gaintable-channel-masks:
    $ref: /schemas/types.yaml#/definitions/uint32-array
    description: |
      Channel bitmask for each gain table entry.
      Must match the length of adi,rx-gaintable-names.

required:
  - compatible
  - reg
  - clocks
  - clock-names
  - adi,device-config-name
  - adi,arm-firmware-name
  - adi,stream-firmware-name

additionalProperties: false

examples:
  - |
    #include <dt-bindings/iio/adc/adi,adxxxx.h>

    spi {
        #address-cells = <1>;
        #size-cells = <0>;

        trx@0 {
            compatible = "adi,adxxxx";
            reg = <0>;
            spi-max-frequency = <25000000>;

            clocks = <&trx_clk>;
            clock-names = "dev_clk";

            clock-output-names = "rx_sampl_clk",
                                 "obs_sampl_clk",
                                 "tx_sampl_clk";
            #clock-cells = <1>;

            reset-gpios = <&gpio 100 0>;

            adi,device-config-name = "adxxxx_profile.bin";
            adi,arm-firmware-name = "adxxxx_fw.bin";
            adi,stream-firmware-name = "adxxxx_stream.bin";
        };
    };
```

### Key DT Properties for RF Transceivers

| Property                            | Purpose                                              |
|-------------------------------------|------------------------------------------------------|
| `clocks` / `clock-names`            | Reference clock driving the transceiver PLL          |
| `reset-gpios`                       | Active-low hardware reset                            |
| `#clock-cells`                      | Device exports sampling clocks to downstream JESD    |
| `adi,device-config-name`            | Profile binary defining RF bandwidth, sample rates   |
| `adi,arm-firmware-name`             | Embedded processor firmware                          |
| `adi,stream-firmware-name`          | Stream processor firmware                            |
| `adi,rx-gaintable-names`            | Custom RX gain table files                           |
| `adi,rx-gaintable-channel-masks`    | Which RX channels each gain table applies to         |

---

## 4. Kconfig

Add to `drivers/iio/trx-rf/Kconfig` in alphabetical order:

```kconfig
config ADXXXX
	tristate "Analog Devices ADXXXX RF Transceiver driver"
	depends on SPI
	select CF_AXI_ADC
	help
	  Say yes here to build support for Analog Devices ADXXXX
	  wideband RF transceiver. Provides direct access via sysfs.

	  To compile this driver as a module, choose M here: the module will be
	  called adxxxx.
```

### Common Optional Selects for RF Transceivers

| Select                   | Purpose                                              |
|--------------------------|------------------------------------------------------|
| `CF_AXI_ADC`            | AXI converter framework for JESD204 data path       |
| `ADI_AXI_HSCI`          | AXI high-speed converter interface                   |
| `REGMAP_SPI`            | regmap SPI bus abstraction                           |
| `GPIOLIB`               | GPIO for reset, enable pins                          |

---

## 5. Makefile

Add to `drivers/iio/trx-rf/Makefile` in alphabetical order:

```makefile
obj-$(CONFIG_ADXXXX) += adxxxx/
```

For a multi-file driver within `drivers/iio/trx-rf/adxxxx/Makefile`:

```makefile
# SPDX-License-Identifier: GPL-2.0-only
obj-$(CONFIG_ADXXXX) += adxxxx_drv.o
adxxxx_drv-y := adxxxx.o adxxxx_conv.o adxxxx_debugfs.o
```

---

## 6. Driver Source (.c)

### Channel Architecture

RF transceivers expose three categories of IIO channels:

1. **LO channels** (`IIO_ALTVOLTAGE`, `.output = 1`) -- one per synthesiser.
   Frequency is handled through `ext_info` callbacks (not `info_mask`) because
   LO frequencies exceed 2^32 Hz.

2. **TX voltage channels** (`IIO_VOLTAGE`, `.output = 1`) -- one per transmit
   path. Expose `HARDWAREGAIN` (TX attenuation in mdB), `ENABLE`, and shared
   `SAMP_FREQ`.

3. **RX voltage channels** (`IIO_VOLTAGE`, `.output = 0`) -- one per receive
   path. Expose `HARDWAREGAIN` (RX gain index mapped to dB), `ENABLE`, and
   shared `SAMP_FREQ`.

### Complete Skeleton

```c
// SPDX-License-Identifier: GPL-2.0
/*
 * ADXXXX RF Transceiver
 *
 * Copyright 2024 Analog Devices Inc.
 */

#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/types.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

/* ------------------------------------------------------------------ */
/* LO ext_info                                                         */
/* ------------------------------------------------------------------ */

enum adxxxx_lo_ext_info_id {
	LOEXT_FREQ,
};

static ssize_t adxxxx_phy_lo_write(struct iio_dev *indio_dev,
				   uintptr_t private,
				   const struct iio_chan_spec *chan,
				   const char *buf, size_t len)
{
	struct adxxxx_rf_phy *phy = iio_priv(indio_dev);
	u64 freq_hz;
	int ret;

	ret = kstrtoull(buf, 10, &freq_hz);
	if (ret)
		return ret;

	scoped_guard(mutex, &phy->lock) {
		/* Program LO identified by chan->channel */
		ret = adxxxx_set_lo_freq(phy, chan->channel, freq_hz);
	}

	return ret ? ret : len;
}

static ssize_t adxxxx_phy_lo_read(struct iio_dev *indio_dev,
				  uintptr_t private,
				  const struct iio_chan_spec *chan,
				  char *buf)
{
	struct adxxxx_rf_phy *phy = iio_priv(indio_dev);
	u64 freq_hz;
	int ret;

	guard(mutex)(&phy->lock);
	ret = adxxxx_get_lo_freq(phy, chan->channel, &freq_hz);

	return ret ? ret : sysfs_emit(buf, "%llu\n", freq_hz);
}

#define _ADXXXX_EXT_LO_INFO(_name, _ident)    \
	{                                      \
		.name = _name,                 \
		.read = adxxxx_phy_lo_read,    \
		.write = adxxxx_phy_lo_write,  \
		.private = _ident,             \
	}

static const struct iio_chan_spec_ext_info adxxxx_phy_ext_lo_info[] = {
	/*
	 * IIO_CHAN_INFO_FREQUENCY uses int, which cannot hold values > 2^32.
	 * Use ext_info to expose the full u64 frequency range in Hz.
	 */
	_ADXXXX_EXT_LO_INFO("frequency", LOEXT_FREQ),
	{},
};

/* ------------------------------------------------------------------ */
/* RX / TX ext_info                                                    */
/* ------------------------------------------------------------------ */

enum adxxxx_rx_ext_info_id {
	RX_QEC,             /* Quadrature error correction tracking */
	RX_RF_BANDWIDTH,    /* RF bandwidth readback */
	RX_NCO_FREQ,        /* Digital NCO frequency (kHz) */
};

enum adxxxx_tx_ext_info_id {
	TX_QEC,             /* Quadrature error correction tracking */
	TX_LOL,             /* LO leakage tracking */
	TX_RF_BANDWIDTH,    /* RF bandwidth readback */
};

/*
 * ext_info read/write callbacks follow the same pattern as LO:
 * _ADXXXX_EXT_RX_INFO / _ADXXXX_EXT_TX_INFO macros wrapping
 * adxxxx_phy_rx_read / adxxxx_phy_rx_write (and TX equivalents).
 */

static const struct iio_chan_spec_ext_info adxxxx_phy_rx_ext_info[] = {
	_ADXXXX_EXT_RX_INFO("quadrature_tracking_en", RX_QEC),
	_ADXXXX_EXT_RX_INFO("rf_bandwidth", RX_RF_BANDWIDTH),
	_ADXXXX_EXT_RX_INFO("nco_frequency_khz", RX_NCO_FREQ),
	{},
};

static struct iio_chan_spec_ext_info adxxxx_phy_tx_ext_info[] = {
	_ADXXXX_EXT_TX_INFO("quadrature_tracking_en", TX_QEC),
	_ADXXXX_EXT_TX_INFO("lo_leakage_tracking_en", TX_LOL),
	_ADXXXX_EXT_TX_INFO("rf_bandwidth", TX_RF_BANDWIDTH),
	{},
};

/* ------------------------------------------------------------------ */
/* IIO Channel Specification                                           */
/* ------------------------------------------------------------------ */

enum adxxxx_iio_voltage_in {
	CHAN_RX1,
	CHAN_RX2,
	/* ... */
};

enum adxxxx_iio_voltage_out {
	CHAN_TX1,
	CHAN_TX2,
	/* ... */
};

static const struct iio_chan_spec adxxxx_phy_chan[] = {
	{
		/* LO1 */
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = 0,
		.extend_name = "LO1",
		.ext_info = adxxxx_phy_ext_lo_info,
	},
	{
		/* LO2 */
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = 1,
		.extend_name = "LO2",
		.ext_info = adxxxx_phy_ext_lo_info,
	},
	{
		/* TX1 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = CHAN_TX1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
				      BIT(IIO_CHAN_INFO_ENABLE),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = adxxxx_phy_tx_ext_info,
	},
	{
		/* RX1 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = CHAN_RX1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
				      BIT(IIO_CHAN_INFO_ENABLE),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = adxxxx_phy_rx_ext_info,
	},
	/* ... repeat for TX2/RX2, TX3/RX3, etc. ... */
	{
		/* Die temperature */
		.type = IIO_TEMP,
		.indexed = 1,
		.channel = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
	},
};

/* ------------------------------------------------------------------ */
/* read_raw / write_raw                                                */
/* ------------------------------------------------------------------ */

static int adxxxx_phy_read_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int *val, int *val2, long m)
{
	struct adxxxx_rf_phy *phy = iio_priv(indio_dev);

	guard(mutex)(&phy->lock);
	switch (m) {
	case IIO_CHAN_INFO_ENABLE:
		/* Read TX/RX enable state from hardware */
		*val = adxxxx_get_channel_enable(phy, chan);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_HARDWAREGAIN:
		if (chan->output) {
			/*
			 * TX: read attenuation in mdB, convert to dB.
			 * Attenuation is reported as negative gain.
			 */
			u32 atten_mdB;

			adxxxx_get_tx_atten(phy, chan->channel, &atten_mdB);
			*val = -1 * (atten_mdB / 1000);
			*val2 = (atten_mdB % 1000) * 1000;
			if (!*val)
				*val2 *= -1;
		} else {
			/*
			 * RX: read gain index, convert to dB via gain table.
			 */
			adxxxx_get_rx_gain(phy, chan->channel, val, val2);
		}
		return IIO_VAL_INT_PLUS_MICRO_DB;

	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = clk_get_rate(phy->clks[chan->output ?
					      TX_SAMPL_CLK : RX_SAMPL_CLK]);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_PROCESSED:
		/* Die temperature in millidegrees Celsius */
		*val = adxxxx_get_temperature(phy);
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

static int adxxxx_phy_write_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int val, int val2, long mask)
{
	struct adxxxx_rf_phy *phy = iio_priv(indio_dev);
	int ret;

	guard(mutex)(&phy->lock);
	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		ret = adxxxx_set_channel_enable(phy, chan, val);
		break;

	case IIO_CHAN_INFO_HARDWAREGAIN:
		if (chan->output)
			ret = adxxxx_set_tx_atten(phy, chan, val, val2);
		else
			ret = adxxxx_set_rx_gain(phy, chan, val, val2);
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

/* ------------------------------------------------------------------ */
/* iio_info                                                            */
/* ------------------------------------------------------------------ */

static const struct iio_info adxxxx_phy_info = {
	.read_raw = adxxxx_phy_read_raw,
	.write_raw = adxxxx_phy_write_raw,
};

/* ------------------------------------------------------------------ */
/* Probe                                                               */
/* ------------------------------------------------------------------ */

static int adxxxx_probe(struct spi_device *spi)
{
	const struct adxxxx_chip_info *chip_info;
	struct device *dev = &spi->dev;
	struct adxxxx_rf_phy *phy;
	struct iio_dev *indio_dev;
	struct clk *clk;
	int ret;

	chip_info = spi_get_device_match_data(spi);
	if (!chip_info)
		return dev_err_probe(dev, -ENODEV,
				     "Failed to get match data\n");

	clk = devm_clk_get_enabled(dev, "dev_clk");
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	indio_dev = devm_iio_device_alloc(dev, sizeof(*phy));
	if (!indio_dev)
		return -ENOMEM;

	phy = iio_priv(indio_dev);
	phy->spi = spi;
	phy->chip_info = chip_info;
	phy->dev_clk = clk;

	ret = devm_mutex_init(dev, &phy->lock);
	if (ret)
		return ret;

	/* Reset GPIO (active low) */
	phy->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						   GPIOD_OUT_LOW);

	/* Parse DT properties: firmware names, gain tables, etc. */
	ret = adxxxx_parse_dt(phy, dev);
	if (ret)
		return ret;

	/* Hardware init: reset, verify chip ID, load firmware, calibrate */
	ret = adxxxx_hw_init(phy);
	if (ret)
		return ret;

	/* Register sampling clocks exported to downstream JESD converters */
	ret = adxxxx_register_clocks(phy);
	if (ret)
		return ret;

	indio_dev->name = chip_info->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &adxxxx_phy_info;
	indio_dev->channels = chip_info->channels;
	indio_dev->num_channels = chip_info->num_channels;

	return devm_iio_device_register(dev, indio_dev);
}

/* ------------------------------------------------------------------ */
/* Match Tables                                                        */
/* ------------------------------------------------------------------ */

static const struct adxxxx_chip_info adxxxx_chip_info = {
	.name = "adxxxx-phy",
	.channels = adxxxx_phy_chan,
	.num_channels = ARRAY_SIZE(adxxxx_phy_chan),
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
MODULE_DESCRIPTION("Analog Devices ADXXXX RF Transceiver");
MODULE_LICENSE("GPL");
```

---

## 7. TX/RX Path Configuration

RF transceivers expose TX/RX path parameters through `ext_info` sysfs
attributes rather than standard `info_mask` bits. This allows per-channel
control of tracking calibrations, NCO configuration, and bandwidth.

### Gain Control

| Parameter                 | Channel          | Sysfs Attribute                            |
|---------------------------|------------------|--------------------------------------------|
| TX attenuation            | `out_voltageN`   | `out_voltageN_hardwaregain` (dB)           |
| RX gain                   | `in_voltageN`    | `in_voltageN_hardwaregain` (dB)            |
| RX gain mode              | `in_voltageN`    | `in_voltageN_gain_control_mode` (ext_info) |

TX gain is expressed as negative attenuation in mdB. RX gain is expressed as
a positive gain index mapped through a device-specific gain table:

```c
/* Convert gain index to dB (example from ADRV903X) */
code = MAX_RX_GAIN_mdB - (255 - index) * RX_GAIN_STEP_mdB;
*val = code / 1000;
*val2 = (code % 1000) * 1000;
```

### Tracking Calibrations (ext_info)

| Attribute                        | Direction | Purpose                                 |
|----------------------------------|-----------|-----------------------------------------|
| `quadrature_tracking_en`         | RX / TX   | Enable/disable QEC tracking calibration |
| `lo_leakage_tracking_en`         | TX        | Enable/disable LO leakage tracking     |
| `bb_dc_offset_tracking_en`       | RX        | Enable/disable baseband DC offset       |
| `adc_tracking_en`                | RX        | Enable/disable ADC tracking             |

### NCO Configuration (ext_info)

| Attribute                 | Direction | Purpose                                      |
|---------------------------|-----------|----------------------------------------------|
| `nco_en`                  | RX / TX   | Enable the digital NCO                       |
| `nco_frequency_khz`       | RX / TX   | NCO shift frequency in kHz                   |
| `nco_phase_degrees`        | RX / TX   | NCO phase offset in degrees                  |
| `nco_band_select`          | RX        | NCO band selection (band A or B)             |

### Test Tone (TX ext_info)

| Attribute                    | Purpose                                           |
|------------------------------|---------------------------------------------------|
| `test_tone_en`               | Enable TX test tone generation                    |
| `test_tone_frequency_khz`    | Test tone frequency in kHz                        |
| `test_tone_phase_degrees`    | Test tone phase offset                            |
| `test_tone_nco_select`       | Select which NCO generates the tone               |
| `test_tone_attenuation`      | Test tone attenuation level                       |

### FIR Filter Configuration

For devices with programmable FIR filters (e.g., AD9361), filters are loaded
through debugfs or a dedicated sysfs interface:

```
# Load RX FIR coefficients
echo "RX 3 -6 12 -24 ..." > /sys/bus/iio/devices/iio:device0/filter_fir_config
```

---

## 8. DT Parsing

RF transceivers parse complex configuration from the devicetree. Common
properties and the APIs used to read them:

### Reference Clock and Firmware

```c
/* Reference clock — required */
phy->dev_clk = devm_clk_get_enabled(dev, "dev_clk");
if (IS_ERR(phy->dev_clk))
	return PTR_ERR(phy->dev_clk);

/* Firmware binary names — required */
if (!device_property_read_string(dev, "adi,device-config-name", &name)) {
	ret = strscpy(phy->profile_path, name, sizeof(phy->profile_path));
	if (ret < 0)
		return dev_err_probe(dev, ret,
				     "device-config-name too long\n");
} else {
	return dev_err_probe(dev, -EINVAL,
			     "missing adi,device-config-name\n");
}
```

### Gain Tables

```c
/* RX gain table names and channel masks — optional */
n = device_property_read_string_array(dev, "adi,rx-gaintable-names",
				      names, ARRAY_SIZE(names));
if (n > 0 && !device_property_read_u32_array(dev,
					     "adi,rx-gaintable-channel-masks",
					     masks, n)) {
	for (i = 0; i < n; i++) {
		strscpy(phy->gain_tables[i].path, names[i],
			sizeof(phy->gain_tables[i].path));
		phy->gain_tables[i].channel_mask = masks[i];
	}
}
```

### Reset GPIO

```c
phy->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
```

### Common DT Properties

| Property                          | API                                         | Purpose                          |
|-----------------------------------|---------------------------------------------|----------------------------------|
| `clocks` / `clock-names`          | `devm_clk_get_enabled()`                    | Reference clock                  |
| `reset-gpios`                     | `devm_gpiod_get_optional()`                 | Hardware reset (active low)      |
| `adi,device-config-name`          | `device_property_read_string()`             | Profile binary filename          |
| `adi,arm-firmware-name`           | `device_property_read_string()`             | ARM firmware filename            |
| `adi,stream-firmware-name`        | `device_property_read_string()`             | Stream processor firmware        |
| `adi,rx-gaintable-names`          | `device_property_read_string_array()`       | Gain table filenames             |
| `adi,rx-gaintable-channel-masks`  | `device_property_read_u32_array()`          | Per-table channel bitmask        |

---

## 9. Test & Debug

### Sysfs Interface

RF transceivers expose a rich set of sysfs attributes:

```
/sys/bus/iio/devices/iio:device0/
    name                                   # "adxxxx-phy"

    # LO frequency (u64 in Hz, via ext_info)
    out_altvoltage0_LO1_frequency          # e.g., 2400000000
    out_altvoltage1_LO2_frequency          # e.g., 900000000

    # TX channels
    out_voltage0_hardwaregain              # TX1 attenuation (dB, negative)
    out_voltage0_en                        # TX1 enable (0 or 1)
    out_voltage0_rf_bandwidth              # TX1 RF bandwidth (ext_info)
    out_voltage0_quadrature_tracking_en    # TX1 QEC (ext_info)
    out_voltage0_lo_leakage_tracking_en    # TX1 LOL (ext_info)
    out_voltage_sampling_frequency         # TX sample rate (shared by type)

    # RX channels
    in_voltage0_hardwaregain               # RX1 gain (dB)
    in_voltage0_en                         # RX1 enable
    in_voltage0_rf_bandwidth               # RX1 RF bandwidth (ext_info)
    in_voltage0_quadrature_tracking_en     # RX1 QEC (ext_info)
    in_voltage0_bb_dc_offset_tracking_en   # RX1 DC offset (ext_info)
    in_voltage_sampling_frequency          # RX sample rate (shared by type)

    # Die temperature
    in_temp0_input                         # millidegrees Celsius
```

### Quick Verification Commands

```sh
# Check LO frequency
cat /sys/bus/iio/devices/iio:device0/out_altvoltage0_LO1_frequency

# Set LO to 2.4 GHz
echo 2400000000 > /sys/bus/iio/devices/iio:device0/out_altvoltage0_LO1_frequency

# Read RX gain
cat /sys/bus/iio/devices/iio:device0/in_voltage0_hardwaregain

# Set TX attenuation to -10 dB
echo -10 0 > /sys/bus/iio/devices/iio:device0/out_voltage0_hardwaregain

# Enable RX1
echo 1 > /sys/bus/iio/devices/iio:device0/in_voltage0_en

# Read RX sample rate
cat /sys/bus/iio/devices/iio:device0/in_voltage_sampling_frequency

# Read die temperature
cat /sys/bus/iio/devices/iio:device0/in_temp0_input
```

### IIO Userspace Tools

```sh
# List all IIO devices and channels
iio_info

# Read a specific attribute
iio_attr -d iio:device0 -c in_voltage0_hardwaregain

# List all device attributes
iio_attr -d iio:device0
```

### debugfs Register Access

For drivers that implement `debugfs_reg_access`:

```sh
# Read register 0x0005
echo 0x0005 > /sys/kernel/debug/iio/iio:device0/direct_reg_access
cat /sys/kernel/debug/iio/iio:device0/direct_reg_access

# Write 0xAB to register 0x0001
echo 0x0001 0xAB > /sys/kernel/debug/iio/iio:device0/direct_reg_access
```

### Extended Debug Attributes

Complex transceivers register additional debugfs entries for:

- PRBS/loopback BIST on JESD204 framers/deframers
- Per-channel QEC status readback
- Per-channel LOL (LO leakage) calibration status
- ADC calibration status

---

## 10. Key Conventions

### License

All new ADI IIO drivers must use GPL-2.0 or GPL-2.0+:

```c
// SPDX-License-Identifier: GPL-2.0
```

`MODULE_LICENSE("GPL")` at the bottom must match.

### Units

| Quantity            | Unit                     | IIO Return Type            |
|---------------------|--------------------------|----------------------------|
| LO frequency        | Hz (u64 via ext_info)    | String (sysfs_emit)        |
| TX/RX gain          | dB (mdB internally)      | `IIO_VAL_INT_PLUS_MICRO_DB`|
| Sampling frequency  | Hz                       | `IIO_VAL_INT`              |
| RF bandwidth        | Hz (via ext_info)        | String (sysfs_emit)        |
| Temperature         | millidegrees Celsius     | `IIO_VAL_INT` (PROCESSED) |
| NCO frequency       | kHz (via ext_info)       | String (sysfs_emit)        |

### Memory Management

Use `devm_*` allocations exclusively:

| Function                           | Purpose                                   |
|------------------------------------|-------------------------------------------|
| `devm_iio_device_alloc()`         | Allocate IIO device + private data        |
| `devm_iio_device_register()`      | Register IIO device (auto-unregister)     |
| `devm_clk_get_enabled()`          | Get and enable the reference clock        |
| `devm_gpiod_get_optional()`       | Get optional reset GPIO                   |
| `devm_mutex_init()`               | Initialize device mutex                   |
| `devm_of_clk_add_hw_provider()`   | Register exported sampling clocks         |

### Locking

- Use `struct mutex` protecting all device state and SPI transactions.
- Initialize with `devm_mutex_init()` in `probe()`.
- Use `guard(mutex)(&phy->lock)` (scoped, auto-release) in `read_raw` /
  `write_raw`.
- Use `scoped_guard(mutex, &phy->lock) { ... }` for narrower critical
  sections in `ext_info` callbacks.

### Coding Style

- Follow the kernel coding style (`Documentation/process/coding-style.rst`).
- Use `FIELD_GET()` / `FIELD_PREP()` with `GENMASK()` for register bitfields.
- Use `dev_err_probe()` for all error returns in `probe()`.
- Headers in alphabetical order within each group.
- Run `scripts/checkpatch.pl` before submitting.

---

## 11. Commit Format

### Subject Line

For drivers in `drivers/iio/trx-rf/`:

```
iio: trx-rf: <devname>: <brief description>
```

For drivers in `drivers/iio/frequency/`:

```
iio: frequency: <devname>: <brief description>
```

### Examples

```
iio: trx-rf: adrv903x: add support for ADRV9032 RF transceiver
iio: trx-rf: adrv903x: fix TX attenuation readback sign
iio: frequency: ad9361: add support for AD9361 2x2 RF transceiver
dt-bindings: iio: frequency: add adi,adrv903x.yaml
```

### Commit Body

- Wrap at 75 characters.
- Explain **why** the change is made.
- Reference datasheets or errata when relevant.
- Include `Signed-off-by:` (use `git commit -s`).

### Patch Series for a New RF Transceiver Driver

1. `dt-bindings: iio: frequency: add adi,<devname>.yaml` -- DT binding
2. `iio: trx-rf: <devname>: add support for <DEVNAME>` -- Driver source
3. `MAINTAINERS: add entry for <DEVNAME> IIO driver` -- Maintainer entry

### Full Example

```
iio: trx-rf: adrv903x: add support for ADRV9032 RF transceiver

The ADRV9032 is a wideband RF transceiver with 8 TX and 8 RX channels,
two independent LO synthesisers, and an integrated ARM processor for
calibration and control. The device interfaces via SPI and JESD204C
high-speed serial links.

This driver exposes:
  - LO frequency control via IIO_ALTVOLTAGE ext_info (u64 Hz)
  - Per-channel TX attenuation and RX gain via HARDWAREGAIN
  - TX/RX enable control via ENABLE
  - Sampling frequency readback via exported clock framework
  - QEC, LOL, and DC offset tracking calibration controls
  - Die temperature monitoring via IIO_TEMP PROCESSED channel

Signed-off-by: First Last <first.last@analog.com>
```
