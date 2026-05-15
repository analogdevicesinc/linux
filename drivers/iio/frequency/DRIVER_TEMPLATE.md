# Linux IIO Frequency / PLL Driver Template

This template covers IIO frequency devices: PLL synthesizers, DDS generators,
and clock distribution chips. These devices live under `drivers/iio/frequency/`
and use `IIO_ALTVOLTAGE` channels with FREQUENCY, PHASE, and optional
HARDWAREGAIN attributes.

---

## 1. Purpose

The IIO `frequency/` subdirectory covers devices that generate, distribute, or
synthesize clock and RF signals:

| Device Category        | Examples                              | Description                                    |
|------------------------|---------------------------------------|------------------------------------------------|
| PLL Synthesizer        | ADF4350, ADF4351, ADF4371, ADF5355    | Wideband fractional-N / integer-N PLLs         |
| Clock Generator        | AD9523, AD9528, HMC7044, LTC6952      | Multi-output low-jitter clock generators       |
| DDS                    | AD9834, AD9833                        | Direct digital synthesis waveform generators   |
| RF Upconverter         | ADMV1013, ADRF6780                    | Microwave upconverters with LO input           |
| RF Downconverter       | ADMV1014, ADMV4420                    | Microwave downconverters with integrated PLL   |

### IIO Channel Mapping

All frequency devices use **`IIO_ALTVOLTAGE`** output channels:

```
out_altvoltage0_frequency          # Output frequency in Hz
out_altvoltage0_phase              # Output phase in milliradians
out_altvoltage0_hardwaregain       # Output power level (if applicable)
out_altvoltage0_powerdown          # Channel power-down control (ext_info)
```

Key characteristic: frequency values often exceed 2^32 Hz (e.g. microwave PLLs
operate up to 44 GHz), so many drivers use `ext_info` string attributes instead
of `IIO_CHAN_INFO_FREQUENCY` to avoid the 32-bit `int *val` limitation.

---

## 2. File Checklist

| File                                                                     | Action   | Required |
|--------------------------------------------------------------------------|----------|----------|
| `drivers/iio/frequency/<devname>.c`                                      | Create   | Yes      |
| `drivers/iio/frequency/Kconfig`                                          | Modify   | Yes      |
| `drivers/iio/frequency/Makefile`                                         | Modify   | Yes      |
| `Documentation/devicetree/bindings/iio/frequency/adi,<devname>.yaml`     | Create   | Yes      |
| `include/linux/iio/frequency/<devname>.h`                                | Create   | Optional |
| `include/dt-bindings/iio/frequency/<devname>.h`                          | Create   | Optional |

### Notes

- A header in `include/linux/iio/frequency/` is needed when platform data
  structures are shared between the driver and board files (legacy pattern, see
  ADF4350).
- A header in `include/dt-bindings/` is needed when DT properties use enum
  constants (e.g. channel driver modes, reference source selections).
- Multi-output clock generators that register as clock providers typically
  require `COMMON_CLK` in their Kconfig dependencies.

---

## 3. Devicetree Binding

Frequency devices are almost exclusively SPI. The binding must describe the
reference clock input, optional lock-detect GPIO, and any output channel
configuration.

### PLL Synthesizer Example (ADF4350-style)

```yaml
# SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)
%YAML 1.2
---
$id: http://devicetree.org/schemas/iio/frequency/adi,<devname>.yaml#
$schema: http://devicetree.org/meta-schemas/core.yaml#

title: Analog Devices <DEVNAME> Wideband Synthesizer

maintainers:
  - First Last <first.last@analog.com>

description: |
  The <DEVNAME> is a wideband fractional-N PLL frequency synthesizer
  operating from <MIN_FREQ> to <MAX_FREQ>.

properties:
  compatible:
    enum:
      - adi,<devname>

  reg:
    maxItems: 1

  spi-max-frequency:
    maximum: 20000000

  clocks:
    maxItems: 1
    description: Reference input clock (REFIN).

  clock-names:
    const: clkin

  '#clock-cells':
    const: 0
    description:
      If present, the device registers as a clock provider. Consumers
      reference this node directly.

  clock-output-names:
    maxItems: 1
    description:
      Override the default clock output name. If omitted, the driver
      generates a name from the device node.

  gpios:
    maxItems: 1
    description: Lock detect GPIO input.

  adi,channel-spacing:
    $ref: /schemas/types.yaml#/definitions/uint32
    description: Channel spacing in Hz (determines PLL modulus).

  adi,power-up-frequency:
    $ref: /schemas/types.yaml#/definitions/uint32
    description: Initial frequency in Hz programmed at probe time.

  adi,charge-pump-current:
    $ref: /schemas/types.yaml#/definitions/uint32
    description: Charge pump current in microamps.

  adi,output-power:
    $ref: /schemas/types.yaml#/definitions/uint32
    enum: [0, 1, 2, 3]
    description: RF output power selection.

  adi,reference-doubler-enable:
    $ref: /schemas/types.yaml#/definitions/flag
    description: Enable reference frequency doubler.

  adi,reference-div2-enable:
    $ref: /schemas/types.yaml#/definitions/flag
    description: Enable reference frequency divide-by-2.

  adi,mute-till-lock-enable:
    $ref: /schemas/types.yaml#/definitions/flag
    description: Mute output until PLL achieves lock.

required:
  - compatible
  - reg
  - clocks

allOf:
  - $ref: /schemas/spi/spi-peripheral-props.yaml#

unevaluatedProperties: false

examples:
  - |
    spi {
        #address-cells = <1>;
        #size-cells = <0>;

        pll@0 {
            compatible = "adi,<devname>";
            reg = <0>;
            spi-max-frequency = <10000000>;
            clocks = <&ref_clk>;
            clock-names = "clkin";
            adi,channel-spacing = <10000>;
            adi,power-up-frequency = <2400000000>;
            adi,charge-pump-current = <2500>;
            adi,output-power = <3>;
            adi,mute-till-lock-enable;
        };
    };
```

### Multi-Output Clock Generator (AD9523-style)

For clock generators with multiple outputs, the binding uses child nodes:

```yaml
properties:
  '#clock-cells':
    const: 1
    description: Output channel index.

  '#address-cells':
    const: 1

  '#size-cells':
    const: 0

patternProperties:
  "^channel@[0-9a-f]+$":
    type: object
    additionalProperties: false
    description: Represents a clock output channel.

    properties:
      reg:
        description: Output channel number.

      adi,extended-name:
        $ref: /schemas/types.yaml#/definitions/string
        description: Descriptive name for this output.

      adi,driver-mode:
        $ref: /schemas/types.yaml#/definitions/uint32
        enum: [0, 1, 2, 3]
        description: Output driver mode (LVPECL, LVDS, HSTL, CMOS).

      adi,divider-phase:
        $ref: /schemas/types.yaml#/definitions/uint32
        description: Output divider phase offset (0-63).

      adi,channel-divider:
        $ref: /schemas/types.yaml#/definitions/uint32
        description: Output channel divider value.

    required:
      - reg

examples:
  - |
    clkgen@0 {
        compatible = "adi,ad9523-1";
        reg = <0>;
        spi-max-frequency = <10000000>;
        clocks = <&vcxo_clk>, <&ref_clk>;
        clock-names = "vcxo", "refclk";
        #clock-cells = <1>;
        #address-cells = <1>;
        #size-cells = <0>;

        channel@0 {
            reg = <0>;
            adi,extended-name = "ADC_CLK";
            adi,driver-mode = <1>;
            adi,channel-divider = <4>;
        };

        channel@9 {
            reg = <9>;
            adi,extended-name = "DAC_CLK";
            adi,driver-mode = <1>;
            adi,channel-divider = <8>;
        };
    };
```

---

## 4. Kconfig

Add under the appropriate submenu in `drivers/iio/frequency/Kconfig`. The file
has three submenus -- use the correct one:

- **Clock Generator/Distribution** -- for multi-output clock generators
- **Direct Digital Synthesis** -- for DDS/DAC devices
- **Phase-Locked Loop (PLL) frequency synthesizers** -- for PLL/VCO devices

Entries within each submenu must be in **alphabetical order**.

### PLL Synthesizer Example

```kconfig
config ADF4350
	tristate "Analog Devices ADF4350/ADF4351 Wideband Synthesizers"
	depends on SPI
	help
	  Say yes here to build support for Analog Devices ADF4350/ADF4351
	  Wideband Synthesizers. The driver provides direct access via sysfs.

	  To compile this driver as a module, choose M here: the
	  module will be called adf4350.
```

### Clock Generator with Clock Framework Example

```kconfig
config AD9523
	tristate "Analog Devices AD9523 Low Jitter Clock Generator"
	depends on SPI
	depends on COMMON_CLK
	help
	  Say yes here to build support for Analog Devices AD9523 Low Jitter
	  Clock Generator. The driver provides direct access via sysfs.

	  To compile this driver as a module, choose M here: the
	  module will be called ad9523.
```

### Common Dependencies and Selects

| Dependency / Select       | When to Use                                       |
|---------------------------|---------------------------------------------------|
| `depends on SPI`          | All SPI-connected frequency devices               |
| `depends on COMMON_CLK`   | Device registers as a clock provider               |
| `select REGMAP_SPI`       | Driver uses regmap for register access             |
| `select GPIOLIB`          | Driver uses GPIO for lock detect or chip enable    |

---

## 5. Makefile

Add to `drivers/iio/frequency/Makefile` in **alphabetical order**:

```makefile
obj-$(CONFIG_ADF4350) += adf4350.o
```

---

## 6. Driver Source

Frequency drivers differ significantly from typical IIO ADC/DAC drivers. Key
differences:

1. **Channel type is `IIO_ALTVOLTAGE`** with `.output = 1`.
2. **Frequency values use `ext_info`** string attributes (not `read_raw`)
   because frequencies often exceed 32 bits.
3. **Phase** can use `IIO_CHAN_INFO_PHASE` via `read_raw`/`write_raw`.
4. **No buffer/trigger** -- frequency devices do not produce sample streams.
5. **Clock framework integration** -- many devices register as `clk_hw`
   providers so downstream consumers (ADCs, DACs) can request clock rates.

### Complete Skeleton (PLL Synthesizer)

```c
// SPDX-License-Identifier: GPL-2.0+
/*
 * <DEVNAME> SPI Wideband Synthesizer driver
 *
 * Copyright 2024 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gcd.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/sysfs.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include <asm/div64.h>

/* ------------------------------------------------------------------ */
/* Register Definitions                                                */
/* ------------------------------------------------------------------ */

#define ADXXXX_REG0			0x00
#define ADXXXX_REG1			0x01
/* ... */

#define ADXXXX_REG0_INT_MSK		GENMASK(30, 15)
#define ADXXXX_REG0_FRACT_MSK		GENMASK(14, 3)

/* PLL limits */
#define ADXXXX_MAX_OUT_FREQ		4400000000ULL	/* Hz */
#define ADXXXX_MIN_OUT_FREQ		34375000ULL	/* Hz */
#define ADXXXX_MIN_VCO_FREQ		2200000000ULL	/* Hz */
#define ADXXXX_MAX_VCO_FREQ		4400000000ULL	/* Hz */
#define ADXXXX_MAX_FREQ_PFD		32000000UL	/* Hz */
#define ADXXXX_MAX_MODULUS		4095
#define ADXXXX_MAX_R_CNT		1023

/* ------------------------------------------------------------------ */
/* ext_info attribute IDs                                              */
/* ------------------------------------------------------------------ */

enum {
	ADXXXX_FREQ,
	ADXXXX_FREQ_REFIN,
	ADXXXX_FREQ_RESOLUTION,
	ADXXXX_PWRDOWN,
};

/* ------------------------------------------------------------------ */
/* Device State                                                        */
/* ------------------------------------------------------------------ */

struct adxxxx_state {
	struct spi_device	*spi;
	struct gpio_desc	*lock_detect_gpiod;
	struct clk		*clk;
	struct clk_hw		hw;		/* clock provider */
	unsigned long		clkin;		/* Reference input freq */
	unsigned long		chspc;		/* Channel spacing */
	unsigned long		fpfd;		/* Phase-frequency detector */
	unsigned int		r_int;		/* Integer divider */
	unsigned int		r_fract;	/* Fractional divider */
	unsigned int		r_mod;		/* Modulus */
	unsigned int		rf_div_sel;	/* RF output divider */
	unsigned long		regs[6];	/* Shadow register bank */
	unsigned long long	freq_req;	/* Requested frequency */
	/*
	 * Lock to protect the state of the device from potential
	 * concurrent writes. The device is configured via a sequence
	 * of SPI writes, and this lock prevents interleaving.
	 */
	struct mutex		lock;
	/*
	 * DMA (thus cache coherency maintenance) may require that
	 * transfer buffers live in their own cache lines.
	 */
	__be32			val __aligned(IIO_DMA_MINALIGN);
};

/* ------------------------------------------------------------------ */
/* Register Access Helpers                                             */
/* ------------------------------------------------------------------ */

static int adxxxx_write_reg(struct adxxxx_state *st, unsigned int reg,
			    unsigned long val)
{
	st->val = cpu_to_be32(val | reg);

	return spi_write(st->spi, &st->val, 4);
}

/* ------------------------------------------------------------------ */
/* PLL Frequency Calculation                                           */
/* ------------------------------------------------------------------ */

/*
 * The output frequency is:
 *
 *   f_OUT = f_PFD * (INT + FRACT/MOD) / RF_DIV
 *
 * where:
 *   f_PFD = f_REFIN * [(1 + D) / (R * (1 + T))]
 *   D = reference doubler enable
 *   T = reference divide-by-2 enable
 *   R = reference division factor
 */
static int adxxxx_set_freq(struct adxxxx_state *st, unsigned long long freq)
{
	u64 tmp;
	u32 div_gcd;

	if (freq > ADXXXX_MAX_OUT_FREQ || freq < ADXXXX_MIN_OUT_FREQ)
		return -EINVAL;

	/* Find RF output divider to bring freq into VCO range */
	st->rf_div_sel = 0;
	while (freq < ADXXXX_MIN_VCO_FREQ) {
		freq <<= 1;
		st->rf_div_sel++;
	}

	/* Calculate modulus from PFD and channel spacing */
	st->r_mod = st->fpfd / st->chspc;

	/* Calculate INT and FRACT */
	tmp = freq * (u64)st->r_mod + (st->fpfd >> 1);
	do_div(tmp, st->fpfd);
	st->r_fract = do_div(tmp, st->r_mod);
	st->r_int = tmp;

	/* Reduce fraction */
	if (st->r_fract && st->r_mod) {
		div_gcd = gcd(st->r_mod, st->r_fract);
		st->r_mod /= div_gcd;
		st->r_fract /= div_gcd;
	} else {
		st->r_fract = 0;
		st->r_mod = 1;
	}

	/* Program registers and sync to hardware */
	/* ... device-specific register writes ... */

	st->freq_req = freq;

	return 0;	/* or call sync_config() */
}

/* ------------------------------------------------------------------ */
/* IIO ext_info Read/Write (for frequencies > 2^32 Hz)                 */
/* ------------------------------------------------------------------ */

static ssize_t adxxxx_read(struct iio_dev *indio_dev,
			   uintptr_t private,
			   const struct iio_chan_spec *chan,
			   char *buf)
{
	struct adxxxx_state *st = iio_priv(indio_dev);
	unsigned long long val;
	int ret = 0;

	mutex_lock(&st->lock);
	switch ((u32)private) {
	case ADXXXX_FREQ:
		val = (u64)((st->r_int * st->r_mod) + st->r_fract) *
			(u64)st->fpfd;
		do_div(val, st->r_mod * (1 << st->rf_div_sel));
		/* Check PLL lock status */
		if (st->lock_detect_gpiod &&
		    !gpiod_get_value(st->lock_detect_gpiod)) {
			dev_dbg(&st->spi->dev, "PLL un-locked\n");
			ret = -EBUSY;
		}
		break;
	case ADXXXX_FREQ_REFIN:
		if (st->clk)
			st->clkin = clk_get_rate(st->clk);
		val = st->clkin;
		break;
	case ADXXXX_FREQ_RESOLUTION:
		val = st->chspc;
		break;
	case ADXXXX_PWRDOWN:
		val = 0;	/* read powerdown state from register */
		break;
	default:
		ret = -EINVAL;
		val = 0;
	}
	mutex_unlock(&st->lock);

	return ret < 0 ? ret : sprintf(buf, "%llu\n", val);
}

static ssize_t adxxxx_write(struct iio_dev *indio_dev,
			    uintptr_t private,
			    const struct iio_chan_spec *chan,
			    const char *buf, size_t len)
{
	struct adxxxx_state *st = iio_priv(indio_dev);
	unsigned long long readin;
	int ret;

	ret = kstrtoull(buf, 10, &readin);
	if (ret)
		return ret;

	mutex_lock(&st->lock);
	switch ((u32)private) {
	case ADXXXX_FREQ:
		ret = adxxxx_set_freq(st, readin);
		break;
	case ADXXXX_FREQ_REFIN:
		if (st->clk) {
			ret = clk_set_rate(st->clk, readin);
			if (ret < 0)
				break;
		}
		st->clkin = readin;
		ret = adxxxx_set_freq(st, st->freq_req);
		break;
	case ADXXXX_FREQ_RESOLUTION:
		if (readin == 0)
			ret = -EINVAL;
		else
			st->chspc = readin;
		break;
	case ADXXXX_PWRDOWN:
		/* Set or clear power-down bit in register */
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&st->lock);

	return ret ? ret : len;
}

/* ------------------------------------------------------------------ */
/* IIO Channel Specification                                           */
/* ------------------------------------------------------------------ */

#define _ADXXXX_EXT_INFO(_name, _ident) { \
	.name = _name, \
	.read = adxxxx_read, \
	.write = adxxxx_write, \
	.private = _ident, \
	.shared = IIO_SEPARATE, \
}

static const struct iio_chan_spec_ext_info adxxxx_ext_info[] = {
	/*
	 * Use ext_info rather than IIO_CHAN_INFO_FREQUENCY because
	 * output frequencies can exceed 2^32 Hz.
	 */
	_ADXXXX_EXT_INFO("frequency", ADXXXX_FREQ),
	_ADXXXX_EXT_INFO("frequency_resolution", ADXXXX_FREQ_RESOLUTION),
	_ADXXXX_EXT_INFO("refin_frequency", ADXXXX_FREQ_REFIN),
	_ADXXXX_EXT_INFO("powerdown", ADXXXX_PWRDOWN),
	{ },
};

static const struct iio_chan_spec adxxxx_chan = {
	.type = IIO_ALTVOLTAGE,
	.indexed = 1,
	.output = 1,
	.channel = 0,
	.ext_info = adxxxx_ext_info,
};

/*
 * For devices with multiple outputs (e.g. RF8, RF16, RF32, AUX),
 * define per-channel ext_info arrays and a channel macro:
 *
 *   #define ADXXXX_CHANNEL(index) {             \
 *       .type = IIO_ALTVOLTAGE,                 \
 *       .output = 1,                            \
 *       .channel = index,                       \
 *       .ext_info = adxxxx_ext_info,            \
 *       .indexed = 1,                           \
 *       .info_mask_shared_by_type =             \
 *           BIT(IIO_CHAN_INFO_PHASE),            \
 *   }
 */

/* ------------------------------------------------------------------ */
/* iio_info (read_raw for PHASE, debugfs)                              */
/* ------------------------------------------------------------------ */

/*
 * For channels that support phase adjustment via the standard
 * IIO read_raw/write_raw interface (values fit in 32-bit int):
 */
static int adxxxx_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long info)
{
	struct adxxxx_state *st = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_PHASE:
		/* Return phase in milliradians */
		mutex_lock(&st->lock);
		*val = 0;	/* read from hardware */
		*val2 = 0;
		mutex_unlock(&st->lock);
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int adxxxx_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long info)
{
	switch (info) {
	case IIO_CHAN_INFO_PHASE:
		/* Write phase in milliradians to hardware */
		return 0;
	default:
		return -EINVAL;
	}
}

static int adxxxx_reg_access(struct iio_dev *indio_dev,
			     unsigned int reg, unsigned int writeval,
			     unsigned int *readval)
{
	struct adxxxx_state *st = iio_priv(indio_dev);

	if (readval) {
		/* PLL registers are often write-only; return shadow */
		if (reg > 5)
			return -EINVAL;
		*readval = st->regs[reg];
		return 0;
	}

	return adxxxx_write_reg(st, reg, writeval);
}

static const struct iio_info adxxxx_info = {
	.read_raw = adxxxx_read_raw,
	.write_raw = adxxxx_write_raw,
	.debugfs_reg_access = adxxxx_reg_access,
};

/* ------------------------------------------------------------------ */
/* Clock Provider (optional)                                           */
/* ------------------------------------------------------------------ */

/*
 * If the PLL output feeds downstream IIO devices as a clock source,
 * register a struct clk_hw so consumers can use the clk framework:
 */

#define to_adxxxx_state(_hw) container_of(_hw, struct adxxxx_state, hw)

static unsigned long adxxxx_clk_recalc_rate(struct clk_hw *hw,
					    unsigned long parent_rate)
{
	struct adxxxx_state *st = to_adxxxx_state(hw);
	unsigned long long tmp;

	tmp = (u64)(st->r_int * st->r_mod + st->r_fract) * st->fpfd;
	do_div(tmp, st->r_mod * (1 << st->rf_div_sel));

	return tmp;
}

static int adxxxx_clk_set_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long parent_rate)
{
	struct adxxxx_state *st = to_adxxxx_state(hw);

	st->clkin = parent_rate;

	return adxxxx_set_freq(st, rate);
}

static long adxxxx_clk_round_rate(struct clk_hw *hw, unsigned long rate,
				  unsigned long *parent_rate)
{
	return rate;
}

static int adxxxx_clk_prepare(struct clk_hw *hw)
{
	/* Power up the output */
	return 0;
}

static void adxxxx_clk_unprepare(struct clk_hw *hw)
{
	/* Power down the output */
}

static const struct clk_ops adxxxx_clk_ops = {
	.recalc_rate = adxxxx_clk_recalc_rate,
	.set_rate = adxxxx_clk_set_rate,
	.round_rate = adxxxx_clk_round_rate,
	.prepare = adxxxx_clk_prepare,
	.unprepare = adxxxx_clk_unprepare,
};

static int adxxxx_clk_register(struct adxxxx_state *st)
{
	struct spi_device *spi = st->spi;
	struct clk_init_data init;
	struct clk *clk;
	const char *parent_name;
	int ret;

	if (!device_property_present(&spi->dev, "#clock-cells"))
		return 0;

	if (device_property_read_string(&spi->dev, "clock-output-names",
					&init.name)) {
		init.name = devm_kasprintf(&spi->dev, GFP_KERNEL, "%s-clk",
					   fwnode_get_name(dev_fwnode(&spi->dev)));
		if (!init.name)
			return -ENOMEM;
	}

	parent_name = of_clk_get_parent_name(spi->dev.of_node, 0);
	if (!parent_name)
		return -EINVAL;

	init.ops = &adxxxx_clk_ops;
	init.parent_names = &parent_name;
	init.num_parents = 1;
	init.flags = CLK_SET_RATE_PARENT;

	st->hw.init = &init;
	clk = devm_clk_register(&spi->dev, &st->hw);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	return of_clk_add_provider(spi->dev.of_node,
				   of_clk_src_simple_get, clk);
}

/* ------------------------------------------------------------------ */
/* Probe                                                               */
/* ------------------------------------------------------------------ */

static int adxxxx_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct adxxxx_state *st;
	struct iio_dev *indio_dev;
	struct clk *clk;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;
	mutex_init(&st->lock);

	/* Reference clock from clk framework */
	clk = devm_clk_get_enabled(dev, "clkin");
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk),
				     "Failed to get reference clock\n");
	st->clk = clk;
	st->clkin = clk_get_rate(clk);

	/* Power supply */
	ret = devm_regulator_get_enable(dev, "vcc");
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to enable vcc supply\n");

	/* Lock detect GPIO (optional) */
	st->lock_detect_gpiod = devm_gpiod_get_optional(dev, NULL, GPIOD_IN);
	if (IS_ERR(st->lock_detect_gpiod))
		return dev_err_probe(dev, PTR_ERR(st->lock_detect_gpiod),
				     "Failed to get lock detect GPIO\n");

	/* Parse DT properties */
	st->chspc = 10000;	/* default channel spacing */
	device_property_read_u32(dev, "adi,channel-spacing", &st->chspc);

	/* Calculate PFD frequency */
	st->fpfd = st->clkin;	/* adjusted by R divider, doubler, etc. */

	/* IIO device setup */
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->info = &adxxxx_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = &adxxxx_chan;
	indio_dev->num_channels = 1;

	/* Register as clock provider (if #clock-cells present) */
	ret = adxxxx_clk_register(st);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to register clock provider\n");

	/* Program initial frequency (if specified) */
	{
		u32 freq = 0;

		device_property_read_u32(dev, "adi,power-up-frequency", &freq);
		if (freq) {
			ret = adxxxx_set_freq(st, freq);
			if (ret)
				return dev_err_probe(dev, ret,
					"Failed to set power-up frequency\n");
		}
	}

	return devm_iio_device_register(dev, indio_dev);
}

/* ------------------------------------------------------------------ */
/* Match Tables                                                        */
/* ------------------------------------------------------------------ */

static const struct of_device_id adxxxx_of_match[] = {
	{ .compatible = "adi,adxxxx" },
	{ }
};
MODULE_DEVICE_TABLE(of, adxxxx_of_match);

static const struct spi_device_id adxxxx_ids[] = {
	{ "adxxxx", 0 },
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
MODULE_DESCRIPTION("Analog Devices ADXXXX PLL Synthesizer");
MODULE_LICENSE("GPL");
```

### Multi-Output Clock Generator Variant (AD9523-style)

For clock generators with per-output IIO channels, the channel spec uses
`IIO_CHAN_INFO_FREQUENCY` and `IIO_CHAN_INFO_PHASE` (output frequencies are
typically within 32-bit range since they are divided-down clocks):

```c
static int ad9523_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long m)
{
	struct ad9523_state *st = iio_priv(indio_dev);

	switch (m) {
	case IIO_CHAN_INFO_RAW:
		/* 1 = enabled, 0 = powered down */
		*val = !(reg & POWER_DOWN_EN);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_FREQUENCY:
		*val = st->vco_freq / channel_divider;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_PHASE:
		/* Phase in radians as INT_PLUS_MICRO */
		code = (phase_raw * 3141592) / divider;
		*val = code / 1000000;
		*val2 = code % 1000000;
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
}
```

Each output channel registers as a `struct clk_hw` with the common clock
framework:

```c
static struct clk *ad9523_clk_register(struct iio_dev *indio_dev,
				       unsigned int num, bool is_enabled)
{
	struct ad9523_state *st = iio_priv(indio_dev);
	struct clk_init_data init;
	char name[SPI_NAME_SIZE + 8];

	sprintf(name, "%s_out%d", indio_dev->name, num);

	init.name = name;
	init.ops = &ad9523_clk_ops;
	init.num_parents = 0;
	init.flags = 0;
	output->hw.init = &init;

	return clk_register(&st->spi->dev, &output->hw);
}
```

---

## 7. PLL Programming

### VCO and Output Frequency Calculation

The fundamental PLL equation for a fractional-N synthesizer:

```
f_VCO = f_PFD * (INT + FRACT / MOD)
f_OUT = f_VCO / RF_DIV
f_PFD = f_REFIN * (1 + D) / (R * (1 + T))
```

Where:
- `INT` = integer division ratio (N divider)
- `FRACT` = fractional numerator
- `MOD` = fractional modulus
- `RF_DIV` = RF output divider (power of 2: 1, 2, 4, 8, 16, ...)
- `R` = reference counter
- `D` = reference doubler enable (0 or 1)
- `T` = reference divide-by-2 enable (0 or 1)

### Frequency Planning Algorithm

```
1. Validate requested frequency against device min/max range.
2. Determine RF_DIV: divide-up frequency until it falls within VCO range.
3. Calculate PFD frequency from reference clock and R divider.
4. Compute MOD from PFD / channel_spacing.
5. Compute INT and FRACT:
     N_total = f_VCO * MOD / f_PFD
     INT = N_total / MOD
     FRACT = N_total % MOD
6. Reduce FRACT/MOD by their GCD.
7. Validate INT against prescaler minimum (4/5 prescaler: min 23,
   8/9 prescaler: min 75).
8. Program register bank and sync to hardware (write regs 5 down to 0).
```

### Lock Detection

PLL lock can be verified in two ways:

1. **GPIO lock detect pin**: Read the LD pin via `gpiod_get_value()`.
   The lock detect output is typically configured via a MUXOUT register field.

2. **Register readback**: Some devices provide a lock status register bit
   (e.g., ADF4371 register 0x7C, AD9523 status register).

Drivers should check lock status when the frequency is read back and return
`-EBUSY` if the PLL is not locked.

### Charge Pump Current

The charge pump current affects loop bandwidth and lock time. It is typically
configured via DT property (`adi,charge-pump-current`) and programmed into a
register bitfield. Common values: 312.5 uA to 5 mA in discrete steps.

---

## 8. Devicetree Parsing

### Reference Clock

Frequency devices obtain their reference clock from the common clock framework:

```c
clk = devm_clk_get_enabled(dev, "clkin");
if (IS_ERR(clk))
	return dev_err_probe(dev, PTR_ERR(clk),
			     "Failed to get reference clock\n");

st->clkin = clk_get_rate(clk);
```

### PLL Configuration Properties

Typical DT properties parsed in the probe function:

```c
static int adxxxx_parse_dt(struct device *dev, struct adxxxx_state *st)
{
	unsigned int tmp;

	/* Channel spacing (default 10 kHz) */
	st->chspc = 10000;
	device_property_read_u32(dev, "adi,channel-spacing", &st->chspc);

	/* Initial frequency */
	tmp = 0;
	device_property_read_u32(dev, "adi,power-up-frequency", &tmp);
	st->power_up_freq = tmp;

	/* Charge pump current */
	tmp = 2500;
	device_property_read_u32(dev, "adi,charge-pump-current", &tmp);
	/* Convert to register field value */

	/* Boolean flags */
	st->ref_doubler_en = device_property_read_bool(dev,
					"adi,reference-doubler-enable");
	st->ref_div2_en = device_property_read_bool(dev,
					"adi,reference-div2-enable");
	st->mute_till_lock = device_property_read_bool(dev,
					"adi,mute-till-lock-enable");

	/* Output power level */
	tmp = 0;
	device_property_read_u32(dev, "adi,output-power", &tmp);

	return 0;
}
```

### Multi-Output Channel Configuration (child nodes)

For clock generators with per-output child nodes:

```c
static int adxxxx_parse_channels(struct iio_dev *indio_dev,
				 struct device *dev)
{
	struct adxxxx_state *st = iio_priv(indio_dev);
	unsigned int num_outputs;
	unsigned int reg;
	int ret;

	num_outputs = device_get_child_node_count(dev);

	device_for_each_child_node_scoped(dev, child) {
		ret = fwnode_property_read_u32(child, "reg", &reg);
		if (ret)
			return dev_err_probe(dev, ret,
					     "Missing reg in %pfwP\n", child);

		/* Parse per-output properties */
		fwnode_property_read_u32(child, "adi,channel-divider",
					&st->outputs[reg].divider);
		fwnode_property_read_u32(child, "adi,driver-mode",
					&st->outputs[reg].driver_mode);
		fwnode_property_read_u32(child, "adi,divider-phase",
					&st->outputs[reg].phase);
		fwnode_property_read_string(child, "adi,extended-name",
					   &st->outputs[reg].name);
	}

	return 0;
}
```

---

## 9. Test & Debug

### sysfs Interface

Frequency devices expose `out_altvoltage` channels:

```sh
# Read current output frequency
cat /sys/bus/iio/devices/iio:device0/out_altvoltage0_frequency

# Set output frequency (in Hz)
echo 2400000000 > /sys/bus/iio/devices/iio:device0/out_altvoltage0_frequency

# Read reference input frequency
cat /sys/bus/iio/devices/iio:device0/out_altvoltage0_refin_frequency

# Read/set channel spacing (frequency resolution)
cat /sys/bus/iio/devices/iio:device0/out_altvoltage0_frequency_resolution

# Power down the output
echo 1 > /sys/bus/iio/devices/iio:device0/out_altvoltage0_powerdown
```

### Lock Detect Status

For PLL lock status with lock detect GPIO:

```sh
# Reading frequency returns -EBUSY if PLL is unlocked
cat /sys/bus/iio/devices/iio:device0/out_altvoltage0_frequency
# If unlocked: "cat: read error: Device or resource busy"
```

For devices with status registers exposed via custom attributes:

```sh
# Clock generators often expose lock status as custom IIO attributes
cat /sys/bus/iio/devices/iio:device0/pll1_locked
cat /sys/bus/iio/devices/iio:device0/pll2_locked
```

### debugfs Register Access

PLL synthesizers are often write-only devices. The `debugfs_reg_access`
callback typically returns shadow register values for reads:

```sh
# Read shadow register 0
echo 0x00 > /sys/kernel/debug/iio/iio:device0/direct_reg_access
cat /sys/kernel/debug/iio/iio:device0/direct_reg_access

# Write register
echo 0x01 0x12345678 > /sys/kernel/debug/iio/iio:device0/direct_reg_access
```

### Multi-Output Clock Generator Testing

```sh
# List all output channels
iio_info

# Read per-output frequency (clock generators)
cat /sys/bus/iio/devices/iio:device0/out_altvoltage0_frequency
cat /sys/bus/iio/devices/iio:device0/out_altvoltage1_frequency

# Read output phase
cat /sys/bus/iio/devices/iio:device0/out_altvoltage0_phase

# Enable/disable an output
echo 1 > /sys/bus/iio/devices/iio:device0/out_altvoltage0_raw
echo 0 > /sys/bus/iio/devices/iio:device0/out_altvoltage0_raw
```

### Clock Framework Verification

When the device registers as a clock provider:

```sh
# Check registered clocks
cat /sys/kernel/debug/clk/clk_summary | grep <devname>

# Verify downstream consumers see the correct rate
cat /sys/kernel/debug/clk/<devname>-clk/clk_rate
```

---

## 10. Key Conventions

### License

```c
// SPDX-License-Identifier: GPL-2.0+
/* ... */
MODULE_LICENSE("GPL");
```

All new ADI IIO drivers use GPL-2.0 or GPL-2.0+. The SPDX identifier goes
on the very first line.

### Frequency Units

- All frequencies exposed to userspace are in **Hz** (not kHz or MHz).
- Phase values via `IIO_CHAN_INFO_PHASE` are in **milliradians**.
- Phase values via `ext_info` vary by driver (some use millidegrees).

### Clock Framework Integration

- Use `depends on COMMON_CLK` in Kconfig when the driver registers clock
  outputs.
- Use `devm_clk_get_enabled()` to acquire and enable reference clock inputs.
- Implement `struct clk_ops` with at minimum: `recalc_rate`, `round_rate`,
  `set_rate`.
- Add `prepare`/`unprepare` callbacks to control output power state.
- For multi-output devices, register one `clk_hw` per output and use
  `of_clk_add_hw_provider()` with a lookup function.
- For single-output devices, use `of_clk_add_provider()` with
  `of_clk_src_simple_get`.

### ext_info vs read_raw for Frequency

Two patterns exist for exposing frequency values:

1. **`ext_info` string attributes** (ADF4350, ADF4371 pattern): Used when
   frequencies exceed 2^32 Hz. The `read`/`write` callbacks use
   `kstrtoull()`/`sprintf("%llu")` for 64-bit values. This is the common
   pattern for RF PLLs.

2. **`IIO_CHAN_INFO_FREQUENCY` via `read_raw`/`write_raw`** (AD9523 pattern):
   Used when frequencies fit in a 32-bit integer (clock generators with outputs
   typically below ~4 GHz). Returns `IIO_VAL_INT`.

### Write-Only Devices

Many PLL synthesizers (ADF4350, ADF4351) are write-only -- they have no SPI
readback capability. Drivers maintain a shadow register array and return
shadow values from `debugfs_reg_access`. Some devices (ADF4371) can verify SPI
communication via the MUXOUT pin.

### Coding Style

- Follow kernel coding style (`Documentation/process/coding-style.rst`).
- Run `scripts/checkpatch.pl` before submitting.
- Use `FIELD_GET()` / `FIELD_PREP()` with `GENMASK()` for register bitfields.
- Use `BIT()` for single-bit definitions.
- Use `dev_err_probe()` for all error returns in `probe()`.
- Include headers in alphabetical order within each group.
- Use `do_div()` from `<asm/div64.h>` for 64-bit division on 32-bit platforms.
- Use `gcd()` from `<linux/gcd.h>` to reduce fractions.
- Use `guard(mutex)` or explicit `mutex_lock()`/`mutex_unlock()` pairs.

---

## 11. Commit Message Format

### Subject Line

```
iio: frequency: <devname>: <brief description>
```

Examples:
```
iio: frequency: adf4350: add support for ADF4350/ADF4351
iio: frequency: ad9523: fix PLL2 divider calculation
iio: frequency: adf4371: add phase adjustment support
iio: frequency: hmc7044: use devm_clk_get_enabled
```

### Patch Series for a New Frequency Driver

```
1/3  dt-bindings: iio: frequency: add adi,<devname>.yaml
2/3  iio: frequency: <devname>: add support for <DEVNAME>
3/3  MAINTAINERS: add entry for <DEVNAME> IIO driver
```

### Full Example

```
iio: frequency: adf4350: add support for ADF4350/ADF4351

The ADF4350/ADF4351 are wideband fractional-N/integer-N PLL
frequency synthesizers with integrated VCO. The ADF4350 covers
137.5 MHz to 4.4 GHz, while the ADF4351 extends down to 35 MHz.

This driver supports:
  - Frequency tuning via IIO sysfs ext_info attributes
  - Configurable channel spacing and charge pump current
  - Lock detect via GPIO
  - Reference clock from the common clock framework
  - Optional registration as a clock provider

Signed-off-by: First Last <first.last@analog.com>
```
