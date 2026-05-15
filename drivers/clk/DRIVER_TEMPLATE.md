# Linux Clock Driver Template

Reference driver: `drivers/clk/adi/clk-ad9545.c` (+ `clk-ad9545-spi.c`,
`clk-ad9545-i2c.c`)

This template covers every file needed to add a new clock driver to the
Linux kernel's Common Clock Framework (CCF).  Replace `<devname>` with the
part number (e.g., `ad9545`) and `<DEVNAME>` with its uppercase form
(e.g., `AD9545`) throughout.

---

## 1. Purpose & Subsystem Mapping

Clock drivers live under `drivers/clk/` and register with the **Common
Clock Framework** (`include/linux/clk-provider.h`).  They model hardware
that generates, multiplies, divides, or distributes clock signals --
PLLs, clock generators, jitter cleaners, and clock distribution ICs such
as:

- **AD9545** -- Quad-input, 10-output, dual DPLL synchronizer
- **AD9523 / AD9528** -- Low-jitter clock generators
- **HMC7044** -- Multi-output clock distribution
- **ADF4350 / ADF4368** -- Wideband PLL synthesizers
- **LTC6948** -- Fractional-N synthesizer with integrated VCO

These devices provide clocks to other peripherals via `#clock-cells` in
devicetree.  Consumer drivers obtain them through `devm_clk_get()` and
the CCF handles rate negotiation, gating, and parent selection
transparently.

---

## 2. File Checklist

```
drivers/clk/adi/
    clk-<devname>.c              # Core driver (clock tree, PLL math, DT parsing)
    clk-<devname>.h              # Shared header (probe prototype, common defines)
    clk-<devname>-spi.c          # SPI bus glue (regmap_init_spi, spi_driver)
    clk-<devname>-i2c.c          # I2C bus glue (regmap_init_i2c, i2c_driver)  [if applicable]
    Kconfig                      # Append COMMON_CLK_<DEVNAME> entry
    Makefile                     # Append obj-$(CONFIG_...) line

Documentation/devicetree/bindings/clock/
    clk-<devname>.yaml           # DT binding schema

include/dt-bindings/clock/
    <devname>.h                  # Clock index constants (optional, for #clock-cells > 0)
```

If only one bus variant exists (SPI-only or I2C-only) the bus glue can
be folded directly into `clk-<devname>.c` and the shared header omitted.

---

## 3. Devicetree Binding (`.yaml`)

```yaml
# SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)
%YAML 1.2
---
$id: http://devicetree.org/schemas/clock/clk-<devname>.yaml#
$schema: http://devicetree.org/meta-schemas/core.yaml#

title: Analog Devices <DEVNAME> Clock Generator

maintainers:
  - Your Name <your.name@analog.com>

description: |
  Analog Devices <DEVNAME> <short description>.
  https://www.analog.com/en/<devname>.html

properties:
  compatible:
    enum:
      - adi,<devname>

  reg:
    maxItems: 1

  "#clock-cells":
    const: 1
    description:
      The cell value selects the output channel index.
      Use 2 cells if outputs are grouped by type (e.g., <&phandle type index>).

  clocks:
    minItems: 1
    maxItems: <N>
    description: |
      Input reference clock(s).  The order must match clock-names.

  clock-names:
    items:
      - const: ref-a
      # Add more items as needed

  clock-output-names:
    maxItems: <M>
    description: |
      Optional override for output clock names.

  assigned-clocks:
    description: |
      Phandle + clock specifier pairs identifying outputs to configure.
    minItems: 1
    maxItems: <M>

  assigned-clock-rates:
    minItems: 1
    maxItems: <M>

  '#address-cells':
    const: 1

  '#size-cells':
    const: 0

  spi-max-frequency:
    maximum: 10000000

patternProperties:
  "^channel@[0-9]+$":
    type: object
    description: Per-output-channel configuration.

    properties:
      reg:
        description: Channel index.

      adi,output-divider:
        $ref: /schemas/types.yaml#/definitions/uint32
        description: Output divider ratio.

    required:
      - reg

required:
  - compatible
  - reg
  - "#clock-cells"
  - clocks

additionalProperties: false

examples:
  - |
    spi {
        #address-cells = <1>;
        #size-cells = <0>;

        clock-generator@0 {
            compatible = "adi,<devname>";
            reg = <0>;
            spi-max-frequency = <10000000>;

            #clock-cells = <1>;
            clocks = <&ref_clk>;
            clock-output-names = "out0", "out1", "out2", "out3";
        };
    };
```

### Key binding properties for clock devices

| Property | When to use |
|---|---|
| `#clock-cells` | Always.  `<0>` = single output, `<1>` = index, `<2>` = type + index |
| `clock-output-names` | Multi-output devices; lets consumers refer to outputs by name |
| `clocks` / `clock-names` | Input reference clocks the device needs |
| `assigned-clocks` | Board-level rate/parent assignment at boot |
| `assigned-clock-rates` | Paired with `assigned-clocks` for initial rates |
| `assigned-clock-phases` | Phase offset for outputs (less common) |

---

## 4. Kconfig

Append to `drivers/clk/adi/Kconfig` (or to `drivers/clk/Kconfig` if not
using a subdirectory).  Keep entries in alphabetical order.

```kconfig
config COMMON_CLK_<DEVNAME>
	tristate

config COMMON_CLK_<DEVNAME>_SPI
	tristate "Analog Devices <DEVNAME> via SPI"
	depends on REGMAP_SPI
	select COMMON_CLK_<DEVNAME>
	help
	  Say yes here to build support for Analog Devices <DEVNAME>
	  <one-line description>.

	  To compile this driver as a module, choose M here: the
	  module will be called clk-<devname>-spi.

config COMMON_CLK_<DEVNAME>_I2C
	tristate "Analog Devices <DEVNAME> via I2C"
	depends on REGMAP_I2C
	select COMMON_CLK_<DEVNAME>
	help
	  Say yes here to build support for Analog Devices <DEVNAME>
	  <one-line description>.

	  To compile this driver as a module, choose M here: the
	  module will be called clk-<devname>-i2c.
```

### Pattern notes

- The base symbol `COMMON_CLK_<DEVNAME>` (no `_SPI`/`_I2C`) is a hidden
  tristate selected by the bus-specific symbols.  It builds the core
  driver object.
- For a single-bus device, a single `COMMON_CLK_<DEVNAME>` tristate with
  an explicit `depends on SPI` (or `I2C`) is sufficient.
- Some drivers use `CLK_<DEVNAME>` instead of `COMMON_CLK_<DEVNAME>` --
  either form is accepted, but `COMMON_CLK_` is the dominant convention
  for clock generators.

---

## 5. Makefile

Append to `drivers/clk/adi/Makefile`:

```makefile
obj-$(CONFIG_COMMON_CLK_<DEVNAME>)     += clk-<devname>.o
obj-$(CONFIG_COMMON_CLK_<DEVNAME>_SPI) += clk-<devname>-spi.o
obj-$(CONFIG_COMMON_CLK_<DEVNAME>_I2C) += clk-<devname>-i2c.o
```

If the driver lives directly in `drivers/clk/` rather than a
subdirectory, add the lines to `drivers/clk/Makefile` instead.

---

## 6. Driver Source (`.c`)

### 6.1 Includes

```c
// SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause
/*
 * <DEVNAME> Clock Generator Driver
 *
 * Copyright (C) YYYY Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include <dt-bindings/clock/<devname>.h>   /* if using dt-bindings constants */
#include "clk-<devname>.h"
```

### 6.2 Per-output clock structure

Each hardware clock is represented by a `struct clk_hw` embedded in a
device-specific wrapper:

```c
struct <devname>_output_clk {
	struct clk_hw		hw;
	struct <devname>_state	*st;
	unsigned int		address;    /* channel/output index */
	bool			enabled;
};

#define to_output_clk(_hw) container_of(_hw, struct <devname>_output_clk, hw)
```

### 6.3 Top-level device state

```c
struct <devname>_state {
	struct device			*dev;
	struct regmap			*regmap;
	struct <devname>_output_clk	outputs[<DEVNAME>_NUM_OUTPUTS];
	struct clk_hw			*clk_hws[<DEVNAME>_NUM_OUTPUTS];
	struct clk_hw_onecell_data	*clk_data;   /* for of_clk_add_hw_provider */
};
```

### 6.4 `clk_ops` implementation

Every clock node in the tree needs a `struct clk_ops`.  The required
callbacks depend on what the hardware does:

```c
/*
 * .recalc_rate  -- Read hardware and return the current rate.
 * .round_rate   -- Find the closest achievable rate (do NOT touch HW).
 * .set_rate     -- Program the hardware for the requested rate.
 * .enable       -- Ungate / power-up the output.
 * .disable      -- Gate / power-down the output.
 * .is_enabled   -- Return 1 if the output is active.
 * .determine_rate -- Modern replacement for round_rate; preferred for
 *                    new drivers (handles parent selection).
 */

static unsigned long <devname>_recalc_rate(struct clk_hw *hw,
					   unsigned long parent_rate)
{
	struct <devname>_output_clk *clk = to_output_clk(hw);
	unsigned int div;

	/* Read the divider from hardware via regmap */
	regmap_read(clk->st->regmap, <DEVNAME>_OUTPUT_DIV(clk->address), &div);
	if (!div)
		div = 1;

	return DIV_ROUND_CLOSEST(parent_rate, div);
}

static long <devname>_round_rate(struct clk_hw *hw, unsigned long rate,
				 unsigned long *parent_rate)
{
	unsigned long div;

	if (!rate)
		return *parent_rate;

	div = DIV_ROUND_CLOSEST(*parent_rate, rate);
	div = clamp(div, 1UL, (unsigned long)<DEVNAME>_MAX_DIV);

	return DIV_ROUND_CLOSEST(*parent_rate, div);
}

static int <devname>_set_rate(struct clk_hw *hw, unsigned long rate,
			      unsigned long parent_rate)
{
	struct <devname>_output_clk *clk = to_output_clk(hw);
	unsigned long div;

	div = DIV_ROUND_CLOSEST(parent_rate, rate);
	div = clamp(div, 1UL, (unsigned long)<DEVNAME>_MAX_DIV);

	return regmap_write(clk->st->regmap,
			    <DEVNAME>_OUTPUT_DIV(clk->address), div);
}

static int <devname>_enable(struct clk_hw *hw)
{
	struct <devname>_output_clk *clk = to_output_clk(hw);

	return regmap_clear_bits(clk->st->regmap,
				 <DEVNAME>_OUTPUT_CTRL(clk->address),
				 <DEVNAME>_OUTPUT_POWERDOWN);
}

static void <devname>_disable(struct clk_hw *hw)
{
	struct <devname>_output_clk *clk = to_output_clk(hw);

	regmap_set_bits(clk->st->regmap,
			<DEVNAME>_OUTPUT_CTRL(clk->address),
			<DEVNAME>_OUTPUT_POWERDOWN);
}

static int <devname>_is_enabled(struct clk_hw *hw)
{
	struct <devname>_output_clk *clk = to_output_clk(hw);
	unsigned int val;

	regmap_read(clk->st->regmap,
		    <DEVNAME>_OUTPUT_CTRL(clk->address), &val);

	return !(val & <DEVNAME>_OUTPUT_POWERDOWN);
}

static const struct clk_ops <devname>_output_clk_ops = {
	.recalc_rate = <devname>_recalc_rate,
	.round_rate  = <devname>_round_rate,
	.set_rate    = <devname>_set_rate,
	.enable      = <devname>_enable,
	.disable     = <devname>_disable,
	.is_enabled  = <devname>_is_enabled,
};
```

### 6.5 Clock registration

```c
static int <devname>_register_clocks(struct <devname>_state *st)
{
	struct clk_init_data init = {};
	const char *parent_name;
	int i, ret;

	st->clk_data = devm_kzalloc(st->dev,
			struct_size(st->clk_data, hws, <DEVNAME>_NUM_OUTPUTS),
			GFP_KERNEL);
	if (!st->clk_data)
		return -ENOMEM;

	parent_name = __clk_get_name(/* parent clk_hw or input clk */);

	for (i = 0; i < <DEVNAME>_NUM_OUTPUTS; i++) {
		struct <devname>_output_clk *out = &st->outputs[i];

		init.name = <devname>_output_names[i];
		init.ops = &<devname>_output_clk_ops;
		init.parent_names = &parent_name;
		init.num_parents = 1;
		init.flags = CLK_SET_RATE_PARENT;  /* propagate rate requests up */

		out->hw.init = &init;
		out->st = st;
		out->address = i;

		ret = devm_clk_hw_register(st->dev, &out->hw);
		if (ret)
			return ret;

		st->clk_data->hws[i] = &out->hw;
	}

	st->clk_data->num = <DEVNAME>_NUM_OUTPUTS;

	return devm_of_clk_add_hw_provider(st->dev,
					    of_clk_hw_onecell_get,
					    st->clk_data);
}
```

### 6.6 Probe and module boilerplate

For the core (bus-independent) driver, export a probe helper:

```c
int <devname>_probe(struct device *dev, struct regmap *regmap)
{
	struct <devname>_state *st;
	int ret;

	st = devm_kzalloc(dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->dev = dev;
	st->regmap = regmap;

	ret = <devname>_parse_dt(st);
	if (ret)
		return ret;

	ret = <devname>_setup(st);
	if (ret)
		return ret;

	return <devname>_register_clocks(st);
}
EXPORT_SYMBOL_GPL(<devname>_probe);

MODULE_AUTHOR("Your Name <your.name@analog.com>");
MODULE_DESCRIPTION("Analog Devices <DEVNAME> Clock Generator");
MODULE_LICENSE("Dual BSD/GPL");
```

### 6.7 SPI bus glue (`clk-<devname>-spi.c`)

```c
// SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause

#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>

#include "clk-<devname>.h"

static const struct regmap_config <devname>_regmap_config = {
	.reg_bits = 16,       /* adjust per datasheet */
	.val_bits = 8,
	.max_register = 0x3FFF,
	.use_single_read = true,
	.use_single_write = true,
};

static int <devname>_spi_probe(struct spi_device *spi)
{
	struct regmap *regmap;

	regmap = devm_regmap_init_spi(spi, &<devname>_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(&spi->dev, PTR_ERR(regmap),
				     "devm_regmap_init_spi failed\n");

	return <devname>_probe(&spi->dev, regmap);
}

static const struct of_device_id <devname>_spi_of_match[] = {
	{ .compatible = "adi,<devname>" },
	{ }
};
MODULE_DEVICE_TABLE(of, <devname>_spi_of_match);

static const struct spi_device_id <devname>_spi_id[] = {
	{ "<devname>", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, <devname>_spi_id);

static struct spi_driver <devname>_spi_driver = {
	.driver = {
		.name = "<devname>",
		.of_match_table = <devname>_spi_of_match,
	},
	.probe    = <devname>_spi_probe,
	.id_table = <devname>_spi_id,
};
module_spi_driver(<devname>_spi_driver);

MODULE_AUTHOR("Your Name <your.name@analog.com>");
MODULE_DESCRIPTION("Analog Devices <DEVNAME> SPI");
MODULE_LICENSE("Dual BSD/GPL");
```

---

## 7. Clock Tree & PLL Programming

Most clock generators have an internal tree:

```
Input Ref ---> R-divider ---> PFD ---> Charge Pump ---> VCO ---> Output Dividers
                               ^                         |
                               |                         |
                               +--- N-divider (feedback)-+
```

### VCO frequency calculation

```
f_VCO = f_REF * (N + FRAC/MOD) / R
```

Where `N` is the integer divider, `FRAC/MOD` is the fractional part
(for fractional-N PLLs), and `R` is the reference divider.

### Output frequency

```
f_OUT = f_VCO / output_divider
```

### Implementation checklist

1. **VCO range check** -- Verify `f_VCO` is within the device's valid
   VCO range before programming.  Example from AD9545:
   ```c
   static const unsigned int ad9545_apll_rate_ranges_hz[2][2] = {
       {2424000000U, 3232000000U},
       {3232000000U, 4040000000U},
   };
   ```

2. **PFD frequency limit** -- The phase-frequency detector has a maximum
   input frequency.  Compute `f_PFD = f_REF / R` and clamp.

3. **Lock detection** -- After programming, poll the lock-detect status
   register and warn if the PLL fails to lock:
   ```c
   ret = regmap_read(st->regmap, <DEVNAME>_PLL_STATUS, &val);
   if (!(val & <DEVNAME>_PLL_LOCKED))
       dev_warn(st->dev, "PLL unlocked\n");
   ```

4. **IO update / latch** -- Some devices require writing to a latch
   register after all configuration writes:
   ```c
   regmap_write(st->regmap, <DEVNAME>_IO_UPDATE, <DEVNAME>_UPDATE_REGS);
   ```

5. **VCO calibration** -- Many PLLs need an explicit calibration
   sequence after configuration changes.

---

## 8. Devicetree Parsing

Use `fwnode` APIs to parse DT properties.  This keeps the driver
firmware-agnostic (works with ACPI as well as OF).

```c
static int <devname>_parse_dt(struct <devname>_state *st)
{
	struct fwnode_handle *fwnode = dev_fwnode(st->dev);
	struct fwnode_handle *child;
	int ret;

	/* Parse device-level properties */
	st->ref_clk = devm_clk_get(st->dev, "ref");
	if (IS_ERR(st->ref_clk))
		return dev_err_probe(st->dev, PTR_ERR(st->ref_clk),
				     "failed to get ref clock\n");

	/* Parse per-channel child nodes */
	fwnode_for_each_available_child_node(fwnode, child) {
		u32 reg;

		ret = fwnode_property_read_u32(child, "reg", &reg);
		if (ret < 0) {
			fwnode_handle_put(child);
			return ret;
		}

		if (reg >= <DEVNAME>_NUM_OUTPUTS) {
			fwnode_handle_put(child);
			return -EINVAL;
		}

		st->outputs[reg].output_used = true;

		/* Read channel-specific properties */
		fwnode_property_read_u32(child, "adi,output-divider",
					&st->outputs[reg].divider);
	}

	return 0;
}
```

### Common DT patterns for clock drivers

| Pattern | Usage |
|---|---|
| `clocks = <&ref>;` | Input reference clock phandle |
| `clock-names = "ref";` | Name for `devm_clk_get(dev, "ref")` |
| `clock-output-names` | Override default output names |
| `#clock-cells = <1>;` | Consumers use `<&devname INDEX>` |
| `assigned-clocks` | Board-level initial rate assignment |
| Child nodes with `reg` | Per-output-channel configuration |

---

## 9. Test & Debug

### clk_summary

The clock framework exposes the full clock tree in debugfs:

```
# cat /sys/kernel/debug/clk/clk_summary
                                 enable  prepare  protect
   clock                          count    count    count     rate
------------------------------------------------------------------------
 ref_clk                              1        1        0  100000000
    <devname>_pll                     1        1        0  2500000000
       <devname>_out0                 1        1        0  125000000
       <devname>_out1                 0        0        0  250000000
```

### Rate queries

```
# cat /sys/kernel/debug/clk/<devname>_out0/clk_rate
125000000
```

### Trigger rate changes (from a consumer or test harness)

```c
struct clk *clk = devm_clk_get(dev, "output0");
clk_set_rate(clk, 156250000);
pr_info("actual rate: %lu\n", clk_get_rate(clk));
```

### Lock status

For devices with lock-detect registers, check via debugfs or
`dev_warn()` at probe time.  The AD9545 driver checks lock after
calibration:

```c
ret = regmap_read(st->regmap, AD9545_PLL_STATUS, &val);
for (i = 0; i < ARRAY_SIZE(st->pll_clks); i++)
    if (st->pll_clks[i].pll_used && !AD9545_PLLX_LOCK(i, val))
        dev_warn(st->dev, "PLL%d unlocked.\n", i);
```

### Common issues

| Symptom | Likely cause |
|---|---|
| Clock rate reads as 0 | `recalc_rate` returns 0; check divider read |
| Consumer gets `-ENOENT` | `#clock-cells` mismatch or missing provider registration |
| PLL won't lock | VCO out of range, missing calibration, or bad input ref |
| Rate won't change | Missing `CLK_SET_RATE_PARENT` flag or `round_rate` clamping |

---

## 10. Key Conventions

### License

Clock drivers in `drivers/clk/adi/` use `GPL-2.0 OR BSD-2-Clause` (dual
license).  Upstream mainline clock drivers typically use `GPL-2.0-only`.

### `clk_hw` vs legacy `clk` API

New drivers must use the `clk_hw`-based API:

| Preferred (clk_hw) | Legacy (avoid) |
|---|---|
| `devm_clk_hw_register()` | `devm_clk_register()` |
| `devm_of_clk_add_hw_provider()` | `of_clk_add_provider()` |
| `of_clk_hw_onecell_get()` | `of_clk_src_onecell_get()` |

### Clock flags

| Flag | Meaning |
|---|---|
| `CLK_SET_RATE_PARENT` | Rate change requests propagate to the parent |
| `CLK_SET_RATE_GATE` | Rate can only change while the clock is disabled |
| `CLK_SET_RATE_NO_REPARENT` | Do not switch parents to satisfy a rate request |
| `CLK_GET_RATE_NOCACHE` | Always call `recalc_rate`, never cache |
| `CLK_IGNORE_UNUSED` | Do not disable at late init even if unused |

### Resource management

- Always use `devm_*` variants (`devm_clk_hw_register`,
  `devm_of_clk_add_hw_provider`, `devm_regmap_init_spi`, etc.) so
  cleanup is automatic on driver unbind.
- Use `devm_clk_get()` for input clocks.
- Use `dev_err_probe()` instead of `dev_err()` + return for deferred
  probe support.

### `container_of` pattern

Every per-clock structure embeds `struct clk_hw`.  Recover the wrapper
with `container_of`:

```c
#define to_output_clk(_hw) container_of(_hw, struct <devname>_output_clk, hw)
```

### `determine_rate` vs `round_rate`

- `round_rate` is the older callback.  It receives a single parent rate
  pointer and cannot select among multiple parents.
- `determine_rate` is the modern replacement.  It receives a
  `struct clk_rate_request *` and can adjust both rate and parent.
  Prefer this for new drivers with parent selection.

### Register access

- Use `regmap` for SPI/I2C register access.  It handles byte ordering,
  caching, and locking.
- For multi-byte fields use `regmap_bulk_read()` / `regmap_bulk_write()`.
- Use `FIELD_PREP()` / `FIELD_GET()` with `GENMASK()` for bitfields.

---

## 11. Commit Message Format

Clock driver commits use the `clk:` prefix:

```
clk: <devname>: add support for <DEVNAME>

Add driver for the Analog Devices <DEVNAME>, a <short description>.
The driver registers <N> output clocks with the Common Clock Framework
and supports rate setting, gating, and PLL lock detection.

Signed-off-by: Your Name <your.name@analog.com>
```

For binding patches:

```
dt-bindings: clock: add Analog Devices <DEVNAME>

Add devicetree binding documentation for the Analog Devices <DEVNAME>
clock generator.

Signed-off-by: Your Name <your.name@analog.com>
```

For Kconfig/Makefile changes bundled with the driver, a single commit
is typical.  If the binding is a separate YAML file, send it as a
separate patch in the series.
