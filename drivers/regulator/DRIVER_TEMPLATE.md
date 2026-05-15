# Linux Regulator Driver Template

Reference drivers:
- `drivers/regulator/adp5055-regulator.c` (ADP5055 triple buck, I2C, regmap helpers)
- `drivers/regulator/ltc3589.c` (LTC3589 8-output, I2C, IRQ, multi-ops)

This template covers the files and patterns needed to add a Linux kernel
regulator driver for a voltage regulator, LDO, or DC-DC converter such as
ADP5360, ADP5055, LT3650, or LTC3589.  Replace `<devname>` with the part
number (e.g., `adp5055`) and `<DEVNAME>` with its uppercase form (e.g.,
`ADP5055`) throughout.

---

## 1. Purpose & Subsystem Mapping

Regulator drivers live under `drivers/regulator/` and plug into the Linux
regulator framework via two core structures:

| Structure | Role |
|---|---|
| `struct regulator_ops` | Callbacks: list_voltage, set/get voltage, enable/disable, set mode |
| `struct regulator_desc` | Static description: name, type, n_voltages, min_uV, uV_step, vsel_reg/mask, enable_reg/mask, ops pointer |

The driver registers each output with `devm_regulator_register()`.
Consumer drivers then call `regulator_get()`, `regulator_enable()`,
`regulator_set_voltage()`, etc. against the regulators.

Typical ADI parts in this subsystem:
- **DC-DC buck converters** -- ADP5055, ADP5054, LTC3589, LTC3676
- **LDOs** -- ADP150, ADP1740, LT3045
- **Battery chargers with regulator outputs** -- ADP5360, LT3650

---

## 2. File Checklist

```
drivers/regulator/
    <devname>-regulator.c           # Driver source (or just <devname>.c)

drivers/regulator/Kconfig           # Add REGULATOR_<DEVNAME> entry
drivers/regulator/Makefile          # Add obj-$(CONFIG_REGULATOR_<DEVNAME>) line

Documentation/devicetree/bindings/regulator/
    adi,<devname>-regulator.yaml    # Devicetree binding
```

---

## 3. Devicetree Binding (.yaml)

File: `Documentation/devicetree/bindings/regulator/adi,<devname>-regulator.yaml`

```yaml
# SPDX-License-Identifier: (GPL-2.0 OR BSD-2-Clause)
%YAML 1.2
---
$id: http://devicetree.org/schemas/regulator/adi,<devname>-regulator.yaml#
$schema: http://devicetree.org/meta-schemas/core.yaml#

title: Analog Devices <DEVNAME> Voltage Regulator

maintainers:
  - Your Name <your.name@analog.com>

description: |
  The <DEVNAME> is a <brief description, e.g., triple-output buck regulator
  with I2C interface>.
  https://www.analog.com/media/en/technical-documentation/data-sheets/<devname>.pdf

properties:
  compatible:
    enum:
      - adi,<devname>

  reg:
    maxItems: 1

  # Device-specific properties (adi,<property-name>):
  # adi,some-feature:
  #   description: ...
  #   type: boolean

patternProperties:
  # One entry per regulator output.  Adjust the regex to match the
  # output names (e.g., "^buck[0-2]$", "^ldo[1-4]$").
  '^buck[0-2]$':
    type: object
    $ref: regulator.yaml#
    unevaluatedProperties: false

    properties:
      regulator-name:
        description: Name of the regulator output.

      # Standard regulator properties (inherited from regulator.yaml):
      #   regulator-min-microvolt
      #   regulator-max-microvolt
      #   regulator-always-on
      #   regulator-boot-on
      #   regulator-allow-bypass
      #   regulator-ramp-delay

    required:
      - regulator-name

required:
  - compatible
  - reg

additionalProperties: false

examples:
  - |
    i2c {
        #address-cells = <1>;
        #size-cells = <0>;

        regulator@70 {
            compatible = "adi,<devname>";
            reg = <0x70>;

            buck0 {
                regulator-name = "buck0";
                regulator-min-microvolt = <800000>;
                regulator-max-microvolt = <1800000>;
            };
        };
    };
```

Key regulator DT properties (defined in `regulator.yaml` base schema):

| Property | Description |
|---|---|
| `regulator-min-microvolt` | Minimum allowed voltage |
| `regulator-max-microvolt` | Maximum allowed voltage |
| `regulator-always-on` | Regulator must never be disabled |
| `regulator-boot-on` | Regulator is enabled at boot |
| `regulator-ramp-delay` | Voltage ramp rate in uV/us |
| `regulator-allow-bypass` | Allow bypass mode |

---

## 4. Kconfig

Add to `drivers/regulator/Kconfig` (alphabetical order within the `if REGULATOR` block):

```kconfig
config REGULATOR_<DEVNAME>
	tristate "Analog Devices <DEVNAME> Voltage Regulator"
	depends on I2C
	select REGMAP_I2C
	help
	  This driver supports the Analog Devices <DEVNAME>, a <short
	  description, e.g., triple-output step-down regulator> controlled
	  via I2C.

	  Say M here if you want to include support for the regulator as a
	  module. The module will be named <devname>-regulator.
```

For SPI-connected parts, replace `depends on I2C` / `select REGMAP_I2C` with
`depends on SPI` / `select REGMAP_SPI`.

---

## 5. Makefile

Add to `drivers/regulator/Makefile` (alphabetical order):

```makefile
obj-$(CONFIG_REGULATOR_<DEVNAME>)	+= <devname>-regulator.o
```

---

## 6. Driver Source (.c)

File: `drivers/regulator/<devname>-regulator.c`

### 6.1 Header & Includes

```c
// SPDX-License-Identifier: GPL-2.0
//
// Regulator driver for Analog Devices <DEVNAME>
//
// Copyright (C) 20XX Analog Devices, Inc.

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
```

### 6.2 Register Map & Masks

```c
/* Register addresses */
#define <DEVNAME>_VOUT_REG      0x01
#define <DEVNAME>_ENABLE_REG    0x02

/* Field masks */
#define <DEVNAME>_VSEL_MASK     GENMASK(7, 0)
#define <DEVNAME>_EN_MASK       BIT(0)
```

### 6.3 Regmap Configuration

```c
static const struct regmap_config <devname>_regmap_config = {
	.reg_bits  = 8,
	.val_bits  = 8,
	.max_register = 0xFF,
};
```

### 6.4 regulator_ops

Use regmap helper functions wherever possible.  The framework provides
ready-made implementations for the most common operations:

| Helper | What it does |
|---|---|
| `regulator_list_voltage_linear` | Linear voltage list from min_uV + n * uV_step |
| `regulator_list_voltage_linear_range` | Multiple linear ranges |
| `regulator_list_voltage_table` | Voltage from explicit table |
| `regulator_map_voltage_linear` | Map requested voltage to selector (linear) |
| `regulator_map_voltage_linear_range` | Map voltage to selector (ranges) |
| `regulator_set_voltage_sel_regmap` | Write selector to vsel_reg/mask |
| `regulator_get_voltage_sel_regmap` | Read selector from vsel_reg/mask |
| `regulator_enable_regmap` | Set enable_reg/mask |
| `regulator_disable_regmap` | Clear enable_reg/mask |
| `regulator_is_enabled_regmap` | Read enable_reg/mask |
| `regulator_set_active_discharge_regmap` | Set active discharge via regmap |
| `regulator_set_ramp_delay_regmap` | Set ramp delay via regmap |

```c
static const struct regulator_ops <devname>_ops = {
	.list_voltage    = regulator_list_voltage_linear,
	.map_voltage     = regulator_map_voltage_linear,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.enable          = regulator_enable_regmap,
	.disable         = regulator_disable_regmap,
	.is_enabled      = regulator_is_enabled_regmap,
};
```

For fixed-voltage outputs (LDOs at a fixed rail), ops can be minimal:

```c
static const struct regulator_ops <devname>_fixed_ops = {
	.enable     = regulator_enable_regmap,
	.disable    = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
};
```

Custom operations (e.g., set_mode) are implemented directly:

```c
static int <devname>_set_mode(struct regulator_dev *rdev, unsigned int mode)
{
	struct <devname> *priv = rdev_get_drvdata(rdev);

	switch (mode) {
	case REGULATOR_MODE_NORMAL:
		return regmap_update_bits(priv->regmap, ...);
	case REGULATOR_MODE_IDLE:
		return regmap_update_bits(priv->regmap, ...);
	default:
		return -EINVAL;
	}
}
```

### 6.5 regulator_desc

The `regulator_desc` structure ties everything together.  Fill in
the fields that match the hardware; the framework uses these to drive
the regmap helper ops automatically.

```c
static const struct regulator_desc <devname>_regulators[] = {
	{
		.name        = "buck0",
		.of_match    = of_match_ptr("buck0"),
		.id          = 0,
		.type        = REGULATOR_VOLTAGE,
		.owner       = THIS_MODULE,
		.ops         = &<devname>_ops,
		.n_voltages  = 256,
		.min_uV      = 800000,
		.uV_step     = 10000,
		.vsel_reg    = <DEVNAME>_VOUT_REG,
		.vsel_mask   = <DEVNAME>_VSEL_MASK,
		.enable_reg  = <DEVNAME>_ENABLE_REG,
		.enable_mask = <DEVNAME>_EN_MASK,
	},
};
```

Key `regulator_desc` fields:

| Field | Description |
|---|---|
| `name` | Human-readable name |
| `of_match` | DT node name to match (under regulators node) |
| `regulators_node` | Parent DT node name, e.g., "regulators" (if sub-node grouping is used) |
| `of_parse_cb` | Per-regulator DT parsing callback |
| `id` | Numeric ID (index into array) |
| `type` | `REGULATOR_VOLTAGE` or `REGULATOR_CURRENT` |
| `n_voltages` | Number of voltage selectors |
| `min_uV` | Minimum output voltage |
| `uV_step` | Voltage step between selectors |
| `linear_ranges` | Array of `struct linear_range` for multi-range devices |
| `n_linear_ranges` | Number of linear ranges |
| `volt_table` | Explicit voltage table (for non-linear parts) |
| `vsel_reg` / `vsel_mask` | Register and mask for voltage selector |
| `enable_reg` / `enable_mask` | Register and mask for enable bit |
| `apply_reg` / `apply_bit` | Register and bit to latch voltage changes (GO bit) |
| `active_discharge_reg` / `active_discharge_mask` | Active discharge control |
| `ramp_reg` / `ramp_mask` / `ramp_delay_table` | Ramp delay control |

For devices with many similar outputs, use a macro to reduce boilerplate
(see ADP5055 and LTC3589 for examples):

```c
#define <DEVNAME>_REG(_name, _id) \
	[_id] = { \
		.name       = _name, \
		.of_match   = of_match_ptr(_name), \
		.id         = _id, \
		.ops        = &<devname>_ops, \
		.type       = REGULATOR_VOLTAGE, \
		.owner      = THIS_MODULE, \
		.n_voltages = 256, \
		.min_uV     = 800000, \
		.uV_step    = 10000, \
		.vsel_reg   = <DEVNAME>_VOUT0 + (_id), \
		.vsel_mask  = GENMASK(7, 0), \
		.enable_reg = <DEVNAME>_EN_REG, \
		.enable_mask = BIT(_id), \
	}

static const struct regulator_desc <devname>_regulators[] = {
	<DEVNAME>_REG("buck0", 0),
	<DEVNAME>_REG("buck1", 1),
	<DEVNAME>_REG("buck2", 2),
};
```

### 6.6 Probe Function

```c
static int <devname>_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct regmap *regmap;
	int i;

	regmap = devm_regmap_init_i2c(client, &<devname>_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap),
				     "Failed to init regmap\n");

	for (i = 0; i < ARRAY_SIZE(<devname>_regulators); i++) {
		struct regulator_config config = { };
		struct regulator_dev *rdev;

		config.dev = dev;
		config.regmap = regmap;

		rdev = devm_regulator_register(dev,
					       &<devname>_regulators[i],
					       &config);
		if (IS_ERR(rdev))
			return dev_err_probe(dev, PTR_ERR(rdev),
					     "Failed to register %s\n",
					     <devname>_regulators[i].name);
	}

	return 0;
}
```

If the driver carries private state, allocate it and pass it via
`config.driver_data`:

```c
struct <devname> {
	struct device *dev;
	struct regmap *regmap;
};

/* In probe: */
struct <devname> *priv;

priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
if (!priv)
	return -ENOMEM;

priv->dev = dev;
priv->regmap = regmap;
config.driver_data = priv;

/* In ops callbacks: */
struct <devname> *priv = rdev_get_drvdata(rdev);
```

### 6.7 Driver Registration

```c
static const struct of_device_id <devname>_of_match[] = {
	{ .compatible = "adi,<devname>" },
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
		.name           = "<devname>",
		.of_match_table = <devname>_of_match,
	},
	.probe    = <devname>_probe,
	.id_table = <devname>_ids,
};
module_i2c_driver(<devname>_driver);

MODULE_DESCRIPTION("<DEVNAME> Voltage Regulator Driver");
MODULE_AUTHOR("Your Name <your.name@analog.com>");
MODULE_LICENSE("GPL");
```

---

## 7. Multi-Regulator Support

Many regulator ICs provide multiple outputs from a single device (e.g.,
ADP5055 has 3 bucks, LTC3589 has 8 outputs).  Patterns to handle this:

- **One `regulator_desc` per output** -- define an array indexed by output ID.
- **Loop in probe** -- iterate over all descriptors and call
  `devm_regulator_register()` for each.
- **Macro-generated descriptors** -- use a `#define` macro to stamp out
  per-channel descriptors with per-channel register offsets.
- **`regulator_init_data`** -- when passed via `config.init_data`, applies
  constraints (min/max voltage, valid ops) to a specific output.
- **`regulators_node`** -- set to `"regulators"` in `regulator_desc` if
  the DT groups all outputs under a `regulators { }` sub-node.

Example from LTC3589 showing mixed output types (linear, fixed, table):

```c
static const struct regulator_desc ltc3589_regulators[] = {
	LTC3589_LINEAR_REG(SW1, sw1, B1DTV1),   /* Linear buck */
	LTC3589_LINEAR_REG(SW2, sw2, B2DTV1),
	LTC3589_LINEAR_REG(SW3, sw3, B3DTV1),
	LTC3589_FIXED_REG(BB_OUT, bb-out),      /* Fixed output */
	LTC3589_REG(LDO1, ldo1, fixed_standby, 0, 0, 0),
	LTC3589_LINEAR_REG(LDO2, ldo2, L2DTV1), /* Linear LDO */
	LTC3589_FIXED_REG(LDO3, ldo3),
	LTC3589_REG(LDO4, ldo4, table, ...),    /* Table-based LDO */
};
```

---

## 8. Devicetree Parsing

### 8.1 of_parse_cb (Per-Regulator Callback)

Use `of_parse_cb` in `regulator_desc` to parse per-regulator DT properties
that go beyond the standard regulator binding.  The framework calls this
once per regulator node during registration.

```c
static int <devname>_of_parse_cb(struct device_node *np,
				 const struct regulator_desc *desc,
				 struct regulator_config *config)
{
	struct <devname> *priv = config->driver_data;
	int id = desc->id;

	/* Parse custom per-regulator properties */
	if (of_property_read_bool(np, "enable-gpios")) {
		config->ena_gpiod = devm_fwnode_gpiod_get(config->dev,
					of_fwnode_handle(np), "enable",
					GPIOD_OUT_LOW, "enable");
		if (IS_ERR(config->ena_gpiod))
			return PTR_ERR(config->ena_gpiod);
	}

	return 0;
}
```

### 8.2 Device-Level DT Parsing

For properties that apply to the whole device (not per-regulator), parse
them in probe or in a dedicated `parse_fw()` helper:

```c
static int <devname>_parse_fw(struct device *dev, struct <devname> *priv)
{
	/* device_property_read_*() works for both DT and ACPI */
	device_property_read_u32(dev, "adi,some-property", &priv->some_val);

	priv->some_flag = device_property_read_bool(dev, "adi,some-flag");

	return 0;
}
```

### 8.3 Legacy Helpers

These are available but less commonly needed in modern drivers:
- `of_regulator_match()` -- match DT regulator sub-nodes by name.
- `of_get_regulator_init_data()` -- extract `regulator_init_data` from DT.
  Useful when constraints must be applied before registration.

---

## 9. Test & Debug

### 9.1 sysfs Interface

Each registered regulator appears under `/sys/class/regulator/`.  Useful
attributes:

```
/sys/class/regulator/regulator.N/
    name            # Regulator name
    state           # "enabled" or "disabled"
    microvolts      # Current output voltage in uV
    min_microvolts  # Minimum allowed voltage
    max_microvolts  # Maximum allowed voltage
    num_users       # Number of consumer references
    type            # "voltage" or "current"
```

### 9.2 debugfs

With `CONFIG_REGULATOR_DEBUG` enabled:

```
/sys/kernel/debug/regulator/
    <regulator_name>/
        use_count
        open_count
        bypass_count
        consumers
```

### 9.3 Kernel Tracing

The regulator subsystem has tracepoints:

```
echo 1 > /sys/kernel/debug/tracing/events/regulator/enable
cat /sys/kernel/debug/tracing/trace
```

### 9.4 Quick Validation

```bash
# List all registered regulators
ls /sys/class/regulator/

# Check a specific regulator
cat /sys/class/regulator/regulator.0/name
cat /sys/class/regulator/regulator.0/state
cat /sys/class/regulator/regulator.0/microvolts

# Verify DT binding
make dt_binding_check DT_SCHEMA_FILES=Documentation/devicetree/bindings/regulator/adi,<devname>-regulator.yaml

# Validate DTS against binding
make dtbs_check DT_SCHEMA_FILES=Documentation/devicetree/bindings/regulator/adi,<devname>-regulator.yaml
```

---

## 10. Key Conventions

- **License**: Use `GPL-2.0` (required for regulator subsystem).  The
  SPDX identifier is `// SPDX-License-Identifier: GPL-2.0`.

- **Regmap helpers**: Prefer `regulator_*_regmap` helper functions over
  hand-rolled register I/O.  Fill in the `vsel_reg`, `vsel_mask`,
  `enable_reg`, `enable_mask` fields in `regulator_desc` and the
  framework does the rest.

- **devm_ allocation**: Always use `devm_regulator_register()`,
  `devm_regmap_init_i2c()`, `devm_kzalloc()`.  No manual cleanup needed
  in remove or error paths.

- **Microvolts**: All voltage values in the kernel API and DT bindings are
  in microvolts (uV).  Field names use `_uV` or `_microvolt` suffixes.

- **dev_err_probe()**: Use instead of `dev_err()` + return in probe paths.
  Handles `-EPROBE_DEFER` transparently.

- **Linear ranges**: For devices with multiple voltage ranges, use
  `REGULATOR_LINEAR_RANGE()` macro and the `linear_ranges` /
  `n_linear_ranges` fields.

- **Voltage tables**: For non-linear voltage mappings, use `volt_table`
  with `regulator_list_voltage_table` / `regulator_set_voltage_sel_regmap`.

- **Naming**: Driver file is `<devname>-regulator.c` or `<devname>.c`.
  Kconfig symbol is `REGULATOR_<DEVNAME>`.  Compatible string is
  `"adi,<devname>"`.

---

## 11. Commit Message Format

Use the `regulator:` prefix for all commits in `drivers/regulator/`:

```
regulator: <devname>: add support for <DEVNAME>

Add driver for the Analog Devices <DEVNAME>, a <brief description>.

The driver supports <list key features: voltage control, enable/disable,
multiple outputs, etc.>.

Signed-off-by: Your Name <your.name@analog.com>
```

For DT binding commits:

```
dt-bindings: regulator: add Analog Devices <DEVNAME>

Add devicetree binding documentation for the Analog Devices <DEVNAME>
voltage regulator.

Signed-off-by: Your Name <your.name@analog.com>
```

For follow-up fixes or features:

```
regulator: <devname>: fix <what is fixed>

<Explain the problem and the fix.>

Fixes: <sha> ("<original commit title>")
Signed-off-by: Your Name <your.name@analog.com>
```
