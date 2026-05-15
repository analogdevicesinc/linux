# Linux Hardware Monitoring (hwmon) Driver Template

This template covers every file needed to add a new hwmon driver to the
Linux kernel.  Replace `<devname>` with the part number (e.g., `ltc2945`)
and `<DEVNAME>` with its uppercase form (e.g., `LTC2945`) throughout.

ADI examples: ADT7475 (temperature/fan), LTC2945 (power monitor),
ADM1177 (hot-swap controller), ADE7753 (energy meter).

---

## 1. Purpose & Subsystem Mapping

The hwmon subsystem exposes hardware monitoring data (temperature,
voltage, current, power, fan speed) through a unified sysfs interface
under `/sys/class/hwmon/`.

Key kernel structures:

| Structure | Purpose |
|---|---|
| `struct hwmon_chip_info` | Top-level descriptor: links ops + channel info |
| `struct hwmon_ops` | Callbacks: `is_visible`, `read`, `write` |
| `struct hwmon_channel_info` | Per-type channel config (temp, in, curr, power, fan) |

The registration function is:

```c
devm_hwmon_device_register_with_info(dev, name, drvdata,
                                     &chip_info, extra_groups);
```

Typical ADI parts that map to hwmon:

- **Power/energy meters**: LTC2945, LTC2947, LTC4245, LTC4282, ADE7753
- **Voltage/current monitors**: ADM1177, ADM1025, ADM1026, ADM9240
- **Temperature monitors**: ADT7475, ADT7410, ADT7310, ADT7462, ADT7470
- **Fan controllers**: ADT7475, ADT7462, ADT7470

---

## 2. File Checklist

```
drivers/hwmon/
    <devname>.c                   # Driver implementation

Documentation/devicetree/bindings/hwmon/
    adi,<devname>.yaml            # DT binding (YAML schema)

Documentation/hwmon/
    <devname>.rst                 # Kernel hwmon documentation
```

For the build system, two files need edits (not new files):

```
drivers/hwmon/Kconfig             # Add SENSORS_<DEVNAME> entry
drivers/hwmon/Makefile            # Add obj-$(CONFIG_SENSORS_<DEVNAME>)
```

If the device supports both I2C and SPI, split into separate files:

```
drivers/hwmon/
    <devname>-core.c              # Shared logic (regmap-based)
    <devname>-i2c.c               # I2C-specific probe + regmap init
    <devname>-spi.c               # SPI-specific probe + regmap init
```

---

## 3. Devicetree Binding (.yaml)

File: `Documentation/devicetree/bindings/hwmon/adi,<devname>.yaml`

```yaml
# SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)
%YAML 1.2
---
$id: http://devicetree.org/schemas/hwmon/adi,<devname>.yaml#
$schema: http://devicetree.org/meta-schemas/core.yaml#

title: Analog Devices <DEVNAME> <Short Description>

maintainers:
  - Your Name <your.name@analog.com>

description: |
  Analog Devices <DEVNAME> <description>.
  https://www.analog.com/media/en/technical-documentation/data-sheets/<DEVNAME>.pdf

properties:
  compatible:
    enum:
      - adi,<devname>

  reg:
    maxItems: 1

  # For I2C devices, no additional bus properties needed.
  # For SPI devices, add spi-max-frequency etc.

  shunt-resistor-micro-ohms:
    description:
      The value of the current sense resistor in microohms.
      Required for current and power measurements.

  # Vendor-specific properties use the "adi," prefix:
  adi,vrange-high-enable:
    description:
      Enable the higher input voltage range.
    type: boolean

required:
  - compatible
  - reg

allOf:
  - $ref: hwmon-common.yaml#

unevaluatedProperties: false

examples:
  - |
    i2c {
        #address-cells = <1>;
        #size-cells = <0>;

        monitor@5a {
            compatible = "adi,<devname>";
            reg = <0x5a>;
            shunt-resistor-micro-ohms = <50000>; /* 50 mOhm */
        };
    };
...
```

Validate the binding with:

```
make dt_binding_check DT_SCHEMA_FILES=Documentation/devicetree/bindings/hwmon/adi,<devname>.yaml
```

---

## 4. Kconfig

Add to `drivers/hwmon/Kconfig` (entries are alphabetically sorted):

```kconfig
config SENSORS_<DEVNAME>
	tristate "Analog Devices <DEVNAME>"
	depends on I2C
	select REGMAP_I2C
	help
	  If you say yes here you get support for Analog Devices <DEVNAME>
	  <short description, e.g., "I2C Power Monitor">.

	  This driver can also be built as a module. If so, the module
	  will be called <devname>.
```

Notes:
- Use `depends on I2C` or `depends on SPI_MASTER` (or both with `||`).
- Use `select REGMAP_I2C` or `select REGMAP_SPI` to pull in regmap.
- For devices needing voltage ID translation, add `select HWMON_VID`.

---

## 5. Makefile

Add to `drivers/hwmon/Makefile` (alphabetically sorted):

```makefile
obj-$(CONFIG_SENSORS_<DEVNAME>)	+= <devname>.o
```

For multi-file drivers (core + bus modules):

```makefile
obj-$(CONFIG_SENSORS_<DEVNAME>)	+= <devname>-core.o
obj-$(CONFIG_SENSORS_<DEVNAME>)	+= <devname>-i2c.o
obj-$(CONFIG_SENSORS_<DEVNAME>)	+= <devname>-spi.o
```

---

## 6. Driver Source (.c)

File: `drivers/hwmon/<devname>.c`

This shows the modern `devm_hwmon_device_register_with_info()` API
(preferred over the legacy `SENSOR_DEVICE_ATTR` sysfs approach).

```c
// SPDX-License-Identifier: GPL-2.0
/*
 * <DEVNAME> - <Short description>
 *
 * Copyright YYYY Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/device.h>
#include <linux/hwmon.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>

/* ---------- Register Definitions ----------------------------------- */

#define <DEVNAME>_REG_STATUS		0x00
#define <DEVNAME>_REG_CONFIG		0x01
#define <DEVNAME>_REG_VOLTAGE_H		0x02
#define <DEVNAME>_REG_CURRENT_H		0x04
#define <DEVNAME>_REG_TEMP_H		0x06

/* ---------- Private Data ------------------------------------------- */

/**
 * struct <devname>_data - device instance data
 * @regmap:           Register map handle
 * @shunt_resistor:   Sense resistor in micro-ohms
 */
struct <devname>_data {
	struct regmap *regmap;
	u32 shunt_resistor;
};

/* ---------- hwmon_ops callbacks ------------------------------------- */

static int <devname>_read(struct device *dev, enum hwmon_sensor_types type,
			  u32 attr, int channel, long *val)
{
	struct <devname>_data *data = dev_get_drvdata(dev);
	unsigned int regval;
	int ret;

	switch (type) {
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_input:
			ret = regmap_read(data->regmap,
					  <DEVNAME>_REG_TEMP_H, &regval);
			if (ret)
				return ret;
			/* Convert to millidegrees Celsius */
			*val = sign_extend32(regval, 15) * 1000 / 256;
			return 0;
		default:
			return -EOPNOTSUPP;
		}
	case hwmon_in:
		switch (attr) {
		case hwmon_in_input:
			ret = regmap_read(data->regmap,
					  <DEVNAME>_REG_VOLTAGE_H, &regval);
			if (ret)
				return ret;
			/* Convert to millivolts */
			*val = regval * 25;
			return 0;
		default:
			return -EOPNOTSUPP;
		}
	case hwmon_curr:
		switch (attr) {
		case hwmon_curr_input:
			ret = regmap_read(data->regmap,
					  <DEVNAME>_REG_CURRENT_H, &regval);
			if (ret)
				return ret;
			/* Convert to milliamps */
			*val = regval * 25 * 1000 / data->shunt_resistor;
			return 0;
		default:
			return -EOPNOTSUPP;
		}
	default:
		return -EOPNOTSUPP;
	}
}

static int <devname>_write(struct device *dev, enum hwmon_sensor_types type,
			   u32 attr, int channel, long val)
{
	struct <devname>_data *data = dev_get_drvdata(dev);

	switch (type) {
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_max:
			val = clamp_val(val, -128000, 127000);
			return regmap_write(data->regmap,
					    <DEVNAME>_REG_TEMP_MAX,
					    val * 256 / 1000);
		default:
			return -EOPNOTSUPP;
		}
	default:
		return -EOPNOTSUPP;
	}
}

static umode_t <devname>_is_visible(const void *data,
				    enum hwmon_sensor_types type,
				    u32 attr, int channel)
{
	const struct <devname>_data *st = data;

	switch (type) {
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_input:
			return 0444;
		case hwmon_temp_max:
		case hwmon_temp_max_alarm:
			return 0644;
		default:
			return 0;
		}
	case hwmon_in:
		switch (attr) {
		case hwmon_in_input:
			return 0444;
		default:
			return 0;
		}
	case hwmon_curr:
		switch (attr) {
		case hwmon_curr_input:
			return st->shunt_resistor ? 0444 : 0;
		default:
			return 0;
		}
	default:
		return 0;
	}
}

/* ---------- Channel Configuration ---------------------------------- */

static const struct hwmon_channel_info * const <devname>_info[] = {
	HWMON_CHANNEL_INFO(temp,
			   HWMON_T_INPUT | HWMON_T_MAX | HWMON_T_MAX_ALARM),
	HWMON_CHANNEL_INFO(in,
			   HWMON_I_INPUT),
	HWMON_CHANNEL_INFO(curr,
			   HWMON_C_INPUT),
	NULL
};

static const struct hwmon_ops <devname>_hwmon_ops = {
	.is_visible = <devname>_is_visible,
	.read = <devname>_read,
	.write = <devname>_write,
};

static const struct hwmon_chip_info <devname>_chip_info = {
	.ops = &<devname>_hwmon_ops,
	.info = <devname>_info,
};

/* ---------- regmap configuration ----------------------------------- */

static const struct regmap_config <devname>_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x3F,
};

/* ---------- Probe -------------------------------------------------- */

static int <devname>_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct <devname>_data *data;
	struct device *hwmon_dev;
	struct regmap *regmap;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	regmap = devm_regmap_init_i2c(client, &<devname>_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap),
				     "failed to init regmap\n");

	data->regmap = regmap;

	if (device_property_read_u32(dev, "shunt-resistor-micro-ohms",
				     &data->shunt_resistor))
		data->shunt_resistor = 1000; /* default 1 mOhm */

	if (data->shunt_resistor == 0)
		return -EINVAL;

	hwmon_dev = devm_hwmon_device_register_with_info(dev, client->name,
							 data,
							 &<devname>_chip_info,
							 NULL);
	return PTR_ERR_OR_ZERO(hwmon_dev);
}

/* ---------- Device tables ------------------------------------------ */

static const struct of_device_id <devname>_of_match[] = {
	{ .compatible = "adi,<devname>" },
	{ }
};
MODULE_DEVICE_TABLE(of, <devname>_of_match);

static const struct i2c_device_id <devname>_id[] = {
	{ "<devname>" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, <devname>_id);

static struct i2c_driver <devname>_driver = {
	.driver = {
		.name = "<devname>",
		.of_match_table = <devname>_of_match,
	},
	.probe = <devname>_probe,
	.id_table = <devname>_id,
};
module_i2c_driver(<devname>_driver);

MODULE_AUTHOR("Your Name <your.name@analog.com>");
MODULE_DESCRIPTION("Analog Devices <DEVNAME> hwmon driver");
MODULE_LICENSE("GPL");
```

---

## 7. Channel Configuration

Channel info arrays use the `HWMON_CHANNEL_INFO()` macro.  Each entry
declares one or more channels of a given type with a bitmask of
supported attributes.

### Available channel types and common attributes

| Type | Enum | Typical attributes |
|---|---|---|
| Temperature | `hwmon_temp` | `HWMON_T_INPUT`, `HWMON_T_MAX`, `HWMON_T_MIN`, `HWMON_T_CRIT`, `HWMON_T_MAX_ALARM`, `HWMON_T_LABEL` |
| Voltage | `hwmon_in` | `HWMON_I_INPUT`, `HWMON_I_MIN`, `HWMON_I_MAX`, `HWMON_I_MIN_ALARM`, `HWMON_I_MAX_ALARM`, `HWMON_I_LABEL` |
| Current | `hwmon_curr` | `HWMON_C_INPUT`, `HWMON_C_MIN`, `HWMON_C_MAX`, `HWMON_C_MIN_ALARM`, `HWMON_C_MAX_ALARM` |
| Power | `hwmon_power` | `HWMON_P_INPUT`, `HWMON_P_MAX`, `HWMON_P_MIN`, `HWMON_P_MAX_ALARM` |
| Fan speed | `hwmon_fan` | `HWMON_F_INPUT`, `HWMON_F_MIN`, `HWMON_F_TARGET`, `HWMON_F_MIN_ALARM` |

### Multiple channels of the same type

List one attribute bitmask per physical channel:

```c
static const struct hwmon_channel_info * const <devname>_info[] = {
	HWMON_CHANNEL_INFO(in,
			   HWMON_I_INPUT | HWMON_I_MIN | HWMON_I_MAX,   /* in0 (VIN) */
			   HWMON_I_INPUT),                               /* in1 (ADIN) */
	HWMON_CHANNEL_INFO(temp,
			   HWMON_T_INPUT,    /* temp1 - internal */
			   HWMON_T_INPUT),   /* temp2 - remote */
	NULL
};
```

### Dynamic channel visibility with is_visible()

Use `is_visible()` to hide channels at runtime based on hardware
configuration (e.g., hide current channel if no sense resistor):

```c
static umode_t <devname>_is_visible(const void *data,
				    enum hwmon_sensor_types type,
				    u32 attr, int channel)
{
	const struct <devname>_data *st = data;

	switch (type) {
	case hwmon_curr:
		/* Only show current if sense resistor is present */
		if (!st->shunt_resistor)
			return 0;
		return 0444;
	case hwmon_in:
		return 0444;
	default:
		return 0;
	}
}
```

Return values: `0444` (read-only), `0644` (read-write), `0` (hidden).

---

## 8. Devicetree Parsing

Parse device-specific configuration in `probe()` using the device
property API.  Common patterns:

```c
/* Integer property with default fallback */
if (device_property_read_u32(dev, "shunt-resistor-micro-ohms",
                             &data->shunt_resistor))
	data->shunt_resistor = 1000; /* default: 1 mOhm */

/* Boolean property */
data->vrange_high = device_property_read_bool(dev,
                                              "adi,vrange-high-enable");

/* Integer property with vendor prefix */
if (device_property_read_u32(dev, "adi,shutdown-threshold-microamp",
                             &alert_threshold_ua))
	alert_threshold_ua = 0;
```

Naming conventions for DT properties:
- Standard properties (defined in `hwmon-common.yaml`): no prefix
  (e.g., `shunt-resistor-micro-ohms`).
- Vendor-specific properties: use `adi,` prefix
  (e.g., `adi,vrange-high-enable`).
- Use SI units with micro/milli suffixes in property names
  (e.g., `-micro-ohms`, `-microamp`).

---

## 9. Test & Debug

### Build and load

```bash
# Build as module
make M=drivers/hwmon modules

# Load
insmod <devname>.ko

# Or via Kconfig: enable CONFIG_SENSORS_<DEVNAME>=m and rebuild
```

### lm-sensors tools

```bash
# Detect attached sensors
sudo sensors-detect

# Read all sensor values
sensors

# Read specific chip
sensors <devname>-i2c-*-*
```

### sysfs interface

All hwmon devices appear under `/sys/class/hwmon/hwmonN/`:

```bash
# Find the device
cat /sys/class/hwmon/hwmon*/name
# Look for "<devname>"

# Read values
cat /sys/class/hwmon/hwmon0/temp1_input    # millidegrees C
cat /sys/class/hwmon/hwmon0/in1_input      # millivolts
cat /sys/class/hwmon/hwmon0/curr1_input    # milliamps
cat /sys/class/hwmon/hwmon0/power1_input   # microwatts

# Read/write thresholds
cat /sys/class/hwmon/hwmon0/temp1_max
echo 85000 > /sys/class/hwmon/hwmon0/temp1_max
```

### DT overlay testing

```bash
# Compile overlay
dtc -@ -I dts -O dtb -o <devname>.dtbo <devname>-overlay.dts

# Apply (on systems with configfs)
mkdir /sys/kernel/config/device-tree/overlays/<devname>
cat <devname>.dtbo > /sys/kernel/config/device-tree/overlays/<devname>/dtbo
```

### Debugging

```bash
# Check driver loaded and bound
ls /sys/bus/i2c/drivers/<devname>/

# Check kernel log
dmesg | grep <devname>

# Regmap debugfs (if CONFIG_REGMAP enabled with debugfs)
cat /sys/kernel/debug/regmap/*-00<addr>/registers
```

---

## 10. Key Conventions

1. **License**: `GPL-2.0` (or `GPL-2.0-or-later`).  Use the
   SPDX identifier on line 1: `// SPDX-License-Identifier: GPL-2.0`.

2. **Unit conventions** -- the hwmon sysfs interface uses fixed units:
   - Temperature: **millidegrees Celsius** (e.g., 25000 = 25.0 C)
   - Voltage: **millivolts** (e.g., 3300 = 3.3 V)
   - Current: **milliamps** (e.g., 500 = 0.5 A)
   - Power: **microwatts** (e.g., 1650000 = 1.65 W)
   - Fan speed: **RPM**

3. **Managed resources** -- use `devm_*` functions everywhere:
   - `devm_kzalloc()` for private data
   - `devm_regmap_init_i2c()` / `devm_regmap_init_spi()` for regmap
   - `devm_hwmon_device_register_with_info()` for hwmon registration
   - `devm_regulator_get_enable()` for power supplies
   - No need for a `remove()` callback when all resources are managed.

4. **regmap** -- always use regmap for register access.  It provides
   caching, locking, and bus abstraction:
   ```c
   static const struct regmap_config <devname>_regmap_config = {
       .reg_bits = 8,
       .val_bits = 8,          /* or 16 for 16-bit registers */
       .max_register = 0x3F,
   };
   ```

5. **Error reporting** -- use `dev_err_probe()` in `probe()` for clean
   deferred-probe handling:
   ```c
   if (IS_ERR(regmap))
       return dev_err_probe(dev, PTR_ERR(regmap),
                            "failed to init regmap\n");
   ```

6. **of_match_table** -- do NOT wrap in `of_match_ptr()` for new
   drivers.  The modern convention is to always include the table
   unconditionally to support ACPI-based matching via PRP0001.

7. **module_i2c_driver() / module_spi_driver()** -- use the
   boilerplate-reduction macros instead of manually writing
   `module_init()` / `module_exit()`.

8. **Naming**: driver name matches the compatible string minus the
   vendor prefix (e.g., `"adi,ltc2945"` -> driver name `"ltc2945"`).

9. **clamp_val()** -- use when bounding user-supplied threshold
   values before writing to hardware registers.

---

## 11. Commit Message Format

hwmon driver patches use the `hwmon:` prefix, optionally followed by
the part-specific prefix:

```
hwmon: (<devname>) Add support for <DEVNAME>

Add hwmon driver for the Analog Devices <DEVNAME>, a <short description,
e.g., "wide-range I2C power monitor">. The driver reports voltage,
current, and power via the standard hwmon sysfs interface.

Signed-off-by: Your Name <your.name@analog.com>
```

For DT binding patches:

```
dt-bindings: hwmon: Add Analog Devices <DEVNAME>

Add devicetree binding documentation for the Analog Devices <DEVNAME>
<short description>.

Signed-off-by: Your Name <your.name@analog.com>
```

A typical patch series order:
1. `dt-bindings: hwmon: Add Analog Devices <DEVNAME>`
2. `hwmon: (<devname>) Add support for <DEVNAME>`
3. (Optional) `MAINTAINERS: add <devname> hwmon driver`
