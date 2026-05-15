# Linux LED Driver Template

Target subsystem: **LED** (`struct led_classdev`, `/sys/class/leds/`)

Reference drivers:
- `drivers/leds/leds-bd2606mvv.c` -- simple multi-channel I2C LED controller
- `drivers/leds/leds-lt3593.c` -- ADI LT3593 GPIO-based LED driver
- `drivers/leds/leds-lm3692x.c` -- I2C LED with regulator and enable GPIO
- `drivers/leds/rgb/leds-ncp5623.c` -- multi-color (led_classdev_mc) I2C driver

Typical ADI/LLTC parts: LT3593, ADP5520, ADPD1080.

Replace `<devname>` with the part number (e.g., `lt3593`) and `<DEVNAME>`
with its uppercase form (e.g., `LT3593`) throughout.

---

## 1. Purpose & Subsystem Mapping

The Linux LED subsystem provides a unified interface for LED controllers
through `struct led_classdev`. Each LED channel is registered as a class
device under `/sys/class/leds/<name>/`, exposing `brightness`,
`max_brightness`, and optional `trigger` attributes.

| no-OS concept | Linux LED subsystem equivalent |
|---|---|
| `<devname>_dev` | `struct led_classdev` (one per LED channel) |
| `<devname>_set_current()` | `.brightness_set_blocking` callback |
| `<devname>_init()` | `probe()` with `devm_led_classdev_register()` |
| `<devname>_remove()` | Managed cleanup (devm) or explicit `remove()` |
| Per-channel enable | Writing `0` or non-zero to `brightness` sysfs |
| IIO current channel | Not used -- brightness is the native LED interface |

LED controllers like the LT3593 or ADP5520 map naturally: each physical
LED output becomes a `led_classdev` instance. Multi-color LEDs (e.g.,
RGB) use `struct led_classdev_mc` from the multicolor class instead.

---

## 2. File Checklist

```
drivers/leds/leds-<devname>.c                          # Driver source
drivers/leds/Kconfig                                   # Add LEDS_<DEVNAME> entry
drivers/leds/Makefile                                  # Add obj-$(CONFIG_LEDS_<DEVNAME>)
Documentation/devicetree/bindings/leds/leds-<devname>.yaml  # DT binding
```

For multi-color (RGB) drivers, place the source in `drivers/leds/rgb/`
and update `drivers/leds/rgb/Kconfig` and `drivers/leds/rgb/Makefile`
instead.

---

## 3. Devicetree Binding (`.yaml`)

The LED controller node follows the `led-controller` convention. Each
physical LED output is a child node with `reg`, `color`, `function`,
and optional `linux,default-trigger`.

```yaml
# SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)
%YAML 1.2
---
$id: http://devicetree.org/schemas/leds/leds-<devname>.yaml#
$schema: http://devicetree.org/meta-schemas/core.yaml#

title: <VENDOR> <DEVNAME> LED Controller

maintainers:
  - Your Name <your.name@analog.com>

description: |
  The <DEVNAME> is an N-channel LED controller with I2C interface.
  It provides programmable current sinks for independent LED control.

properties:
  compatible:
    const: <vendor>,<devname>

  reg:
    maxItems: 1

  "#address-cells":
    const: 1

  "#size-cells":
    const: 0

patternProperties:
  "^led@[0-9a-f]+$":
    type: object
    $ref: common.yaml#
    unevaluatedProperties: false

    properties:
      reg:
        description: LED output channel index.
        minimum: 0
        maximum: <N-1>

      color:
        description: |
          LED color. Use LED_COLOR_ID_* from
          include/dt-bindings/leds/common.h.

      function:
        description: |
          LED function. Use LED_FUNCTION_* from
          include/dt-bindings/leds/common.h.

      linux,default-trigger:
        description: Trigger assigned to this LED at boot.

    required:
      - reg

required:
  - compatible
  - reg
  - "#address-cells"
  - "#size-cells"

additionalProperties: false

examples:
  - |
    #include <dt-bindings/leds/common.h>

    i2c {
        #address-cells = <1>;
        #size-cells = <0>;

        led-controller@33 {
            compatible = "<vendor>,<devname>";
            reg = <0x33>;
            #address-cells = <1>;
            #size-cells = <0>;

            led@0 {
                reg = <0>;
                color = <LED_COLOR_ID_RED>;
                function = LED_FUNCTION_INDICATOR;
                linux,default-trigger = "heartbeat";
            };

            led@1 {
                reg = <1>;
                color = <LED_COLOR_ID_GREEN>;
                function = LED_FUNCTION_STATUS;
            };
        };
    };
...
```

Key binding properties from `common.yaml`:
- `color` -- integer from `LED_COLOR_ID_*` (include/dt-bindings/leds/common.h)
- `function` -- string from `LED_FUNCTION_*` (same header)
- `function-enumerator` -- integer disambiguator when multiple LEDs share the same function
- `label` -- deprecated in favor of `function` + `color`
- `default-state` -- `"on"`, `"off"`, or `"keep"` (default: `"off"`)
- `linux,default-trigger` -- e.g., `"heartbeat"`, `"default-on"`, `"pattern"`
- `led-max-microamp` -- maximum LED current in microamps

---

## 4. Kconfig

Add an entry to `drivers/leds/Kconfig` (keep alphabetical order):

```kconfig
config LEDS_<DEVNAME>
	tristate "LED driver for <DEVNAME>"
	depends on LEDS_CLASS
	depends on I2C
	select REGMAP_I2C
	help
	  This option enables support for the <VENDOR> <DEVNAME>
	  LED controller connected via I2C. It provides N independently
	  controllable LED outputs with programmable current sinks.

	  To compile this driver as a module, choose M here: the module
	  will be called leds-<devname>.
```

For SPI-connected devices, replace `depends on I2C` / `select REGMAP_I2C`
with `depends on SPI` / `select REGMAP_SPI`. For GPIO-only controllers
(like LT3593), use `depends on GPIOLIB || COMPILE_TEST` and drop the
regmap select.

---

## 5. Makefile

Add to `drivers/leds/Makefile` (keep alphabetical order within the
platform drivers section):

```makefile
obj-$(CONFIG_LEDS_<DEVNAME>)		+= leds-<devname>.o
```

For SPI-based drivers, add to the `# LED SPI Drivers` section instead.

---

## 6. Driver Source (`.c`)

### 6.1 Minimal I2C LED driver skeleton

```c
// SPDX-License-Identifier: GPL-2.0
/*
 * <DEVNAME> LED driver
 *
 * Copyright (C) YYYY Analog Devices, Inc.
 */

#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/property.h>
#include <linux/regmap.h>

#define <DEVNAME>_MAX_LEDS		N
#define <DEVNAME>_MAX_BRIGHTNESS	255

/* Register addresses */
#define <DEVNAME>_REG_ENABLE		0x00
#define <DEVNAME>_REG_BRIGHTNESS(ch)	(0x01 + (ch))

#define ldev_to_led(c) container_of(c, struct <devname>_led, ldev)

struct <devname>_led {
	unsigned int		led_no;
	struct led_classdev	ldev;
	struct <devname>_priv	*priv;
};

struct <devname>_priv {
	struct <devname>_led	leds[<DEVNAME>_MAX_LEDS];
	struct regmap		*regmap;
};

static int <devname>_brightness_set(struct led_classdev *led_cdev,
				    enum led_brightness brightness)
{
	struct <devname>_led *led = ldev_to_led(led_cdev);
	struct <devname>_priv *priv = led->priv;
	int ret;

	if (brightness == 0)
		return regmap_update_bits(priv->regmap,
					 <DEVNAME>_REG_ENABLE,
					 BIT(led->led_no), 0);

	ret = regmap_write(priv->regmap,
			   <DEVNAME>_REG_BRIGHTNESS(led->led_no),
			   brightness);
	if (ret)
		return ret;

	return regmap_update_bits(priv->regmap,
				 <DEVNAME>_REG_ENABLE,
				 BIT(led->led_no),
				 BIT(led->led_no));
}

static const struct regmap_config <devname>_regmap = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= 0x0F,
};

static int <devname>_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct <devname>_priv *priv;
	int err, reg;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->regmap = devm_regmap_init_i2c(client, &<devname>_regmap);
	if (IS_ERR(priv->regmap))
		return dev_err_probe(dev, PTR_ERR(priv->regmap),
				     "Failed to init regmap\n");

	i2c_set_clientdata(client, priv);

	device_for_each_child_node_scoped(dev, child) {
		struct <devname>_led *led;
		struct led_init_data init_data = {};

		err = fwnode_property_read_u32(child, "reg", &reg);
		if (err)
			return err;

		if (reg >= <DEVNAME>_MAX_LEDS)
			return -EINVAL;

		led = &priv->leds[reg];
		led->priv = priv;
		led->led_no = reg;
		led->ldev.brightness_set_blocking = <devname>_brightness_set;
		led->ldev.max_brightness = <DEVNAME>_MAX_BRIGHTNESS;

		init_data.fwnode = child;

		err = devm_led_classdev_register_ext(dev, &led->ldev,
						     &init_data);
		if (err)
			return dev_err_probe(dev, err,
					     "Failed to register LED %d\n",
					     reg);
	}

	return 0;
}

static const struct of_device_id <devname>_of_match[] = {
	{ .compatible = "<vendor>,<devname>" },
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
		.name		= "leds-<devname>",
		.of_match_table	= <devname>_of_match,
	},
	.probe		= <devname>_probe,
	.id_table	= <devname>_id,
};
module_i2c_driver(<devname>_driver);

MODULE_AUTHOR("Your Name <your.name@analog.com>");
MODULE_DESCRIPTION("<DEVNAME> LED driver");
MODULE_LICENSE("GPL");
```

### 6.2 Key structures and callbacks

| Structure / API | Purpose |
|---|---|
| `struct led_classdev` | Core LED abstraction; one per LED channel |
| `.brightness_set` | Non-blocking brightness callback (for GPIO-only drivers) |
| `.brightness_set_blocking` | Blocking brightness callback (for I2C/SPI -- can sleep) |
| `.max_brightness` | Maximum brightness value (hardware-dependent) |
| `struct led_init_data` | Passes DT node info to the registration function |
| `devm_led_classdev_register_ext()` | Managed registration with init_data (preferred) |
| `devm_led_classdev_register()` | Simpler managed registration (no init_data) |
| `struct regmap_config` | regmap abstraction over I2C/SPI register access |
| `devm_regmap_init_i2c()` | Create managed regmap for I2C devices |
| `devm_regmap_init_spi()` | Create managed regmap for SPI devices |

For GPIO-only devices (like LT3593), use `struct gpio_desc *` with
`devm_gpiod_get()` instead of regmap and register the driver via
`module_platform_driver()` rather than `module_i2c_driver()`.

---

## 7. Multi-LED Support

For multi-color LEDs (e.g., RGB), use `struct led_classdev_mc` from
`<linux/led-class-multicolor.h>`.

```c
#include <linux/led-class-multicolor.h>

struct <devname>_priv {
	struct i2c_client	*client;
	struct led_classdev_mc	mc_dev;
	struct mutex		lock;
};

static int <devname>_mc_brightness_set(struct led_classdev *cdev,
				       enum led_brightness brightness)
{
	struct led_classdev_mc *mc = lcdev_to_mccdev(cdev);
	struct <devname>_priv *priv = container_of(mc, struct <devname>_priv,
						   mc_dev);

	guard(mutex)(&priv->lock);

	for (int i = 0; i < mc->num_colors; i++) {
		/* Write subled_info[i].intensity to channel register */
		/* subled_info[i].channel gives the hardware channel index */
		/* subled_info[i].color_index gives the LED_COLOR_ID_* */
	}

	/* Write overall brightness */
	return 0;
}
```

The DT binding for multi-color uses a `multi-led` child node containing
per-color sub-nodes:

```dts
led-controller@33 {
    compatible = "<vendor>,<devname>";
    reg = <0x33>;
    #address-cells = <1>;
    #size-cells = <0>;

    multi-led@0 {
        reg = <0>;
        color = <LED_COLOR_ID_MULTI>;
        function = LED_FUNCTION_INDICATOR;

        led-0 {
            color = <LED_COLOR_ID_RED>;
            reg = <0>;
        };
        led-1 {
            color = <LED_COLOR_ID_GREEN>;
            reg = <1>;
        };
        led-2 {
            color = <LED_COLOR_ID_BLUE>;
            reg = <2>;
        };
    };
};
```

Registration uses `led_classdev_multicolor_register_ext()` (or
`devm_led_classdev_multicolor_register_ext()`) and requires:
- `mc_dev.subled_info` -- array of `struct mc_subled` (channel, color_index, intensity)
- `mc_dev.num_colors` -- number of sub-LEDs
- `mc_dev.led_cdev.brightness_set_blocking` -- brightness callback
- Kconfig must `select LEDS_CLASS_MULTICOLOR`

---

## 8. Devicetree Parsing

### 8.1 Iterating child LED nodes

```c
device_for_each_child_node_scoped(dev, child) {
    u32 reg;

    err = fwnode_property_read_u32(child, "reg", &reg);
    if (err)
        return err;

    if (reg >= MAX_LEDS)
        return -EINVAL;

    /* Configure led[reg] from child properties */
}
```

`device_for_each_child_node_scoped()` automatically manages the
`fwnode_handle` lifetime. Older drivers use
`fwnode_for_each_available_child_node()` with explicit
`fwnode_handle_put()` on error paths.

### 8.2 Reading default state

```c
#include <linux/leds.h>

/* In probe, after reading child node: */
enum led_default_state state;

state = led_init_default_state_get(child);
switch (state) {
case LEDS_DEFSTATE_ON:
    led->ldev.brightness = led->ldev.max_brightness;
    break;
case LEDS_DEFSTATE_KEEP:
    /* Read current HW brightness */
    break;
default:
    led->ldev.brightness = LED_OFF;
    break;
}
```

### 8.3 Common fwnode property reads

```c
u32 max_microamp;
const char *trigger;

fwnode_property_read_u32(child, "led-max-microamp", &max_microamp);
fwnode_property_read_string(child, "linux,default-trigger", &trigger);
```

When using `devm_led_classdev_register_ext()` with `init_data.fwnode`
set, the LED core automatically handles the `function`, `color`,
`function-enumerator`, and `linux,default-trigger` properties, so the
driver does not need to parse them manually.

---

## 9. Test & Debug

### 9.1 sysfs interface

Each registered LED appears at `/sys/class/leds/<name>/`:

```bash
# List all LEDs
ls /sys/class/leds/

# Read current brightness
cat /sys/class/leds/<name>/brightness

# Set brightness (0 = off, max_brightness = full on)
echo 128 > /sys/class/leds/<name>/brightness

# Read maximum brightness
cat /sys/class/leds/<name>/max_brightness

# List available triggers
cat /sys/class/leds/<name>/trigger

# Set trigger
echo heartbeat > /sys/class/leds/<name>/trigger

# Disable trigger (manual control)
echo none > /sys/class/leds/<name>/trigger
```

### 9.2 LED naming

When `devm_led_classdev_register_ext()` is used with `init_data`, the
LED name is automatically derived from the DT `function` and `color`
properties in the format `<devicename>:<color>:<function>`. For example:

```
leds-bd2606mvv:red:indicator
leds-bd2606mvv:green:status
```

### 9.3 regmap debugfs

If the driver uses regmap, register state is visible under:

```bash
ls /sys/kernel/debug/regmap/<i2c-addr>/
cat /sys/kernel/debug/regmap/<i2c-addr>/registers
```

### 9.4 Common triggers for testing

| Trigger | Behavior |
|---|---|
| `none` | Manual brightness control via sysfs |
| `default-on` | LED on at registration |
| `heartbeat` | Blinks at a rate proportional to CPU load |
| `timer` | Configurable on/off blinking (delay_on, delay_off) |
| `pattern` | Hardware or software pattern playback |
| `disk-activity` | Blinks on disk I/O |

---

## 10. Key Conventions

1. **License** -- use `// SPDX-License-Identifier: GPL-2.0` (or
   `GPL-2.0-only`). LED drivers must be GPL-compatible.

2. **LED naming** -- prefer `function` + `color` DT properties over
   the deprecated `label`. The LED core generates the sysfs name as
   `<devicename>:<color>:<function>`.

3. **devm_ everywhere** -- use managed resource APIs:
   - `devm_kzalloc()` for allocations
   - `devm_regmap_init_i2c()` / `devm_regmap_init_spi()` for regmap
   - `devm_led_classdev_register_ext()` for LED registration
   - `devm_gpiod_get()` for GPIO descriptors
   - `devm_regulator_get_optional()` for supply regulators
   With full devm usage, a `.remove()` callback is often unnecessary.

4. **Blocking vs non-blocking brightness** -- use
   `.brightness_set_blocking` for I2C/SPI drivers (they need to sleep).
   Use `.brightness_set` only for drivers that can set brightness in
   atomic context (e.g., memory-mapped GPIO).

5. **regmap for register access** -- prefer regmap over raw
   `i2c_smbus_*` calls. It provides caching, locking, and debugfs
   integration. Use `regmap_write()`, `regmap_read()`,
   `regmap_update_bits()`.

6. **Error reporting** -- use `dev_err_probe()` in probe paths. It
   handles `-EPROBE_DEFER` transparently and provides consistent
   error messages.

7. **Container macros** -- use `container_of()` to get from
   `led_classdev` to the driver-private structure. Define a helper
   macro at the top of the file:
   ```c
   #define ldev_to_led(c) container_of(c, struct <devname>_led, ldev)
   ```

8. **Module boilerplate** -- every driver must include:
   ```c
   MODULE_AUTHOR("...");
   MODULE_DESCRIPTION("...");
   MODULE_LICENSE("GPL");
   ```

9. **Match tables** -- provide both `of_device_id` (for DT) and
   `i2c_device_id` (for board files / ACPI fallback). Use
   `MODULE_DEVICE_TABLE()` for both.

10. **No `of_match_ptr()`** -- modern drivers should not wrap
    `of_match_table` with `of_match_ptr()`. Just assign the table
    directly. This avoids unused-variable warnings and supports
    compile testing without CONFIG_OF.

---

## 11. Commit Message Format

LED subsystem commits use the `leds:` prefix:

```
leds: <devname>: add support for <DEVNAME> LED controller

Add a driver for the <VENDOR> <DEVNAME> N-channel LED controller.
The device communicates over I2C and provides independently
controllable LED outputs with programmable current sinks.

Signed-off-by: Your Name <your.name@analog.com>
```

For DT binding additions, use a separate commit:

```
dt-bindings: leds: add binding for <DEVNAME>

Add devicetree binding documentation for the <VENDOR> <DEVNAME>
LED controller.

Signed-off-by: Your Name <your.name@analog.com>
```

The typical patch series order:
1. `dt-bindings: leds: add binding for <DEVNAME>`
2. `leds: <devname>: add support for <DEVNAME> LED controller`
3. `ARM: dts: <board>: add <DEVNAME> LED controller node` (if applicable)
