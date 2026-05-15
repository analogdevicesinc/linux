# Linux GPIO Driver Template

Reference drivers:
- I2C GPIO expander: `drivers/gpio/gpio-pca953x.c`
- SPI digital I/O: `drivers/gpio/gpio-max3191x.c`
- DT binding: `Documentation/devicetree/bindings/gpio/gpio-pca95xx.yaml`

This template covers every file needed to add a new GPIO expander or
SPI digital I/O driver to the Linux kernel GPIO subsystem. Replace
`<devname>` with the chip name (e.g., `pca953x`) and `<DEVNAME>` with
its uppercase form (e.g., `PCA953X`) throughout.

---

## 1. Purpose & Subsystem Mapping

GPIO expander and SPI digital I/O drivers sit in `drivers/gpio/` and
register with the GPIOLIB subsystem through `struct gpio_chip`. Two
primary device categories are covered:

- **I2C GPIO expanders** (e.g., PCA953x, PCA957x, PCAL6524, TCA6424,
  MAX7310) -- multi-port I/O expanders accessed over I2C/SMBus. These
  expose configurable input/output pins with optional interrupt support.

- **SPI digital I/O devices** (e.g., MAX3191x, 74x164, MAX7301) --
  industrial serializers and shift registers that provide digital inputs
  or outputs over SPI. Some are input-only (serializer) or output-only
  (shift register).

Core kernel structures involved:

| Structure | Header | Purpose |
|---|---|---|
| `struct gpio_chip` | `<linux/gpio/driver.h>` | Represents a GPIO controller; carries callback pointers for direction, get, set, etc. |
| `struct irq_chip` | `<linux/irq.h>` | Represents an interrupt controller; carries mask/unmask/set_type callbacks. |
| `struct gpio_irq_chip` | `<linux/gpio/driver.h>` | Embedded in `gpio_chip` to integrate IRQ support directly with the GPIO controller. |
| `struct regmap` | `<linux/regmap.h>` | Register abstraction for I2C or SPI register access with optional caching. |

Registration flow:
1. Allocate and populate `gpio_chip` with callbacks.
2. Optionally set up `gpio_chip.irq` (the embedded `gpio_irq_chip`).
3. Call `devm_gpiochip_add_data()` to register with GPIOLIB.

---

## 2. File Checklist

```
drivers/gpio/
    gpio-<devname>.c                        # Driver source

drivers/gpio/Kconfig                        # Add GPIO_<DEVNAME> entry
drivers/gpio/Makefile                       # Add obj-$(CONFIG_...) line

Documentation/devicetree/bindings/gpio/
    gpio-<devname>.yaml                     # DT binding schema

include/linux/platform_data/<devname>.h     # Platform data (optional, legacy)
```

---

## 3. Devicetree Binding (`.yaml`)

Create `Documentation/devicetree/bindings/gpio/gpio-<devname>.yaml`:

```yaml
# SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)
%YAML 1.2
---
$id: http://devicetree.org/schemas/gpio/gpio-<devname>.yaml#
$schema: http://devicetree.org/meta-schemas/core.yaml#

title: <Vendor> <DEVNAME> GPIO Expander

maintainers:
  - Your Name <your.name@analog.com>

description: |
  The <DEVNAME> is an I2C/SPI GPIO expander providing <N> configurable
  I/O pins with optional interrupt support.

properties:
  compatible:
    enum:
      - <vendor>,<devname>
      # Add variants as needed

  reg:
    maxItems: 1

  gpio-controller: true

  '#gpio-cells':
    const: 2
    description:
      The first cell is the GPIO line number and the second cell
      encodes GPIO flags (e.g., GPIO_ACTIVE_LOW).

  gpio-line-names:
    minItems: 1
    maxItems: <N>   # number of GPIO lines

  interrupts:
    maxItems: 1

  interrupt-controller: true

  '#interrupt-cells':
    const: 2
    description:
      The first cell is the GPIO line number and the second cell
      encodes the IRQ trigger type.

  reset-gpios:
    maxItems: 1
    description: GPIO connected to the active-low RESET pin.

  vcc-supply:
    description: Power supply regulator for the device.

  ngpios:
    minimum: 1
    maximum: <N>
    description:
      Number of GPIO lines to use if fewer than the full <N> are
      connected.

patternProperties:
  "^(hog-[0-9]+|.+-hog(-[0-9]+)?)$":
    type: object
    required:
      - gpio-hog

required:
  - compatible
  - reg
  - gpio-controller
  - "#gpio-cells"

additionalProperties: false

examples:
  - |
    #include <dt-bindings/gpio/gpio.h>
    #include <dt-bindings/interrupt-controller/irq.h>

    i2c {
        #address-cells = <1>;
        #size-cells = <0>;

        gpio@20 {
            compatible = "<vendor>,<devname>";
            reg = <0x20>;
            gpio-controller;
            #gpio-cells = <2>;
            interrupt-parent = <&gpio3>;
            interrupts = <23 IRQ_TYPE_LEVEL_LOW>;
            interrupt-controller;
            #interrupt-cells = <2>;
            vcc-supply = <&vcc_3v3>;
            gpio-line-names = "led0", "led1", "btn0", "btn1",
                              "relay0", "relay1", "spare0", "spare1";
        };
    };
```

Key DT properties for GPIO controllers:
- `gpio-controller` -- marks the node as a GPIO provider.
- `#gpio-cells = <2>` -- standard for most controllers (line number + flags).
- `interrupt-controller` -- marks the node as an IRQ provider (when IRQ is supported).
- `#interrupt-cells = <2>` -- standard for GPIO IRQ controllers (line + trigger type).

---

## 4. Kconfig

Add an entry under the appropriate menu section in `drivers/gpio/Kconfig`.
I2C expanders go under `menu "I2C GPIO expanders"`, SPI devices go under
`menu "SPI GPIO expanders"`.

### I2C GPIO Expander

```kconfig
config GPIO_<DEVNAME>
	tristate "<DEVNAME> I2C GPIO expander"
	depends on I2C
	select REGMAP_I2C
	help
	  Say yes here to provide GPIO access to the <DEVNAME> I2C
	  GPIO expander with <N> I/O ports. The driver uses the
	  GPIOLIB framework and supports direction control, input
	  read, and output set.

	  This driver can also be built as a module. If so, the module
	  will be called gpio-<devname>.

config GPIO_<DEVNAME>_IRQ
	bool "Interrupt controller support for <DEVNAME>"
	depends on GPIO_<DEVNAME>
	select GPIOLIB_IRQCHIP
	help
	  Say yes here to enable the <DEVNAME> to be used as an
	  interrupt controller. Each GPIO line can generate an
	  interrupt on rising/falling edge or level changes.
```

### SPI Digital I/O

```kconfig
config GPIO_<DEVNAME>
	tristate "<DEVNAME> SPI digital I/O"
	depends on SPI_MASTER
	select REGMAP_SPI       # if register-based access is used
	help
	  GPIO driver for the <DEVNAME> SPI digital I/O device.
	  Provides <N> digital inputs (or outputs) accessible via
	  SPI. Multiple chips can be daisy-chained.

	  This driver can also be built as a module. If so, the module
	  will be called gpio-<devname>.
```

---

## 5. Makefile

Add one line to `drivers/gpio/Makefile`, maintaining alphabetical order:

```makefile
obj-$(CONFIG_GPIO_<DEVNAME>)		+= gpio-<devname>.o
```

---

## 6. Driver Source (`gpio-<devname>.c`)

### 6.1 File Header

```c
// SPDX-License-Identifier: GPL-2.0-only
/*
 * gpio-<devname>.c - GPIO driver for <Vendor> <DEVNAME>
 *
 * Copyright (C) YYYY Your Name / Company
 */
```

### 6.2 Includes

```c
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/gpio/driver.h>
#include <linux/i2c.h>           /* for I2C expanders */
/* #include <linux/spi/spi.h> */ /* for SPI devices */
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
```

### 6.3 Register Definitions

```c
#define <DEVNAME>_REG_INPUT      0x00
#define <DEVNAME>_REG_OUTPUT     0x01
#define <DEVNAME>_REG_POLARITY   0x02
#define <DEVNAME>_REG_CONFIG     0x03   /* 1 = input, 0 = output */

#define <DEVNAME>_NGPIO          8      /* GPIOs per bank */
```

### 6.4 Device Structure

```c
struct <devname>_chip {
	struct regmap *regmap;
	struct gpio_chip gpio_chip;
	struct mutex lock;

	struct i2c_client *client;      /* for I2C */
	/* struct spi_device *spi; */   /* for SPI */

#ifdef CONFIG_GPIO_<DEVNAME>_IRQ
	struct mutex irq_lock;
	u8 irq_mask;
	u8 irq_trig_raise;
	u8 irq_trig_fall;
	u8 irq_stat;
#endif
};
```

### 6.5 regmap Configuration

```c
static const struct regmap_config <devname>_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x03,
	.cache_type = REGCACHE_MAPLE,
};
```

### 6.6 gpio_chip Callbacks

Every GPIO driver must implement a subset of the `gpio_chip` callbacks.
The minimum set is `get`, `set`, `direction_input`, and `direction_output`.

```c
static int <devname>_gpio_get_direction(struct gpio_chip *gc, unsigned int off)
{
	struct <devname>_chip *chip = gpiochip_get_data(gc);
	unsigned int reg_val;
	int ret;

	guard(mutex)(&chip->lock);
	ret = regmap_read(chip->regmap, <DEVNAME>_REG_CONFIG, &reg_val);
	if (ret < 0)
		return ret;

	if (reg_val & BIT(off))
		return GPIO_LINE_DIRECTION_IN;

	return GPIO_LINE_DIRECTION_OUT;
}

static int <devname>_gpio_direction_input(struct gpio_chip *gc, unsigned int off)
{
	struct <devname>_chip *chip = gpiochip_get_data(gc);

	guard(mutex)(&chip->lock);

	/* Set direction bit to 1 = input */
	return regmap_update_bits(chip->regmap, <DEVNAME>_REG_CONFIG,
				  BIT(off), BIT(off));
}

static int <devname>_gpio_direction_output(struct gpio_chip *gc,
					   unsigned int off, int val)
{
	struct <devname>_chip *chip = gpiochip_get_data(gc);
	int ret;

	guard(mutex)(&chip->lock);

	/* Set output level first */
	ret = regmap_update_bits(chip->regmap, <DEVNAME>_REG_OUTPUT,
				 BIT(off), val ? BIT(off) : 0);
	if (ret)
		return ret;

	/* Then set direction to output (bit = 0) */
	return regmap_update_bits(chip->regmap, <DEVNAME>_REG_CONFIG,
				  BIT(off), 0);
}

static int <devname>_gpio_get_value(struct gpio_chip *gc, unsigned int off)
{
	struct <devname>_chip *chip = gpiochip_get_data(gc);
	unsigned int reg_val;
	int ret;

	scoped_guard(mutex, &chip->lock)
		ret = regmap_read(chip->regmap, <DEVNAME>_REG_INPUT, &reg_val);
	if (ret < 0)
		return ret;

	return !!(reg_val & BIT(off));
}

static void <devname>_gpio_set_value(struct gpio_chip *gc, unsigned int off,
				     int val)
{
	struct <devname>_chip *chip = gpiochip_get_data(gc);

	guard(mutex)(&chip->lock);

	regmap_update_bits(chip->regmap, <DEVNAME>_REG_OUTPUT,
			   BIT(off), val ? BIT(off) : 0);
}
```

For multi-port get/set (optional but recommended for bulk operations):

```c
static int <devname>_gpio_get_multiple(struct gpio_chip *gc,
				       unsigned long *mask,
				       unsigned long *bits)
{
	struct <devname>_chip *chip = gpiochip_get_data(gc);
	unsigned int reg_val;
	int ret;

	scoped_guard(mutex, &chip->lock)
		ret = regmap_read(chip->regmap, <DEVNAME>_REG_INPUT, &reg_val);
	if (ret)
		return ret;

	*bits = reg_val & *mask;

	return 0;
}

static void <devname>_gpio_set_multiple(struct gpio_chip *gc,
					unsigned long *mask,
					unsigned long *bits)
{
	struct <devname>_chip *chip = gpiochip_get_data(gc);

	guard(mutex)(&chip->lock);

	regmap_update_bits(chip->regmap, <DEVNAME>_REG_OUTPUT,
			   (unsigned int)*mask, (unsigned int)*bits);
}
```

### 6.7 gpio_chip Setup

```c
static void <devname>_setup_gpio(struct <devname>_chip *chip)
{
	struct gpio_chip *gc = &chip->gpio_chip;

	gc->direction_input  = <devname>_gpio_direction_input;
	gc->direction_output = <devname>_gpio_direction_output;
	gc->get              = <devname>_gpio_get_value;
	gc->set              = <devname>_gpio_set_value;
	gc->get_direction    = <devname>_gpio_get_direction;
	gc->get_multiple     = <devname>_gpio_get_multiple;
	gc->set_multiple     = <devname>_gpio_set_multiple;
	gc->can_sleep        = true;    /* I2C/SPI access may sleep */
	gc->base             = -1;      /* dynamic base allocation */
	gc->ngpio            = <DEVNAME>_NGPIO;
	gc->label            = dev_name(&chip->client->dev);
	gc->parent           = &chip->client->dev;
	gc->owner            = THIS_MODULE;
}
```

Note: `can_sleep = true` is required for any GPIO controller behind a
sleeping bus (I2C, SPI). This tells GPIOLIB consumers that `get`/`set`
may block.

### 6.8 Probe Function (I2C)

```c
static int <devname>_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct <devname>_chip *chip;
	int ret;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;
	i2c_set_clientdata(client, chip);
	mutex_init(&chip->lock);

	chip->regmap = devm_regmap_init_i2c(client, &<devname>_regmap_config);
	if (IS_ERR(chip->regmap))
		return dev_err_probe(dev, PTR_ERR(chip->regmap),
				     "failed to init regmap\n");

	<devname>_setup_gpio(chip);

	/* Optional: IRQ setup -- see Section 7 */
	ret = <devname>_irq_setup(chip);
	if (ret)
		return ret;

	return devm_gpiochip_add_data(dev, &chip->gpio_chip, chip);
}
```

### 6.9 Probe Function (SPI)

```c
static int <devname>_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct <devname>_chip *chip;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->spi = spi;
	spi_set_drvdata(spi, chip);
	mutex_init(&chip->lock);

	chip->regmap = devm_regmap_init_spi(spi, &<devname>_regmap_config);
	if (IS_ERR(chip->regmap))
		return dev_err_probe(dev, PTR_ERR(chip->regmap),
				     "failed to init regmap\n");

	<devname>_setup_gpio(chip);

	chip->gpio_chip.parent = dev;
	chip->gpio_chip.label = dev_name(dev);

	return devm_gpiochip_add_data(dev, &chip->gpio_chip, chip);
}
```

### 6.10 Match Tables and Module Registration (I2C)

```c
static const struct of_device_id <devname>_dt_ids[] = {
	{ .compatible = "<vendor>,<devname>", },
	{ }
};
MODULE_DEVICE_TABLE(of, <devname>_dt_ids);

static const struct i2c_device_id <devname>_id[] = {
	{ "<devname>" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, <devname>_id);

static struct i2c_driver <devname>_driver = {
	.driver = {
		.name		= "<devname>",
		.of_match_table	= <devname>_dt_ids,
	},
	.probe    = <devname>_probe,
	.id_table = <devname>_id,
};
module_i2c_driver(<devname>_driver);
```

### 6.11 Match Tables and Module Registration (SPI)

```c
static const struct of_device_id <devname>_dt_ids[] = {
	{ .compatible = "<vendor>,<devname>", },
	{ }
};
MODULE_DEVICE_TABLE(of, <devname>_dt_ids);

static const struct spi_device_id <devname>_spi_id[] = {
	{ "<devname>" },
	{ }
};
MODULE_DEVICE_TABLE(spi, <devname>_spi_id);

static struct spi_driver <devname>_driver = {
	.driver = {
		.name		= "<devname>",
		.of_match_table	= <devname>_dt_ids,
	},
	.probe    = <devname>_probe,
	.id_table = <devname>_spi_id,
};
module_spi_driver(<devname>_driver);
```

### 6.12 Module Footer

```c
MODULE_AUTHOR("Your Name <your.name@analog.com>");
MODULE_DESCRIPTION("GPIO driver for <Vendor> <DEVNAME>");
MODULE_LICENSE("GPL");
```

---

## 7. IRQ Support

Interrupt support is gated behind a separate Kconfig symbol
(`GPIO_<DEVNAME>_IRQ`) and requires `select GPIOLIB_IRQCHIP`.

### 7.1 irq_chip Callbacks

```c
#ifdef CONFIG_GPIO_<DEVNAME>_IRQ

static void <devname>_irq_mask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct <devname>_chip *chip = gpiochip_get_data(gc);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);

	clear_bit(hwirq, (unsigned long *)&chip->irq_mask);
	gpiochip_disable_irq(gc, hwirq);
}

static void <devname>_irq_unmask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct <devname>_chip *chip = gpiochip_get_data(gc);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);

	gpiochip_enable_irq(gc, hwirq);
	set_bit(hwirq, (unsigned long *)&chip->irq_mask);
}

static int <devname>_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct <devname>_chip *chip = gpiochip_get_data(gc);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);

	if (!(type & IRQ_TYPE_EDGE_BOTH)) {
		dev_err(gc->parent, "irq %u: unsupported type %u\n",
			d->irq, type);
		return -EINVAL;
	}

	assign_bit(hwirq, (unsigned long *)&chip->irq_trig_fall,
		   type & IRQ_TYPE_EDGE_FALLING);
	assign_bit(hwirq, (unsigned long *)&chip->irq_trig_raise,
		   type & IRQ_TYPE_EDGE_RISING);

	return 0;
}

static void <devname>_irq_bus_lock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct <devname>_chip *chip = gpiochip_get_data(gc);

	mutex_lock(&chip->irq_lock);
}

static void <devname>_irq_bus_sync_unlock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct <devname>_chip *chip = gpiochip_get_data(gc);

	/* Write the current irq_mask to the device's interrupt mask register */
	guard(mutex)(&chip->lock);
	/* regmap_write(chip->regmap, <DEVNAME>_REG_INT_MASK, ~chip->irq_mask); */

	mutex_unlock(&chip->irq_lock);
}
```

### 7.2 irq_chip Structure

Use `IRQCHIP_IMMUTABLE` and `GPIOCHIP_IRQ_RESOURCE_HELPERS` (modern pattern):

```c
static const struct irq_chip <devname>_irq_chip = {
	.irq_mask            = <devname>_irq_mask,
	.irq_unmask          = <devname>_irq_unmask,
	.irq_set_type        = <devname>_irq_set_type,
	.irq_bus_lock        = <devname>_irq_bus_lock,
	.irq_bus_sync_unlock = <devname>_irq_bus_sync_unlock,
	.flags               = IRQCHIP_IMMUTABLE,
	GPIOCHIP_IRQ_RESOURCE_HELPERS,
};
```

### 7.3 Threaded IRQ Handler

```c
static irqreturn_t <devname>_irq_handler(int irq, void *devid)
{
	struct <devname>_chip *chip = devid;
	struct gpio_chip *gc = &chip->gpio_chip;
	unsigned int pending;
	int level;

	/* Read interrupt status register */
	scoped_guard(mutex, &chip->lock) {
		/* Determine which lines changed -- compare current input
		 * against cached state, masked by irq_mask */
	}

	for_each_set_bit(level, (unsigned long *)&pending, gc->ngpio) {
		int nested_irq = irq_find_mapping(gc->irq.domain, level);

		if (unlikely(nested_irq <= 0))
			continue;
		handle_nested_irq(nested_irq);
	}

	return IRQ_RETVAL(pending);
}
```

### 7.4 IRQ Setup in Probe

Wire the `gpio_irq_chip` into `gpio_chip` before calling
`devm_gpiochip_add_data()`:

```c
static int <devname>_irq_setup(struct <devname>_chip *chip)
{
	struct i2c_client *client = chip->client;
	struct device *dev = &client->dev;
	struct gpio_irq_chip *girq;
	int ret;

	if (!client->irq)
		return 0;

	mutex_init(&chip->irq_lock);

	girq = &chip->gpio_chip.irq;
	gpio_irq_chip_set_chip(girq, &<devname>_irq_chip);
	girq->parent_handler = NULL;
	girq->num_parents = 0;
	girq->parents = NULL;
	girq->default_type = IRQ_TYPE_NONE;
	girq->handler = handle_simple_irq;
	girq->threaded = true;

	ret = devm_request_threaded_irq(dev, client->irq, NULL,
					<devname>_irq_handler,
					IRQF_ONESHOT | IRQF_SHARED,
					dev_name(dev), chip);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request irq\n");

	return 0;
}

#else /* CONFIG_GPIO_<DEVNAME>_IRQ */

static int <devname>_irq_setup(struct <devname>_chip *chip)
{
	return 0;
}

#endif
```

IRQ flow handler selection:
- `handle_simple_irq` -- default for GPIO expanders behind sleeping buses (threaded only).
- `handle_level_irq` -- for chips with hardware level-triggered interrupt status.
- `handle_edge_irq` -- for chips with hardware edge detection and latch registers.

---

## 8. Devicetree Parsing

### 8.1 Standard GPIO Properties

GPIOLIB automatically handles several standard DT properties when
`devm_gpiochip_add_data()` is called:

- `gpio-line-names` -- assigns debug names to each GPIO line. GPIOLIB
  reads this from the `fwnode` automatically; no driver code needed.
- `ngpios` -- if the driver reads this property, it can limit the
  number of active lines. Use `device_property_read_u32()`:

```c
u32 ngpios;

if (!device_property_read_u32(dev, "ngpios", &ngpios))
	gc->ngpio = ngpios;
```

### 8.2 GPIO Hogs

GPIO hogs are child nodes of the GPIO controller that automatically
configure pin direction and level at probe time. GPIOLIB handles them
automatically after `devm_gpiochip_add_data()`. Example DT:

```dts
gpio@20 {
    compatible = "<vendor>,<devname>";
    reg = <0x20>;
    gpio-controller;
    #gpio-cells = <2>;

    led-hog {
        gpio-hog;
        gpios = <0 GPIO_ACTIVE_HIGH>;
        output-high;
        line-name = "status-led";
    };
};
```

### 8.3 Driver-Specific Properties

For device-specific DT properties, use the `device_property_*` API:

```c
/* Read optional reset GPIO */
struct gpio_desc *reset_gpio;

reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
if (IS_ERR(reset_gpio))
	return PTR_ERR(reset_gpio);

/* Read optional power supply */
struct regulator *reg;

reg = devm_regulator_get(dev, "vcc");
if (IS_ERR(reg))
	return dev_err_probe(dev, PTR_ERR(reg), "failed to get vcc\n");
```

---

## 9. Test & Debug

### 9.1 gpioinfo / libgpiod Tools

After the driver loads, use `libgpiod` userspace tools to inspect and
test GPIO lines:

```sh
# List all GPIO controllers and their lines
gpioinfo

# Read a specific GPIO line value
gpioget <gpiochip> <line>

# Set a GPIO line value
gpioset <gpiochip> <line>=<value>

# Monitor GPIO events (interrupts)
gpiomon <gpiochip> <line>

# Detailed chip info
gpiodetect
```

### 9.2 sysfs (legacy, deprecated)

The sysfs GPIO interface is deprecated but may still be available:

```sh
# Export a GPIO
echo <N> > /sys/class/gpio/export

# Set direction and value
echo out > /sys/class/gpio/gpio<N>/direction
echo 1 > /sys/class/gpio/gpio<N>/value

# Read value
cat /sys/class/gpio/gpio<N>/value
```

### 9.3 debugfs

GPIO debug information is available via debugfs:

```sh
cat /sys/kernel/debug/gpio
```

This shows all registered `gpiochip` instances, their base numbers,
number of lines, and current pin states.

### 9.4 Interrupt Testing

If the device supports interrupts, verify IRQ setup:

```sh
# Check registered interrupts
cat /proc/interrupts | grep <devname>

# Monitor GPIO events via libgpiod
gpiomon --falling-edge <gpiochip> <line>
```

### 9.5 regmap debugfs

When using regmap, register values can be inspected at runtime:

```sh
cat /sys/kernel/debug/regmap/<i2c-addr>/registers
```

---

## 10. Key Conventions

1. **License** -- use `GPL-2.0-only` SPDX identifier. The `MODULE_LICENSE`
   macro must be `"GPL"`.

2. **GPIOLIB patterns** -- always use `devm_gpiochip_add_data()` (not the
   non-devm variant). Pass driver-private data via the `data` parameter,
   retrieve it in callbacks with `gpiochip_get_data()`.

3. **devm_* allocation** -- use `devm_kzalloc()`, `devm_regmap_init_i2c()`,
   `devm_request_threaded_irq()`, and other managed resource functions.
   This avoids the need for explicit cleanup on driver removal.

4. **Sleeping context** -- set `gc->can_sleep = true` for any GPIO
   controller behind I2C or SPI. Callers must not use `gpio_get_value()`
   from atomic context; they must use `gpiod_get_value_cansleep()`.

5. **Locking** -- use a per-device mutex to serialise register access.
   For IRQ callbacks that run in bus-lock context, use a separate
   `irq_lock` mutex with `irq_bus_lock` / `irq_bus_sync_unlock`.

6. **Immutable irq_chip** -- declare `struct irq_chip` as `const` with
   `IRQCHIP_IMMUTABLE` flag and use `GPIOCHIP_IRQ_RESOURCE_HELPERS`.
   Call `gpiochip_enable_irq()` / `gpiochip_disable_irq()` in
   unmask/mask callbacks.

7. **regmap** -- prefer `regmap` over raw I2C/SPI transfer functions.
   Use `regmap_read()`, `regmap_write()`, `regmap_update_bits()`, and
   `regmap_bulk_read()` / `regmap_bulk_write()` for multi-register access.
   Enable register caching (`REGCACHE_MAPLE` or `REGCACHE_RBTREE`) for
   output and direction registers.

8. **dev_err_probe()** -- use `dev_err_probe()` instead of `dev_err()`
   followed by `return ret` in probe functions. This handles `-EPROBE_DEFER`
   correctly by logging at debug level instead of error level.

9. **guard() / scoped_guard()** -- use the cleanup.h scoped lock macros
   (`guard(mutex)` and `scoped_guard(mutex, ...)`) for cleaner lock
   management.

10. **Module macros** -- use `module_i2c_driver()` or `module_spi_driver()`
    to avoid boilerplate `__init` / `__exit` functions. For GPIO expanders
    that must probe early (e.g., to control an I2C mux), use
    `subsys_initcall()` instead.

---

## 11. Commit Message Format

All commits touching `drivers/gpio/` must use the `gpio:` prefix.
Subsystem maintainer conventions:

```
gpio: <devname>: add support for <Vendor> <DEVNAME>

Add GPIO expander driver for the <DEVNAME>, an I2C-controlled <N>-bit
GPIO expander supporting configurable I/O direction, interrupt
generation, and polarity inversion.

The driver registers a gpio_chip with GPIOLIB and optionally provides
irq_chip integration for interrupt-capable variants.

Signed-off-by: Your Name <your.name@analog.com>
```

For DT binding additions:

```
dt-bindings: gpio: add <Vendor> <DEVNAME> binding

Document the devicetree binding for the <DEVNAME> I2C GPIO expander
in YAML schema format.

Signed-off-by: Your Name <your.name@analog.com>
```

For follow-up patches (e.g., adding IRQ support):

```
gpio: <devname>: add interrupt controller support

Add irq_chip integration to the <DEVNAME> driver using the
gpio_irq_chip framework. Each GPIO line can generate an interrupt
on rising or falling edge.

Signed-off-by: Your Name <your.name@analog.com>
```
