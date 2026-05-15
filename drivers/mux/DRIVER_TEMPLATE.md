# Linux Mux Driver Template

Reference drivers: `drivers/mux/adg792a.c`, `drivers/mux/gpio.c`,
`drivers/mux/adgs1408.c`

This template covers every file needed to add a new mux (multiplexer)
driver to the Linux kernel mux subsystem. Replace `<devname>` with the
part number (e.g., `adg792a`) and `<DEVNAME>` with its uppercase form
(e.g., `ADG792A`) throughout.

---

## 1. Purpose & Subsystem Mapping

The Linux **mux subsystem** (`drivers/mux/`) provides a generic
framework for controlling multiplexer hardware. It is built around
three core abstractions:

| Subsystem Struct      | Role                                                |
|-----------------------|-----------------------------------------------------|
| `struct mux_chip`     | Represents the physical mux chip. One chip can expose one or more mux controllers. Allocated with `mux_chip_alloc()` / `devm_mux_chip_alloc()`. |
| `struct mux_control`  | A single multiplexer controller within a chip. Each has a number of states (channels) and an idle state. |
| `struct mux_control_ops` | Operations provided by the driver. The only required callback is `.set`, which switches the mux to a given state. |

**Devices that fit this subsystem:**

- **Analog multiplexers** -- ADG792A/G (I2C triple 4:1), ADGS1408/1409
  (SPI 8:1/4:1), ADG2404 (GPIO 4:1), ADG1412 (GPIO quad SPST switch).
- **Analog switches** -- devices that connect/disconnect signal paths,
  modeled as a mux with 2 states (on/off) or with
  `MUX_IDLE_DISCONNECT`.
- **GPIO-controlled mux** -- the generic `gpio-mux` driver handles any
  multiplexer controlled by N GPIO pins encoding 2^N states.

The mux subsystem separates **providers** (mux controller drivers that
call `devm_mux_chip_register()`) from **consumers** (other drivers that
call `mux_control_select()` / `mux_control_deselect()` to route
signals).

---

## 2. File Checklist

```
drivers/mux/
    <devname>.c                                    # Driver source

include/dt-bindings/mux/
    mux.h                                          # Already exists (MUX_IDLE_*)

Documentation/devicetree/bindings/mux/
    adi,<devname>.yaml                             # DT binding schema

drivers/mux/Kconfig                                # Add MUX_<DEVNAME> entry
drivers/mux/Makefile                               # Add build rule
```

For GPIO-controlled muxes using the existing `gpio-mux` compatible,
no new driver file is needed -- the generic `drivers/mux/gpio.c`
handles them. Only register-controlled (I2C/SPI) muxes need a
dedicated `.c` file.

---

## 3. Devicetree Binding (.yaml)

Create `Documentation/devicetree/bindings/mux/adi,<devname>.yaml`:

```yaml
# SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
%YAML 1.2
---
$id: http://devicetree.org/schemas/mux/adi,<devname>.yaml#
$schema: http://devicetree.org/meta-schemas/core.yaml#

title: Analog Devices <DEVNAME> Multiplexer

maintainers:
  - Your Name <your.name@analog.com>

description: |
  The <DEVNAME> is a <description, e.g., wide bandwidth triple 4:1
  multiplexer> controlled over <I2C/SPI>.

allOf:
  - $ref: mux-controller.yaml#

properties:
  compatible:
    enum:
      - adi,<devname>
      # Add variants here, e.g.:
      # - adi,<devname_variant>

  reg:
    maxItems: 1

  '#mux-control-cells':
    enum: [0, 1]
    description: |
      Use 0 if the chip provides a single mux controller (or if all
      controllers operate in parallel). Use 1 to address individual
      mux controllers on multi-controller chips.

  idle-state:
    description: |
      State to set when the mux is idle. Special values from
      <dt-bindings/mux/mux.h>:
        MUX_IDLE_AS_IS   (-1)  leave the mux in its last state (default)
        MUX_IDLE_DISCONNECT (-2)  disconnect all paths (high-Z)
      Or a numeric state index (0 to N-1) to select a specific channel.
    $ref: /schemas/types.yaml#/definitions/int32
    minimum: -2

  idle-states:
    description: |
      Array form of idle-state, one entry per mux controller. Used for
      multi-controller chips (e.g., triple mux with #mux-control-cells=1).
    $ref: /schemas/types.yaml#/definitions/int32-array
    items:
      minimum: -2

  # For GPIO-controlled muxes only (compatible = "gpio-mux"):
  # mux-gpios:
  #   description: GPIO pins controlling the mux, LSB first.

required:
  - compatible
  - reg
  - '#mux-control-cells'

additionalProperties: false

examples:
  - |
    #include <dt-bindings/mux/mux.h>

    i2c {
        #address-cells = <1>;
        #size-cells = <0>;

        mux: mux-controller@50 {
            compatible = "adi,<devname>";
            reg = <0x50>;
            #mux-control-cells = <1>;

            idle-states = <MUX_IDLE_DISCONNECT
                           MUX_IDLE_AS_IS
                           2>;
        };
    };

  - |
    /* GPIO-controlled mux example (uses existing gpio-mux driver) */
    #include <dt-bindings/gpio/gpio.h>

    mux_gpio: mux-controller {
        compatible = "gpio-mux";
        #mux-control-cells = <0>;

        mux-gpios = <&gpio 0 GPIO_ACTIVE_HIGH>,
                    <&gpio 1 GPIO_ACTIVE_HIGH>;
        idle-state = <0>;
    };
...
```

### Key binding points

- `#mux-control-cells` = 0 means consumers reference the mux with
  `mux-controls = <&mux>` (no index). Set to 1 for multi-controller
  chips so consumers use `mux-controls = <&mux N>`.
- `#mux-state-cells` is an alternative used when the consumer wants
  to specify a fixed state: `mux-states = <&mux STATE>` (1 cell) or
  `mux-states = <&mux CONTROLLER STATE>` (2 cells).
- `MUX_IDLE_AS_IS` (-1) and `MUX_IDLE_DISCONNECT` (-2) are defined
  in `<dt-bindings/mux/mux.h>`.

---

## 4. Kconfig

Add a new entry in `drivers/mux/Kconfig` inside the
`menu "Multiplexer drivers"` block:

```kconfig
config MUX_<DEVNAME>
	tristate "Analog Devices <DEVNAME> Multiplexer"
	depends on I2C   # or SPI, or GPIOLIB
	help
	  <DEVNAME> <brief description, e.g., Wide Bandwidth Triple 4:1
	  Multiplexer>.

	  To compile the driver as a module, choose M here: the module will
	  be called mux-<devname>.
```

Notes:

- All mux drivers live under `menu "Multiplexer drivers"` which
  `depends on MULTIPLEXER`. The core module (`CONFIG_MULTIPLEXER`) is
  pulled in automatically via `select` or dependency chains.
- Use `depends on I2C` for I2C-controlled muxes, `depends on SPI`
  for SPI-controlled muxes, `depends on GPIOLIB || COMPILE_TEST` for
  GPIO-controlled muxes.

---

## 5. Makefile

Add to `drivers/mux/Makefile`:

```makefile
mux-<devname>-objs		:= <devname>.o
obj-$(CONFIG_MUX_<DEVNAME>)	+= mux-<devname>.o
```

The naming convention is `mux-<devname>` for the module name. This
matches the existing pattern:

```
mux-adg792a-objs    := adg792a.o
obj-$(CONFIG_MUX_ADG792A)   += mux-adg792a.o
```

---

## 6. Driver Source (.c)

### 6.1 I2C-controlled mux (e.g., ADG792A pattern)

```c
// SPDX-License-Identifier: GPL-2.0
/*
 * Multiplexer driver for Analog Devices <DEVNAME>
 *
 * Copyright (C) YYYY Analog Devices, Inc.
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mux/driver.h>
#include <linux/property.h>

/* Hardware command/register definitions */
#define <DEVNAME>_DISABLE	0x00
#define <DEVNAME>_MUX(state)	(((state) << 1) | 1)

/*
 * .set callback -- called by the mux core to switch to a new state.
 *
 * @mux:   The mux controller being changed.
 * @state: The new state to select (0-based index), or
 *         MUX_IDLE_DISCONNECT to disable all paths.
 *
 * Access the parent I2C/SPI device via mux->chip->dev.parent.
 */
static int <devname>_set(struct mux_control *mux, int state)
{
	struct i2c_client *i2c = to_i2c_client(mux->chip->dev.parent);
	u8 cmd;

	if (state == MUX_IDLE_DISCONNECT)
		cmd = <DEVNAME>_DISABLE;
	else
		cmd = <DEVNAME>_MUX(state);

	return i2c_smbus_write_byte_data(i2c, /* register */ 0x00, cmd);
}

static const struct mux_control_ops <devname>_ops = {
	.set = <devname>_set,
};

static int <devname>_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct mux_chip *mux_chip;
	s32 idle_state;
	u32 cells;
	int ret;

	/* Read #mux-control-cells to determine operating mode */
	ret = device_property_read_u32(dev, "#mux-control-cells", &cells);
	if (ret < 0)
		return ret;

	/*
	 * Allocate the mux chip with managed allocation.
	 *
	 * devm_mux_chip_alloc(dev, controllers, sizeof_priv)
	 *   controllers: number of independent mux controllers
	 *   sizeof_priv: extra bytes for driver private data,
	 *                retrieved later with mux_chip_priv()
	 */
	mux_chip = devm_mux_chip_alloc(dev, cells ? /* N */ 3 : 1, 0);
	if (IS_ERR(mux_chip))
		return PTR_ERR(mux_chip);

	mux_chip->ops = &<devname>_ops;

	/*
	 * Configure each mux controller:
	 *   mux->states     = number of valid states (channels)
	 *   mux->idle_state = state when idle (default MUX_IDLE_AS_IS)
	 */
	mux_chip->mux[0].states = 4;   /* e.g., 4:1 mux */

	/* Parse idle-state from DT (optional) */
	ret = device_property_read_u32(dev, "idle-state", (u32 *)&idle_state);
	if (ret < 0)
		idle_state = MUX_IDLE_AS_IS;

	switch (idle_state) {
	case MUX_IDLE_DISCONNECT:
	case MUX_IDLE_AS_IS:
	case 0 ... 3:
		mux_chip->mux[0].idle_state = idle_state;
		break;
	default:
		dev_err(dev, "invalid idle-state %d\n", idle_state);
		return -EINVAL;
	}

	/*
	 * Register the mux chip. This sets the idle state on all
	 * controllers and makes them available to consumers.
	 * The devm variant auto-unregisters on driver detach.
	 */
	return devm_mux_chip_register(dev, mux_chip);
}

static const struct i2c_device_id <devname>_id[] = {
	{ .name = "<devname>", },
	{ }
};
MODULE_DEVICE_TABLE(i2c, <devname>_id);

static const struct of_device_id <devname>_of_match[] = {
	{ .compatible = "adi,<devname>", },
	{ }
};
MODULE_DEVICE_TABLE(of, <devname>_of_match);

static struct i2c_driver <devname>_driver = {
	.driver		= {
		.name		= "<devname>",
		.of_match_table = <devname>_of_match,
	},
	.probe		= <devname>_probe,
	.id_table	= <devname>_id,
};
module_i2c_driver(<devname>_driver);

MODULE_DESCRIPTION("Analog Devices <DEVNAME> Multiplexer driver");
MODULE_AUTHOR("Your Name <your.name@analog.com>");
MODULE_LICENSE("GPL v2");
```

### 6.2 SPI-controlled mux (e.g., ADGS1408 pattern)

```c
// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * <DEVNAME> SPI MUX driver
 *
 * Copyright YYYY Analog Devices Inc.
 */

#include <linux/err.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mux/driver.h>
#include <linux/property.h>
#include <linux/spi/spi.h>

#define <DEVNAME>_SW_DATA	0x01
#define <DEVNAME>_DISABLE	0x00
#define <DEVNAME>_MUX(state)	(((state) << 1) | 1)

static int <devname>_spi_reg_write(struct spi_device *spi,
				   u8 reg_addr, u8 reg_data)
{
	u8 tx_buf[2] = { reg_addr, reg_data };

	return spi_write_then_read(spi, tx_buf, sizeof(tx_buf), NULL, 0);
}

static int <devname>_set(struct mux_control *mux, int state)
{
	struct spi_device *spi = to_spi_device(mux->chip->dev.parent);
	u8 reg;

	if (state == MUX_IDLE_DISCONNECT)
		reg = <DEVNAME>_DISABLE;
	else
		reg = <DEVNAME>_MUX(state);

	return <devname>_spi_reg_write(spi, <DEVNAME>_SW_DATA, reg);
}

static const struct mux_control_ops <devname>_ops = {
	.set = <devname>_set,
};

static int <devname>_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct mux_chip *mux_chip;
	struct mux_control *mux;
	s32 idle_state;
	int ret;

	mux_chip = devm_mux_chip_alloc(dev, 1, 0);
	if (IS_ERR(mux_chip))
		return PTR_ERR(mux_chip);

	mux_chip->ops = &<devname>_ops;
	mux = mux_chip->mux;
	mux->states = 8;   /* e.g., 8:1 mux */

	/* Disable all switches at init */
	ret = <devname>_spi_reg_write(spi, <DEVNAME>_SW_DATA,
				      <DEVNAME>_DISABLE);
	if (ret < 0)
		return ret;

	/* Parse idle-state from DT */
	ret = device_property_read_u32(dev, "idle-state", (u32 *)&idle_state);
	if (ret < 0)
		idle_state = MUX_IDLE_AS_IS;

	switch (idle_state) {
	case MUX_IDLE_DISCONNECT:
	case MUX_IDLE_AS_IS:
	case 0 ... 7:
		mux->idle_state = idle_state;
		break;
	default:
		dev_err(dev, "invalid idle-state %d\n", idle_state);
		return -EINVAL;
	}

	return devm_mux_chip_register(dev, mux_chip);
}

static const struct spi_device_id <devname>_spi_id[] = {
	{ "<devname>", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, <devname>_spi_id);

static const struct of_device_id <devname>_of_match[] = {
	{ .compatible = "adi,<devname>", },
	{ }
};
MODULE_DEVICE_TABLE(of, <devname>_of_match);

static struct spi_driver <devname>_driver = {
	.driver = {
		.name = "<devname>",
		.of_match_table = <devname>_of_match,
	},
	.probe = <devname>_probe,
	.id_table = <devname>_spi_id,
};
module_spi_driver(<devname>_driver);

MODULE_AUTHOR("Your Name <your.name@analog.com>");
MODULE_DESCRIPTION("Analog Devices <DEVNAME> MUX driver");
MODULE_LICENSE("GPL");
```

### 6.3 GPIO-controlled mux (generic gpio-mux driver)

For muxes controlled purely by GPIO address lines (e.g., ADG2404),
the existing `drivers/mux/gpio.c` driver handles them. No new `.c`
file is needed -- only a devicetree entry with
`compatible = "gpio-mux"`:

```c
/* Key elements from drivers/mux/gpio.c for reference */

struct mux_gpio {
	struct gpio_descs *gpios;
};

static int mux_gpio_set(struct mux_control *mux, int state)
{
	struct mux_gpio *mux_gpio = mux_chip_priv(mux->chip);
	DECLARE_BITMAP(values, BITS_PER_TYPE(state));
	u32 value = state;

	bitmap_from_arr32(values, &value, BITS_PER_TYPE(value));

	gpiod_set_array_value_cansleep(mux_gpio->gpios->ndescs,
				       mux_gpio->gpios->desc,
				       mux_gpio->gpios->info, values);
	return 0;
}

/*
 * In probe: the number of GPIO pins determines the number of states.
 *   pins = gpiod_count(dev, "mux");
 *   mux_chip->mux->states = BIT(pins);  // 2^N states for N pins
 *
 * Private data is passed via sizeof_priv in devm_mux_chip_alloc()
 * and retrieved with mux_chip_priv(mux_chip).
 */
```

To use this for a custom GPIO mux, provide the DT node:

```dts
#include <dt-bindings/gpio/gpio.h>

mux_ctrl: mux-controller {
    compatible = "gpio-mux";
    #mux-control-cells = <0>;

    mux-gpios = <&gpio 10 GPIO_ACTIVE_HIGH>,  /* A0 */
                <&gpio 11 GPIO_ACTIVE_HIGH>;   /* A1 */

    /* Optional: set idle state to channel 0 */
    idle-state = <0>;
};
```

---

## 7. Consumer API

Mux consumers are other kernel drivers that need to route signals
through a multiplexer. The consumer API is defined in
`<linux/mux/consumer.h>`.

### 7.1 Acquiring a mux control

```c
#include <linux/mux/consumer.h>

/* In consumer's probe function: */

/* Option A: mux-controls property (consumer selects states dynamically) */
struct mux_control *mux;
mux = devm_mux_control_get(dev, NULL);    /* NULL = first mux-control */
mux = devm_mux_control_get(dev, "adc");   /* Named mux-control */
if (IS_ERR(mux))
	return PTR_ERR(mux);

/* Option B: mux-states property (consumer uses a fixed state) */
struct mux_state *mstate;
mstate = devm_mux_state_get(dev, NULL);
if (IS_ERR(mstate))
	return PTR_ERR(mstate);
```

### 7.2 Selecting and deselecting states

```c
int ret;

/*
 * mux_control_select() locks the mux and sets the state.
 * It blocks if the mux is already held by another consumer.
 * mux_control_try_select() returns -EBUSY instead of blocking.
 */
ret = mux_control_select(mux, desired_state);
if (ret)
	return ret;

/* ... perform the operation that requires this mux state ... */

/* Release the mux so other consumers can use it */
mux_control_deselect(mux);

/*
 * For mux_state (fixed state from DT):
 */
ret = mux_state_select(mstate);
if (ret)
	return ret;
/* ... */
mux_state_deselect(mstate);
```

### 7.3 Consumer DT example

```dts
/* Consumer references the mux controller */
adc-mux {
    compatible = "io-channel-mux";
    io-channels = <&adc 0>;
    io-channel-names = "parent";

    mux-controls = <&mux_ctrl>;
    mux-control-names = "adc";

    channels = "ch0", "ch1", "ch2", "ch3";
};

/* I2C bus mux consumer */
i2c-mux {
    compatible = "i2c-mux";
    i2c-parent = <&i2c1>;

    mux-controls = <&mux_ctrl>;

    #address-cells = <1>;
    #size-cells = <0>;

    i2c@0 {
        reg = <0>;
        #address-cells = <1>;
        #size-cells = <0>;
        /* devices on mux channel 0 */
    };
};

/* Fixed-state consumer (mux-states instead of mux-controls) */
can-phy {
    compatible = "ti,tcan1042";
    #phy-cells = <0>;
    mux-states = <&mux_ctrl 1>;   /* Always select state 1 */
};
```

---

## 8. Devicetree Parsing

### 8.1 Common DT properties to parse in probe

```c
/* Number of mux controllers exposed */
u32 cells;
ret = device_property_read_u32(dev, "#mux-control-cells", &cells);

/* Idle state (single controller) */
s32 idle_state;
ret = device_property_read_u32(dev, "idle-state", (u32 *)&idle_state);

/* Idle states (multi-controller, e.g., triple mux) */
s32 idle_states[3];
ret = device_property_read_u32_array(dev, "idle-state",
				     (u32 *)idle_states,
				     mux_chip->controllers);

/* GPIO specs (for gpio-mux driver) */
int pins = gpiod_count(dev, "mux");
struct gpio_descs *gpios = devm_gpiod_get_array(dev, "mux", GPIOD_OUT_LOW);

/* Number of mux states */
mux->states = 4;           /* fixed for the hardware */
mux->states = BIT(pins);   /* 2^N for GPIO-based mux with N pins */
```

### 8.2 Idle state validation pattern

```c
switch (idle_state) {
case MUX_IDLE_DISCONNECT:   /* -2: disconnect all paths */
case MUX_IDLE_AS_IS:        /* -1: leave as-is (default) */
case 0 ... (NUM_STATES - 1):
	mux->idle_state = idle_state;
	break;
default:
	dev_err(dev, "invalid idle-state %d\n", idle_state);
	return -EINVAL;
}
```

`MUX_IDLE_AS_IS` is the default set by `mux_chip_alloc()`, so if the
DT property is absent, no action is needed.

---

## 9. Test & Debug

### 9.1 debugfs

The mux core does not create debugfs entries by default. Mux state
can be observed indirectly via:

```
# Check sysfs class entries
ls /sys/class/mux/

# Each registered mux chip appears as muxchipN
cat /sys/class/mux/muxchip0/uevent
```

### 9.2 sysfs

The mux subsystem registers devices under `/sys/class/mux/`. Each
`muxchipN` device corresponds to a registered `mux_chip`. Attributes
include the standard device model attributes (`uevent`, `dev`,
`subsystem`).

### 9.3 Debugging tips

- Check `dmesg` for mux core messages (prefixed with `mux-core:`).
- Verify DT bindings with `make dt_binding_check`:
  ```
  make dt_binding_check DT_SCHEMA_FILES=Documentation/devicetree/bindings/mux/adi,<devname>.yaml
  ```
- Verify DT overlay/blob with `make dtbs_check`.
- Use `dev_dbg()` in the `.set` callback to trace state changes.
- For GPIO muxes, use `gpioinfo` (from `libgpiod-utils`) to verify
  GPIO line states.
- For I2C muxes, use `i2cdetect` / `i2cdump` to verify register
  writes reach the device.

---

## 10. Key Conventions

1. **License** -- `GPL-2.0` (or `GPL-2.0-or-later`). Use the
   SPDX identifier on line 1: `// SPDX-License-Identifier: GPL-2.0`.
2. **Managed allocation** -- prefer `devm_mux_chip_alloc()` and
   `devm_mux_chip_register()`. These handle cleanup automatically
   when the parent device is removed, eliminating the need for
   explicit `.remove` callbacks.
3. **Private data** -- pass `sizeof_priv` to `devm_mux_chip_alloc()`
   to allocate driver-specific data alongside the mux chip. Retrieve
   it with `mux_chip_priv(mux_chip)`.
4. **Idle state handling** -- always validate `idle-state` from DT
   against the valid range. The mux core calls `.set` with the idle
   state during `mux_chip_register()` and on `mux_control_deselect()`.
   Handle `MUX_IDLE_DISCONNECT` in `.set` to disable all switch
   paths.
5. **No `.remove` needed** -- with fully `devm_*` managed resources,
   the driver does not need a `.remove` callback. The mux chip is
   unregistered and freed automatically.
6. **Module macros** -- use `module_i2c_driver()`,
   `module_spi_driver()`, or `module_platform_driver()` as
   appropriate. These expand to `module_init` / `module_exit` with
   the bus register/unregister calls.
7. **OF match table** -- always provide `of_match_table` with
   `"adi,<devname>"` compatible strings. Include `of_match_ptr()`
   wrapper to compile out the table when CONFIG_OF is disabled.
8. **State numbering** -- mux states are zero-based contiguous
   integers. Set `mux->states` to the total number of valid states.
9. **Locking** -- the mux core handles locking via a semaphore per
   `mux_control`. The `.set` callback does not need to take any
   locks itself.
10. **Error paths** -- return negative errno values from `.set`.
    The core will mark the cached state as unknown on failure.

---

## 11. Commit Message Format

```
mux: <devname>: add support for <DEVNAME>

Add a mux controller driver for the Analog Devices <DEVNAME>,
a <brief description, e.g., wide bandwidth triple 4:1 analog
multiplexer> controlled via <I2C/SPI/GPIO>.

Signed-off-by: Your Name <your.name@analog.com>
```

For DT binding additions:

```
dt-bindings: mux: add binding for adi,<devname>

Add devicetree binding documentation for the Analog Devices
<DEVNAME> multiplexer.

Signed-off-by: Your Name <your.name@analog.com>
```
