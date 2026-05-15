# Linux IIO Multiplexer Driver Template

Reference driver: `drivers/iio/multiplexer/iio-mux.c`

This template covers the IIO multiplexer (iio-mux) driver pattern. The IIO mux
driver creates a virtual IIO device that routes IIO channels through a
`mux_control` from the kernel mux subsystem. It wraps a parent IIO channel and
exposes one virtual channel per mux state, selecting the appropriate mux state
before forwarding reads/writes to the parent channel.

Replace `<devname>` with the part-specific name (e.g., `iio-mux`) and
`<DEVNAME>` with its uppercase CONFIG symbol form (e.g., `IIO_MUX`) throughout.

---

## 1. Purpose

The IIO multiplexer subsystem provides virtual IIO devices that sit between
userspace and a real (parent) IIO channel. A mux controller -- typically a
GPIO-based mux, I2C mux, or any `struct mux_control` provider -- selects which
hardware signal reaches the parent channel.

Key characteristics:

- **Virtual device**: The mux driver does not perform any ADC/DAC conversion
  itself. It delegates all reads and writes to a parent IIO channel.
- **Channel-per-state**: Each mux controller state maps to one virtual IIO
  channel. For a 4:1 analog mux (2 select lines), there are up to 4 virtual
  channels.
- **Transparent forwarding**: `read_raw` selects the mux state, reads from the
  parent channel, then deselects. Scale, offset, and ext_info are forwarded
  the same way.
- **Platform driver**: The mux node in DT is a platform device
  (`compatible = "io-channel-mux"`), not SPI or I2C.
- **Consumer of two subsystems**: The driver consumes an `iio_channel`
  (via `io-channels` / `devm_iio_channel_get()`) and a `mux_control`
  (via `mux-controls` / `devm_mux_control_get()`).

---

## 2. File Checklist

| File                                                                  | Action   | Required |
|-----------------------------------------------------------------------|----------|----------|
| `drivers/iio/multiplexer/<devname>.c`                                 | Create   | Yes      |
| `drivers/iio/multiplexer/Kconfig`                                     | Modify   | Yes      |
| `drivers/iio/multiplexer/Makefile`                                    | Modify   | Yes      |
| `Documentation/devicetree/bindings/iio/multiplexer/<devname>.yaml`    | Create   | Yes      |

### Notes

- No header file is needed in the typical case. The mux driver is self-contained
  and does not export symbols.
- The parent IIO channel driver and the mux controller driver are separate;
  they must already exist and be probed before the iio-mux node.

---

## 3. DT Binding

Bindings live under
`Documentation/devicetree/bindings/iio/multiplexer/<devname>.yaml`.

### Required Properties

| Property           | Type              | Description                                                              |
|--------------------|-------------------|--------------------------------------------------------------------------|
| `compatible`       | string            | Match string (e.g. `"io-channel-mux"`)                                   |
| `io-channels`      | phandle + cell    | Reference to the parent IIO channel (the actual ADC/DAC input)           |
| `io-channel-names` | string            | Must be `"parent"`                                                       |
| `mux-controls`     | phandle           | Reference to the mux controller (e.g. a `gpio-mux` node)                |
| `channels`         | string array      | Labels for each mux state; empty string `""` means state is unused       |

### Optional Properties

| Property           | Type     | Description                                                             |
|--------------------|----------|-------------------------------------------------------------------------|
| `settle-time-us`   | u32      | Settling time in microseconds after mux selection (default: 0)          |
| `mux-control-names`| string   | Name for the mux control (when multiple mux-controls are present)       |

### Example Binding YAML

```yaml
# SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)
%YAML 1.2
---
$id: http://devicetree.org/schemas/iio/multiplexer/io-channel-mux.yaml#
$schema: http://devicetree.org/meta-schemas/core.yaml#

title: I/O channel multiplexer

maintainers:
  - Peter Rosin <peda@axentia.se>

description: |
  If a multiplexer is used to select which hardware signal is fed to
  e.g. an ADC channel, these bindings describe that situation.

  For each non-empty string in the channels property, an io-channel will be
  created. The number of this io-channel is the same as the index into the
  list of strings in the channels property, and also matches the mux
  controller state.

properties:
  compatible:
    const: io-channel-mux

  io-channels:
    maxItems: 1
    description: Channel node of the parent channel that has multiplexed input.

  io-channel-names:
    const: parent

  mux-controls: true
  mux-control-names: true

  channels:
    $ref: /schemas/types.yaml#/definitions/non-unique-string-array
    description:
      List of strings, labeling the mux controller states. An empty
      string for a state means that the channel is not available.

  settle-time-us:
    default: 0
    description:
      Time required for analog signals to settle after muxing.

required:
  - compatible
  - io-channels
  - io-channel-names
  - mux-controls
  - channels

additionalProperties: false
```

### Example DT Node

```dts
#include <dt-bindings/gpio/gpio.h>

mux: mux-controller {
    compatible = "gpio-mux";
    #mux-control-cells = <0>;

    mux-gpios = <&pioA 0 GPIO_ACTIVE_HIGH>,
                <&pioA 1 GPIO_ACTIVE_HIGH>;
};

adc-mux {
    compatible = "io-channel-mux";
    io-channels = <&adc 0>;
    io-channel-names = "parent";

    mux-controls = <&mux>;
    channels = "sync", "in", "system-regulator";
};
```

In this example, the `gpio-mux` node provides a 2-bit mux (4 states). The
`adc-mux` node wraps ADC channel 0 (`<&adc 0>`) and creates three virtual
IIO channels labeled `sync`, `in`, and `system-regulator`, mapping to mux
states 0, 1, and 2 respectively.

---

## 4. Kconfig

Add the entry to `drivers/iio/multiplexer/Kconfig` in alphabetical order.

```kconfig
config IIO_MUX
	tristate "IIO multiplexer driver"
	select MULTIPLEXER
	help
	  Say yes here to build support for the IIO multiplexer.

	  To compile this driver as a module, choose M here: the
	  module will be called iio-mux.
```

### Dependencies

| Symbol          | Purpose                                                                 |
|-----------------|-------------------------------------------------------------------------|
| `MULTIPLEXER`   | The kernel mux subsystem (`drivers/mux/`), provides `mux_control` API  |

The `MULTIPLEXER` symbol is pulled in via `select`, so users only need to
enable `IIO_MUX`. The actual mux controller driver (e.g., `MUX_GPIO`,
`MUX_ADG792A`) must be enabled separately.

---

## 5. Makefile

Add the entry to `drivers/iio/multiplexer/Makefile` in alphabetical order.

```makefile
obj-$(CONFIG_IIO_MUX) += iio-mux.o
```

The mux driver is a single-file driver; no multi-object rules are needed.

---

## 6. Driver Source

The iio-mux driver follows a distinctive pattern compared to standard IIO
drivers. It does not access hardware registers directly. Instead, it acts as
a proxy: select a mux state, forward the IIO operation to the parent channel,
then deselect.

### Key Data Structures

```c
struct mux_ext_info_cache {
	char *data;
	ssize_t size;
};

struct mux_child {
	struct mux_ext_info_cache *ext_info_cache;
};

struct mux {
	int cached_state;
	struct mux_control *control;
	struct iio_channel *parent;
	struct iio_chan_spec *chan;
	struct iio_chan_spec_ext_info *ext_info;
	struct mux_child *child;
	u32 delay_us;
};
```

- `mux_control *control` -- Handle to the mux subsystem controller.
- `iio_channel *parent` -- The real (upstream) IIO channel being multiplexed.
- `chan[]` -- Array of virtual `iio_chan_spec`, one per active mux state.
- `child[]` -- Per-channel cache of ext_info values, so each mux state can
  maintain independent ext_info settings.
- `cached_state` -- Tracks the last selected state to avoid redundant ext_info
  writes when the mux state has not changed.

### Select / Deselect Pattern

Every IIO operation wraps the parent access in a select/deselect pair:

```c
static int iio_mux_select(struct mux *mux, int idx)
{
	struct mux_child *child = &mux->child[idx];
	struct iio_chan_spec const *chan = &mux->chan[idx];
	int ret;

	ret = mux_control_select_delay(mux->control, chan->channel,
				       mux->delay_us);
	if (ret < 0) {
		mux->cached_state = -1;
		return ret;
	}

	if (mux->cached_state == chan->channel)
		return 0;

	/* Restore ext_info for this mux state if changed */
	if (chan->ext_info) {
		for (i = 0; chan->ext_info[i].name; ++i) {
			cache = &child->ext_info_cache[i];
			if (cache->size < 0)
				continue;
			ret = iio_write_channel_ext_info(mux->parent,
				chan->ext_info[i].name,
				cache->data, cache->size);
			if (ret < 0) {
				mux_control_deselect(mux->control);
				mux->cached_state = -1;
				return ret;
			}
		}
	}
	mux->cached_state = chan->channel;

	return 0;
}

static void iio_mux_deselect(struct mux *mux)
{
	mux_control_deselect(mux->control);
}
```

### read_raw / write_raw

```c
static int mux_read_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			int *val, int *val2, long mask)
{
	struct mux *mux = iio_priv(indio_dev);
	int idx = chan - mux->chan;
	int ret;

	ret = iio_mux_select(mux, idx);
	if (ret < 0)
		return ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = iio_read_channel_raw(mux->parent, val);
		break;
	case IIO_CHAN_INFO_SCALE:
		ret = iio_read_channel_scale(mux->parent, val, val2);
		break;
	default:
		ret = -EINVAL;
	}

	iio_mux_deselect(mux);

	return ret;
}

static int mux_write_raw(struct iio_dev *indio_dev,
			 struct iio_chan_spec const *chan,
			 int val, int val2, long mask)
{
	struct mux *mux = iio_priv(indio_dev);
	int idx = chan - mux->chan;
	int ret;

	ret = iio_mux_select(mux, idx);
	if (ret < 0)
		return ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = iio_write_channel_raw(mux->parent, val);
		break;
	default:
		ret = -EINVAL;
	}

	iio_mux_deselect(mux);

	return ret;
}
```

### Channel Configuration

Channels are not statically defined. They are constructed dynamically at probe
time based on the parent channel's type and capabilities:

```c
static int mux_configure_channel(struct device *dev, struct mux *mux,
				 u32 state, const char *label, int idx)
{
	struct iio_chan_spec *chan = &mux->chan[idx];
	struct iio_chan_spec const *pchan = mux->parent->channel;

	chan->indexed = 1;
	chan->output = pchan->output;
	chan->datasheet_name = label;
	chan->ext_info = mux->ext_info;

	/* Inherit type from parent */
	iio_get_channel_type(mux->parent, &chan->type);

	/* Inherit info_mask from parent */
	if (iio_channel_has_info(pchan, IIO_CHAN_INFO_RAW))
		chan->info_mask_separate |= BIT(IIO_CHAN_INFO_RAW);
	if (iio_channel_has_info(pchan, IIO_CHAN_INFO_SCALE))
		chan->info_mask_separate |= BIT(IIO_CHAN_INFO_SCALE);
	if (iio_channel_has_available(pchan, IIO_CHAN_INFO_RAW))
		chan->info_mask_separate_available |= BIT(IIO_CHAN_INFO_RAW);

	/* Mux state becomes the channel number */
	chan->channel = state;

	/* ... set up ext_info cache for this child ... */

	return 0;
}
```

### Probe Function

```c
static int mux_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iio_dev *indio_dev;
	struct iio_channel *parent;
	struct mux *mux;
	const char **labels;
	int all_children, children;
	u32 state;
	int ret;

	/* 1. Acquire the parent IIO channel */
	parent = devm_iio_channel_get(dev, "parent");
	if (IS_ERR(parent))
		return dev_err_probe(dev, PTR_ERR(parent),
				     "failed to get parent channel\n");

	/* 2. Read channel labels from DT "channels" property */
	all_children = device_property_string_array_count(dev, "channels");
	if (all_children < 0)
		return all_children;

	labels = devm_kmalloc_array(dev, all_children,
				    sizeof(*labels), GFP_KERNEL);
	if (!labels)
		return -ENOMEM;

	ret = device_property_read_string_array(dev, "channels",
						labels, all_children);
	if (ret < 0)
		return ret;

	/* Count non-empty labels (active channels) */
	children = 0;
	for (state = 0; state < all_children; state++) {
		if (*labels[state])
			children++;
	}

	/* 3. Allocate IIO device with space for mux + children + channels */
	indio_dev = devm_iio_device_alloc(dev, sizeof_priv);
	if (!indio_dev)
		return -ENOMEM;

	mux = iio_priv(indio_dev);
	mux->parent = parent;
	mux->cached_state = -1;

	device_property_read_u32(dev, "settle-time-us", &mux->delay_us);

	/* 4. Get the mux controller */
	mux->control = devm_mux_control_get(dev, NULL);
	if (IS_ERR(mux->control))
		return dev_err_probe(dev, PTR_ERR(mux->control),
				     "failed to get control-mux\n");

	/* 5. Configure one virtual channel per active mux state */
	for (state = 0; state < all_children; state++) {
		if (!*labels[state])
			continue;
		ret = mux_configure_channel(dev, mux, state,
					    labels[state], i++);
		if (ret < 0)
			return ret;
	}

	indio_dev->name = dev_name(dev);
	indio_dev->info = &mux_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = mux->chan;
	indio_dev->num_channels = children;

	return devm_iio_device_register(dev, indio_dev);
}
```

### Match Table and Module Boilerplate

```c
static const struct of_device_id mux_match[] = {
	{ .compatible = "io-channel-mux" },
	{ }
};
MODULE_DEVICE_TABLE(of, mux_match);

static struct platform_driver mux_driver = {
	.probe = mux_probe,
	.driver = {
		.name = "iio-mux",
		.of_match_table = mux_match,
	},
};
module_platform_driver(mux_driver);

MODULE_DESCRIPTION("IIO multiplexer driver");
MODULE_AUTHOR("First Last <first.last@analog.com>");
MODULE_LICENSE("GPL v2");
```

---

## 7. Interaction with MUX Subsystem

The iio-mux driver depends on the kernel mux subsystem
(`include/linux/mux/consumer.h`). Key APIs:

| Function                          | Purpose                                               |
|-----------------------------------|-------------------------------------------------------|
| `devm_mux_control_get(dev, NULL)` | Acquire the mux control handle from DT                |
| `mux_control_select(ctrl, state)` | Lock the mux and set it to `state`                    |
| `mux_control_select_delay(ctrl, state, us)` | Same, but wait `us` microseconds after switching |
| `mux_control_deselect(ctrl)`      | Release the mux lock                                  |
| `mux_control_states(ctrl)`        | Query how many states the mux controller supports     |

### Lock Ordering

- `mux_control_select()` acquires an internal mutex inside the mux subsystem.
  It blocks if another consumer already holds the mux.
- While the mux is selected, the IIO mux driver reads/writes the parent IIO
  channel. The parent channel's own locking (if any) is acquired inside the
  parent driver.
- Lock order is always: **mux lock first, then parent channel access**. Never
  hold a parent channel lock and then call `mux_control_select()`.
- `mux_control_deselect()` must always be called after `mux_control_select()`
  returns successfully, even on error paths for the subsequent IIO operation.

### Settling Time

The `settle-time-us` DT property is passed to `mux_control_select_delay()`.
This inserts a delay after the mux controller switches state, giving analog
signals time to settle before the parent ADC performs a conversion. Typical
values range from 0 (fast digital signals) to hundreds of microseconds
(high-impedance analog signals through long traces).

---

## 8. DT Parsing

The iio-mux driver uses a flat property-based approach rather than child nodes:

### Channel Labels from the `channels` Property

Unlike most IIO drivers that use `channel@N` child nodes, the iio-mux driver
reads mux states from a single string array property:

```dts
channels = "ch0_label", "", "ch2_label", "ch3_label";
```

- Each string position corresponds to a mux controller state (0, 1, 2, 3, ...).
- Non-empty strings create a virtual IIO channel with that label as its
  `datasheet_name`.
- Empty strings (`""`) indicate unused states -- no channel is created for them.
- The string index directly maps to the `chan->channel` value passed to
  `mux_control_select()`.

### Parsing Flow in `probe()`

```
device_property_string_array_count(dev, "channels")
    --> total number of mux states

device_property_read_string_array(dev, "channels", labels, count)
    --> populates the labels array

for each non-empty label:
    mux_configure_channel(dev, mux, state_index, label, channel_index)
```

### Mux State Mapping

The relationship between DT strings and mux states:

```
channels = "vbat", "temp", "", "vsys";
             ^       ^     ^    ^
             |       |     |    state 3 --> channel 2 (chan->channel = 3)
             |       |     unused (skipped)
             |       state 1 --> channel 1 (chan->channel = 1)
             state 0 --> channel 0 (chan->channel = 0)
```

The IIO channel index is sequential (0, 1, 2, ...) for active channels only,
but `chan->channel` stores the actual mux state number. This allows sparse
mux configurations where some states are intentionally unused.

### Parent Channel

The parent IIO channel is obtained via:

```c
parent = devm_iio_channel_get(dev, "parent");
```

This looks for `io-channels` in the DT node and matches it with
`io-channel-names = "parent"`. The parent must be a valid IIO channel provider
(typically an ADC). The mux driver inherits the parent's channel type,
info_mask, and ext_info.

---

## 9. Test & Debug

### Sysfs Verification

Once the driver loads, virtual channels appear under the IIO device:

```
/sys/bus/iio/devices/iio:deviceN/
    name                              # e.g., "adc-mux"
    in_voltage0_raw                   # Reads parent via mux state 0
    in_voltage1_raw                   # Reads parent via mux state 1
    in_voltage2_raw                   # Reads parent via mux state 2
    in_voltage0_scale                 # Forwarded from parent
    in_voltage1_scale                 # Same scale (same parent ADC)
    ...
```

Each `in_voltageN_raw` read triggers the sequence:

1. `mux_control_select_delay(ctrl, N, settle_time)`
2. `iio_read_channel_raw(parent, &val)`
3. `mux_control_deselect(ctrl)`

### Verifying Mux Selection

Check that distinct channels return different raw values when different
analog signals are connected to the mux inputs:

```sh
# Should show different values if inputs differ
cat /sys/bus/iio/devices/iio:device0/in_voltage0_raw
cat /sys/bus/iio/devices/iio:device0/in_voltage1_raw
```

### Checking DT Probe Success

```sh
# Verify the platform device was probed
dmesg | grep iio-mux

# Check that both the mux controller and parent ADC are present
ls /sys/bus/iio/devices/
```

### Common Probe Failures

| Error Message                        | Cause                                                    |
|--------------------------------------|----------------------------------------------------------|
| `failed to get parent channel`       | Parent ADC not probed, or `io-channels` phandle wrong    |
| `failed to get control-mux`          | Mux controller not probed, or `mux-controls` wrong       |
| `too many channels`                  | More non-empty labels than mux controller states          |

### Debugging ext_info Forwarding

If the parent ADC exposes custom ext_info attributes (e.g., calibration), the
mux driver caches and restores them per mux state. Verify with:

```sh
# Write to ext_info on channel 0
echo "value" > /sys/bus/iio/devices/iio:device0/in_voltage0_ext_attr

# Switch to channel 1, write different value
echo "other" > /sys/bus/iio/devices/iio:device0/in_voltage1_ext_attr

# Read back channel 0 -- should restore original "value"
cat /sys/bus/iio/devices/iio:device0/in_voltage0_ext_attr
```

---

## 10. Key Conventions

### License

All IIO multiplexer drivers must use GPL-2.0:

```c
// SPDX-License-Identifier: GPL-2.0
```

With matching `MODULE_LICENSE("GPL v2")` at the bottom.

### Platform Driver

The mux driver is a `platform_driver` using `module_platform_driver()`, not
SPI or I2C. The mux node is a standalone platform device in the DT, typically
at the root level or under a bus that supports platform devices.

### IIO Consumer Pattern

The iio-mux driver is an IIO *consumer* (it reads from another IIO device)
and an IIO *provider* (it registers its own `iio_dev`). This dual role
requires:

- `#include <linux/iio/consumer.h>` -- for `devm_iio_channel_get()`,
  `iio_read_channel_raw()`, etc.
- `#include <linux/iio/iio.h>` -- for `devm_iio_device_alloc()`,
  `devm_iio_device_register()`, etc.

### Memory Management

Use `devm_*` allocations throughout:

| Function                          | Purpose                                      |
|-----------------------------------|----------------------------------------------|
| `devm_iio_device_alloc()`         | Allocate IIO device + private data           |
| `devm_iio_device_register()`      | Register IIO device (auto-unregister)        |
| `devm_iio_channel_get()`          | Get parent IIO channel reference             |
| `devm_mux_control_get()`          | Get mux control handle                       |
| `devm_kmalloc_array()`            | Allocate labels array                        |
| `devm_kcalloc()`                  | Allocate ext_info cache arrays               |
| `devm_kmemdup()`                  | Duplicate ext_info data and strings          |
| `devm_kfree()`                    | Free replaced ext_info cache entries         |

### No Buffer / Trigger Support

The iio-mux driver operates in `INDIO_DIRECT_MODE` only. It does not support
triggered buffers or continuous capture. Each read is a one-shot operation
that selects the mux, reads the parent, and deselects.

### Error Handling in probe()

Use `dev_err_probe()` for all error returns -- it handles `-EPROBE_DEFER`
correctly, which is common for mux drivers since they depend on both a parent
IIO channel and a mux controller that may not be probed yet.

---

## 11. Commit Format

### Subject Line Prefix

```
iio: multiplexer: <devname>: <brief description>
```

### Examples

```
iio: multiplexer: iio-mux: add support for io-channel-mux
iio: multiplexer: iio-mux: fix ext_info cache restore on state change
iio: multiplexer: iio-mux: add settle-time-us support
iio: multiplexer: iio-gen-mux: add generic mux control via IIO attribute
```

### DT Binding Patch

```
dt-bindings: iio: multiplexer: add io-channel-mux binding
```

### Patch Series for a New Multiplexer Driver

1. `dt-bindings: iio: multiplexer: add <devname> binding` -- DT binding YAML
2. `iio: multiplexer: <devname>: add support for <DEVNAME>` -- Driver source
3. `MAINTAINERS: add entry for <DEVNAME> IIO mux driver` -- Maintainer entry

### Commit Body

- Wrap at 75 characters.
- Explain why the mux driver is needed (e.g., what hardware mux is being
  driven, what parent ADC it wraps).
- Reference the parent device and mux controller when relevant.
- Include `Signed-off-by:` (use `git commit -s`).

### Full Example

```
iio: multiplexer: iio-mux: add support for io-channel-mux

The io-channel-mux driver creates virtual IIO channels that route
through a mux controller to a single parent IIO channel. This allows
multiple analog signals to share a single ADC input via an external
analog multiplexer.

The driver supports:
  - Dynamic channel creation from DT channel labels
  - Per-channel ext_info caching and restoration
  - Configurable settle time after mux state changes
  - Transparent forwarding of read_raw and write_raw

Signed-off-by: First Last <first.last@analog.com>
```
