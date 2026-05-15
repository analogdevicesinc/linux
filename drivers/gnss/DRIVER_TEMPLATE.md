# Linux GNSS Driver Template

Reference driver: `drivers/gnss/ubx.c` (serial helper) and `drivers/gnss/sirf.c`
(standalone)

This template covers every file needed to add a new GNSS receiver driver to the
Linux kernel's GNSS subsystem. The subsystem exposes each receiver as a
`/dev/gnssN` character device that carries raw NMEA (or vendor-proprietary)
data. Replace `<devname>` with the chip/vendor shortname (e.g., `ubx`) and
`<DEVNAME>` with its uppercase form (e.g., `UBX`) throughout.

---

## 1. Purpose & Subsystem Mapping

| Concept | GNSS subsystem |
|---|---|
| Core abstraction | `struct gnss_device` (`include/linux/gnss.h`) |
| Userspace interface | `/dev/gnssN` character device (read/write/poll) |
| Device types | `GNSS_TYPE_NMEA`, `GNSS_TYPE_SIRF`, `GNSS_TYPE_UBX`, `GNSS_TYPE_MTK` |
| Data flow (RX) | Driver calls `gnss_insert_raw()` to push received NMEA/binary data into the core FIFO |
| Data flow (TX) | Core calls `ops->write_raw()` to send commands to the receiver |
| Serial helper | `gnss-serial` module wraps common serdev open/close/PM patterns |
| Typical hardware | GPS/GNSS receivers connected via UART (serdev), I2C, or USB |

The GNSS core (`drivers/gnss/core.c`) manages the character device, read
FIFO (4 KiB), write buffer (1 KiB), open/close reference counting, and
poll support. Individual drivers only need to implement the
`struct gnss_operations` callbacks and feed received data into the core.

---

## 2. File Checklist

```
drivers/gnss/
    <devname>.c                          # Driver source

Documentation/devicetree/bindings/gnss/
    <vendor>,<part>.yaml                 # DT binding (if DT-based)

# Modified files:
drivers/gnss/Kconfig                     # Add GNSS_<DEVNAME> entry
drivers/gnss/Makefile                    # Add obj-$(CONFIG_GNSS_<DEVNAME>)
```

If the driver uses the generic serial helper, no additional `.h` file is
required -- `serial.h` is already provided by the subsystem.

---

## 3. Devicetree Binding (`.yaml`)

Path: `Documentation/devicetree/bindings/gnss/<vendor>,<part>.yaml`

```yaml
# SPDX-License-Identifier: GPL-2.0
%YAML 1.2
---
$id: http://devicetree.org/schemas/gnss/<vendor>,<part>.yaml#
$schema: http://devicetree.org/meta-schemas/core.yaml#

title: <Vendor> GNSS Receiver

allOf:
  - $ref: gnss-common.yaml#
  - $ref: /schemas/serial/serial-peripheral-props.yaml#

maintainers:
  - Your Name <your.name@example.com>

description: >
  The <PART> is a multi-constellation GNSS receiver supporting GPS, GLONASS,
  Galileo, and BeiDou. It communicates over a UART serial interface.

properties:
  compatible:
    enum:
      - <vendor>,<part>
      # Add variants here

  reg:
    description: >
      DDC (I2C) slave address, SPI chip-select, or USB port number
      (bus-dependent). Required for non-UART buses.

  vcc-supply:
    description: Main power supply regulator.

  v-bckp-supply:
    description: Backup battery supply (optional, preserves ephemeris data).

  enable-gpios:
    maxItems: 1
    description: GPIO to enable/disable the receiver (from gnss-common.yaml).

  timepulse-gpios:
    maxItems: 1
    description: PPS (pulse-per-second) output from the receiver (from gnss-common.yaml).

  reset-gpios:
    maxItems: 1
    description: Active-low hardware reset line.

  current-speed:
    description: Initial UART baud rate (serdev property, default 9600).

required:
  - compatible
  - vcc-supply

unevaluatedProperties: false

examples:
  - |
    #include <dt-bindings/gpio/gpio.h>

    serial {
        gnss {
            compatible = "<vendor>,<part>";
            vcc-supply = <&gnss_vcc_reg>;
            v-bckp-supply = <&gnss_bckp_reg>;
            enable-gpios = <&gpio 5 GPIO_ACTIVE_HIGH>;
            timepulse-gpios = <&gpio 6 GPIO_ACTIVE_HIGH>;
            reset-gpios = <&gpio 7 GPIO_ACTIVE_LOW>;
            current-speed = <9600>;
        };
    };
```

Properties inherited from `gnss-common.yaml`:
- `lna-supply` -- external Low Noise Amplifier regulator
- `enable-gpios` -- enable/disable line
- `timepulse-gpios` -- PPS output

---

## 4. Kconfig

Add to `drivers/gnss/Kconfig`, inside the `if GNSS` block:

```kconfig
config GNSS_<DEVNAME>_SERIAL
	tristate "<Vendor> GNSS receiver support"
	depends on SERIAL_DEV_BUS
	select GNSS_SERIAL
	help
	  Say Y here if you have a <Vendor>-based GNSS receiver which uses a
	  serial interface.

	  To compile this driver as a module, choose M here: the module will
	  be called gnss-<devname>.

	  If unsure, say N.
```

Notes:
- Use `depends on SERIAL_DEV_BUS` for UART-attached receivers.
- Use `select GNSS_SERIAL` if the driver uses the generic serial helper
  (`gnss_serial_allocate` / `gnss_serial_register`).
- For USB-attached receivers, use `depends on USB` instead.
- For I2C-attached receivers, use `depends on I2C`.
- Omit `select GNSS_SERIAL` if the driver manages serdev directly
  (like the SiRFstar driver does).

---

## 5. Makefile

Add to `drivers/gnss/Makefile`:

```makefile
obj-$(CONFIG_GNSS_<DEVNAME>_SERIAL)	+= gnss-<devname>.o
gnss-<devname>-y := <devname>.o
```

The naming convention `gnss-<devname>` produces the module name
`gnss-<devname>.ko`.

---

## 6. Driver Source (`.c`)

### 6a. Using the Serial Helper (simple receivers)

The `gnss-serial` helper handles serdev open/close, baud rate
configuration, receive buffering, and power management. The driver only
provides a `set_power` callback.

```c
// SPDX-License-Identifier: GPL-2.0
/*
 * <Vendor> GNSS receiver driver
 *
 * Copyright (C) YYYY Your Name <your.name@example.com>
 */

#include <linux/errno.h>
#include <linux/gnss.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/serdev.h>

#include "serial.h"

struct <devname>_data {
	struct regulator *vcc;
};

static int <devname>_set_active(struct gnss_serial *gserial)
{
	struct <devname>_data *data = gnss_serial_get_drvdata(gserial);

	return regulator_enable(data->vcc);
}

static int <devname>_set_standby(struct gnss_serial *gserial)
{
	struct <devname>_data *data = gnss_serial_get_drvdata(gserial);

	return regulator_disable(data->vcc);
}

static int <devname>_set_power(struct gnss_serial *gserial,
			       enum gnss_serial_pm_state state)
{
	switch (state) {
	case GNSS_SERIAL_ACTIVE:
		return <devname>_set_active(gserial);
	case GNSS_SERIAL_OFF:
	case GNSS_SERIAL_STANDBY:
		return <devname>_set_standby(gserial);
	}

	return -EINVAL;
}

static const struct gnss_serial_ops <devname>_gserial_ops = {
	.set_power = <devname>_set_power,
};

static int <devname>_probe(struct serdev_device *serdev)
{
	struct gnss_serial *gserial;
	struct <devname>_data *data;
	int ret;

	gserial = gnss_serial_allocate(serdev, sizeof(*data));
	if (IS_ERR(gserial))
		return PTR_ERR(gserial);

	gserial->ops = &<devname>_gserial_ops;
	gserial->gdev->type = GNSS_TYPE_NMEA; /* or GNSS_TYPE_UBX, etc. */

	data = gnss_serial_get_drvdata(gserial);

	data->vcc = devm_regulator_get(&serdev->dev, "vcc");
	if (IS_ERR(data->vcc)) {
		ret = PTR_ERR(data->vcc);
		goto err_free_gserial;
	}

	/* Optional: backup supply */
	ret = devm_regulator_get_enable_optional(&serdev->dev, "v-bckp");
	if (ret < 0 && ret != -ENODEV)
		goto err_free_gserial;

	/* Optional: deassert reset */
	/* devm_gpiod_get_optional(&serdev->dev, "reset", GPIOD_OUT_LOW); */

	ret = gnss_serial_register(gserial);
	if (ret)
		goto err_free_gserial;

	return 0;

err_free_gserial:
	gnss_serial_free(gserial);

	return ret;
}

static void <devname>_remove(struct serdev_device *serdev)
{
	struct gnss_serial *gserial = serdev_device_get_drvdata(serdev);

	gnss_serial_deregister(gserial);
	gnss_serial_free(gserial);
}

#ifdef CONFIG_OF
static const struct of_device_id <devname>_of_match[] = {
	{ .compatible = "<vendor>,<part>" },
	{},
};
MODULE_DEVICE_TABLE(of, <devname>_of_match);
#endif

static struct serdev_device_driver <devname>_driver = {
	.driver	= {
		.name		= "gnss-<devname>",
		.of_match_table	= of_match_ptr(<devname>_of_match),
		.pm		= &gnss_serial_pm_ops,
	},
	.probe	= <devname>_probe,
	.remove	= <devname>_remove,
};
module_serdev_device_driver(<devname>_driver);

MODULE_AUTHOR("Your Name <your.name@example.com>");
MODULE_DESCRIPTION("<Vendor> GNSS receiver driver");
MODULE_LICENSE("GPL v2");
```

### 6b. Standalone (without serial helper)

For receivers requiring custom power sequencing, GPIO toggling, or
interrupt-driven wakeup detection, manage the `gnss_device` directly.

```c
// SPDX-License-Identifier: GPL-2.0

#include <linux/gnss.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/serdev.h>

struct <devname>_data {
	struct gnss_device *gdev;
	struct serdev_device *serdev;
	speed_t speed;
	struct regulator *vcc;
	struct gpio_desc *enable;
	bool open;
	struct mutex gdev_mutex;
};

/* -------- gnss_operations callbacks -------- */

static int <devname>_open(struct gnss_device *gdev)
{
	struct <devname>_data *data = gnss_get_drvdata(gdev);
	int ret;

	ret = serdev_device_open(data->serdev);
	if (ret)
		return ret;

	serdev_device_set_baudrate(data->serdev, data->speed);
	serdev_device_set_flow_control(data->serdev, false);

	mutex_lock(&data->gdev_mutex);
	data->open = true;
	mutex_unlock(&data->gdev_mutex);

	ret = pm_runtime_get_sync(&data->serdev->dev);
	if (ret < 0) {
		pm_runtime_put_noidle(&data->serdev->dev);
		serdev_device_close(data->serdev);
		return ret;
	}

	return 0;
}

static void <devname>_close(struct gnss_device *gdev)
{
	struct <devname>_data *data = gnss_get_drvdata(gdev);

	pm_runtime_put(&data->serdev->dev);

	mutex_lock(&data->gdev_mutex);
	data->open = false;
	mutex_unlock(&data->gdev_mutex);

	serdev_device_close(data->serdev);
}

static int <devname>_write_raw(struct gnss_device *gdev,
			       const unsigned char *buf, size_t count)
{
	struct <devname>_data *data = gnss_get_drvdata(gdev);
	int ret;

	ret = serdev_device_write(data->serdev, buf, count,
				  MAX_SCHEDULE_TIMEOUT);
	if (ret < 0 || ret < count)
		return ret;

	serdev_device_wait_until_sent(data->serdev, 0);

	return count;
}

static const struct gnss_operations <devname>_gnss_ops = {
	.open		= <devname>_open,
	.close		= <devname>_close,
	.write_raw	= <devname>_write_raw,
};

/* -------- serdev receive callback -------- */

static size_t <devname>_receive_buf(struct serdev_device *serdev,
				    const u8 *buf, size_t count)
{
	struct <devname>_data *data = serdev_device_get_drvdata(serdev);
	int ret = 0;

	mutex_lock(&data->gdev_mutex);
	if (data->open)
		ret = gnss_insert_raw(data->gdev, buf, count);
	mutex_unlock(&data->gdev_mutex);

	return ret;
}

static const struct serdev_device_ops <devname>_serdev_ops = {
	.receive_buf	= <devname>_receive_buf,
	.write_wakeup	= serdev_device_write_wakeup,
};

/* -------- probe / remove -------- */

static int <devname>_probe(struct serdev_device *serdev)
{
	struct device *dev = &serdev->dev;
	struct gnss_device *gdev;
	struct <devname>_data *data;
	int ret;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	gdev = gnss_allocate_device(dev);
	if (!gdev)
		return -ENOMEM;

	gdev->type = GNSS_TYPE_NMEA;
	gdev->ops = &<devname>_gnss_ops;
	gnss_set_drvdata(gdev, data);

	data->serdev = serdev;
	data->gdev = gdev;
	data->speed = 9600;
	mutex_init(&data->gdev_mutex);

	serdev_device_set_drvdata(serdev, data);
	serdev_device_set_client_ops(serdev, &<devname>_serdev_ops);

	/* Parse DT: current-speed */
	of_property_read_u32(dev->of_node, "current-speed",
			     (u32 *)&data->speed);

	data->vcc = devm_regulator_get(dev, "vcc");
	if (IS_ERR(data->vcc)) {
		ret = PTR_ERR(data->vcc);
		goto err_put_device;
	}

	/* Optional enable GPIO */
	data->enable = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(data->enable)) {
		ret = PTR_ERR(data->enable);
		goto err_put_device;
	}

	if (IS_ENABLED(CONFIG_PM)) {
		pm_runtime_set_suspended(dev);
		pm_runtime_enable(dev);
	}

	ret = gnss_register_device(gdev);
	if (ret)
		goto err_disable_rpm;

	return 0;

err_disable_rpm:
	if (IS_ENABLED(CONFIG_PM))
		pm_runtime_disable(dev);
err_put_device:
	gnss_put_device(gdev);

	return ret;
}

static void <devname>_remove(struct serdev_device *serdev)
{
	struct <devname>_data *data = serdev_device_get_drvdata(serdev);

	gnss_deregister_device(data->gdev);

	if (IS_ENABLED(CONFIG_PM))
		pm_runtime_disable(&serdev->dev);

	gnss_put_device(data->gdev);
}
```

### Key API summary

| Function | Purpose |
|---|---|
| `gnss_allocate_device(parent)` | Allocate a `gnss_device`, init FIFO/mutexes/cdev |
| `gnss_put_device(gdev)` | Drop reference (counterpart to allocate) |
| `gnss_register_device(gdev)` | Create `/dev/gnssN`, expose to userspace |
| `gnss_deregister_device(gdev)` | Unregister, mark disconnected, close if open |
| `gnss_insert_raw(gdev, buf, count)` | Push received NMEA/binary data into the read FIFO |
| `gnss_set_drvdata(gdev, data)` | Store driver-private pointer |
| `gnss_get_drvdata(gdev)` | Retrieve driver-private pointer |
| `gnss_serial_allocate(serdev, sz)` | Allocate serial-helper wrapper (includes gnss_device) |
| `gnss_serial_register(gserial)` | Register via serial helper |
| `gnss_serial_deregister(gserial)` | Deregister via serial helper |
| `gnss_serial_free(gserial)` | Free serial helper |

---

## 7. Power Management

### Runtime PM (standalone driver)

```c
static int <devname>_runtime_suspend(struct device *dev)
{
	struct <devname>_data *data = dev_get_drvdata(dev);

	if (data->enable)
		gpiod_set_value_cansleep(data->enable, 0);

	return regulator_disable(data->vcc);
}

static int <devname>_runtime_resume(struct device *dev)
{
	struct <devname>_data *data = dev_get_drvdata(dev);
	int ret;

	ret = regulator_enable(data->vcc);
	if (ret)
		return ret;

	if (data->enable)
		gpiod_set_value_cansleep(data->enable, 1);

	return 0;
}

static int __maybe_unused <devname>_suspend(struct device *dev)
{
	if (!pm_runtime_suspended(dev))
		return <devname>_runtime_suspend(dev);

	return 0;
}

static int __maybe_unused <devname>_resume(struct device *dev)
{
	if (!pm_runtime_suspended(dev))
		return <devname>_runtime_resume(dev);

	return 0;
}

static const struct dev_pm_ops <devname>_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(<devname>_suspend, <devname>_resume)
	SET_RUNTIME_PM_OPS(<devname>_runtime_suspend,
			   <devname>_runtime_resume, NULL)
};
```

### Serial helper PM

Drivers using the serial helper get PM for free via `&gnss_serial_pm_ops`
-- the helper calls `ops->set_power()` with the appropriate
`gnss_serial_pm_state` (`GNSS_SERIAL_ACTIVE`, `GNSS_SERIAL_STANDBY`, or
`GNSS_SERIAL_OFF`).

### Enable/disable GPIO pattern

Many GNSS modules have an active-high enable pin. In runtime PM callbacks:
- **resume**: assert the enable GPIO, then wait for the receiver boot
  delay before returning.
- **suspend**: deassert the enable GPIO.

Use `devm_gpiod_get_optional()` during probe so the GPIO is optional.

---

## 8. Devicetree Parsing

Common properties parsed during probe:

```c
/* UART baud rate (serdev property) */
u32 speed = 9600;
of_property_read_u32(dev->of_node, "current-speed", &speed);
data->speed = speed;

/* Main supply (required) */
data->vcc = devm_regulator_get(dev, "vcc");

/* Backup supply (optional) */
ret = devm_regulator_get_enable_optional(dev, "v-bckp");

/* LNA supply (optional, from gnss-common) */
data->lna = devm_regulator_get_optional(dev, "lna");

/* Enable GPIO (optional, from gnss-common) */
data->enable = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_LOW);

/* Reset GPIO (optional) */
reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);

/* Timepulse/PPS GPIO (optional, from gnss-common) */
data->timepulse = devm_gpiod_get_optional(dev, "timepulse", GPIOD_IN);
```

The serial helper (`gnss_serial_allocate`) automatically reads
`current-speed` from the DT node -- drivers using the helper do not need
to parse it themselves.

---

## 9. Test & Debug

### Userspace interface

```sh
# The GNSS core creates /dev/gnssN on registration
cat /dev/gnss0                    # Stream raw NMEA sentences

# Write commands to the receiver (if write_raw is implemented)
echo '$PMTK314,0,1,0,0,0,0*...' > /dev/gnss0

# Check device type
cat /sys/class/gnss/gnss0/type    # e.g., "NMEA", "UBX", "SiRF", "MTK"
```

### gpsd integration

```sh
# gpsd can read directly from /dev/gnssN
gpsd /dev/gnss0 -N -D 2

# Monitor with cgps or gpsmon
cgps -s
gpsmon /dev/gnss0
```

### NMEA parsing

Typical NMEA sentences from a GNSS receiver:
```
$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,*47
$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
$GPGSV,2,1,08,01,40,083,46,02,17,308,41,12,07,344,39,14,22,228,45*75
```

### Kernel debug

```sh
# Module loading
modprobe gnss-<devname>
dmesg | grep gnss

# Verify device registration
ls -la /dev/gnss*
ls /sys/class/gnss/

# Runtime PM status
cat /sys/devices/.../power/runtime_status
```

---

## 10. Key Conventions

1. **License** -- use `GPL-2.0` (SPDX in first line: `// SPDX-License-Identifier: GPL-2.0`).
   Module macro: `MODULE_LICENSE("GPL v2")`.

2. **devm_* everywhere** -- use `devm_regulator_get()`,
   `devm_gpiod_get_optional()`, `devm_kzalloc()` so resources are
   automatically freed on driver unbind.

3. **Error propagation** -- return `PTR_ERR()` from failed `devm_*` calls.
   Use `IS_ERR()` to check. Use goto-based error unwinding in probe.

4. **gnss_put_device** -- always call `gnss_put_device()` (not `kfree`)
   on the `gnss_device` in the error path. The core manages its own memory
   via the device release callback.

5. **Power management** -- GNSS receivers are power-hungry; always
   implement runtime PM to power down the module when `/dev/gnssN` is not
   open.

6. **gnss_insert_raw serialization** -- the core requires that
   `gnss_insert_raw()` is not called for a closed device. Guard calls with
   a mutex and an `open` flag when managing serdev directly.

7. **Coding style** -- Linux kernel coding style (tabs, 80 columns,
   `checkpatch.pl` clean).

8. **Serial helper vs standalone** -- use the serial helper
   (`gnss_serial_allocate`) for simple receivers that only need
   power on/off. Use the standalone path (direct `gnss_allocate_device`)
   when the driver needs custom power sequencing, GPIO toggling, interrupt
   handling, or non-UART buses.

9. **Device type** -- set `gdev->type` to the appropriate `gnss_type`
   enum value. This appears in `/sys/class/gnss/gnssN/type` and in uevents.
   Use `GNSS_TYPE_NMEA` for generic NMEA-only receivers.

10. **OF match table** -- wrap in `#ifdef CONFIG_OF` and use
    `of_match_ptr()` in the driver struct.

---

## 11. Commit Message Format

```
gnss: add support for <Vendor> <Part> receiver

Add a driver for the <Vendor> <Part> multi-constellation GNSS receiver.
The <Part> communicates over UART and outputs standard NMEA sentences.
The driver uses the GNSS serial helper for power management and data
handling.

Signed-off-by: Your Name <your.name@example.com>
```

For DT binding additions:

```
dt-bindings: gnss: add <vendor>,<part> binding

Document the devicetree binding for the <Vendor> <Part> GNSS receiver,
including power supply, reset GPIO, and serial interface properties.

Signed-off-by: Your Name <your.name@example.com>
```
