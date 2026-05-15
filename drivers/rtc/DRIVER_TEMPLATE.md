# Linux RTC Driver Template

Reference driver: `drivers/rtc/rtc-rv3028.c`

This template covers every file needed to add a new I2C or SPI real-time
clock driver to the Linux RTC subsystem.  Replace `<devname>` with the chip
name (e.g., `rv3028`) and `<DEVNAME>` with its uppercase form (e.g.,
`RV3028`) throughout.

---

## 1. Purpose & Subsystem Mapping

RTC drivers live under `drivers/rtc/` and expose time/date/alarm
functionality through the **RTC class** (`struct rtc_class_ops`).

| Concept | Kernel API |
|---|---|
| Register operations | `struct rtc_class_ops` (`read_time`, `set_time`, `read_alarm`, `set_alarm`, `alarm_irq_enable`) |
| Device allocation | `devm_rtc_allocate_device()` |
| Device registration | `devm_rtc_register_device()` |
| Alarm notification | `rtc_update_irq()` |
| Time representation | `struct rtc_time` |
| BCD conversion | `bcd2bin()` / `bin2bcd()` from `<linux/bcd.h>` |
| Userspace interface | `/dev/rtcN`, `/sys/class/rtc/rtcN/`, `hwclock(8)` |

Typical use cases: battery-backed I2C or SPI real-time clock devices that
keep wall-clock time while the system is powered off.

---

## 2. File Checklist

```
drivers/rtc/
    rtc-<devname>.c                                  # Driver source

drivers/rtc/Kconfig                                  # Add RTC_DRV_<DEVNAME> entry
drivers/rtc/Makefile                                 # Add obj-$(CONFIG_RTC_DRV_<DEVNAME>)

Documentation/devicetree/bindings/rtc/
    <vendor>,<devname>.yaml                          # DT binding schema
```

---

## 3. Devicetree Binding (`.yaml`)

Create `Documentation/devicetree/bindings/rtc/<vendor>,<devname>.yaml`.
Reference the common RTC binding (`rtc.yaml`) via `allOf`.

```yaml
# SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
%YAML 1.2
---
$id: http://devicetree.org/schemas/rtc/<vendor>,<devname>.yaml#
$schema: http://devicetree.org/meta-schemas/core.yaml#

title: <Vendor> <DEVNAME> RTC

allOf:
  - $ref: rtc.yaml#

maintainers:
  - Your Name <your.email@example.com>

properties:
  compatible:
    const: <vendor>,<devname>

  reg:
    maxItems: 1

  interrupts:
    maxItems: 1
    description: Alarm interrupt, active low.

  "#clock-cells":
    const: 0

  # Trickle charger (if the device has one)
  trickle-resistor-ohms:
    enum:
      - 3000
      - 5000
      - 9000
      - 15000

  # Common RTC properties inherited from rtc.yaml:
  #   aux-voltage-chargeable, quartz-load-femtofarads,
  #   start-year, wakeup-source, reset-source

required:
  - compatible
  - reg

unevaluatedProperties: false

examples:
  - |
    #include <dt-bindings/interrupt-controller/irq.h>
    i2c {
        #address-cells = <1>;
        #size-cells = <0>;

        rtc@51 {
            compatible = "<vendor>,<devname>";
            reg = <0x51>;
            interrupts-extended = <&gpio1 16 IRQ_TYPE_LEVEL_LOW>;
            trickle-resistor-ohms = <3000>;
        };
    };

...
```

Key points:
- `allOf: - $ref: rtc.yaml#` inherits common RTC properties
  (`trickle-resistor-ohms`, `aux-voltage-chargeable`, `wakeup-source`, etc.).
- `interrupts` is optional: when absent the driver disables alarm support.
- `#clock-cells` is only needed when the device provides a clock output
  (e.g., 32.768 kHz CLKOUT pin).
- Run `make dt_binding_check DT_SCHEMA_FILES=rtc/<vendor>,<devname>.yaml`
  to validate.

---

## 4. Kconfig

Add an entry under the `if RTC_CLASS` block in `drivers/rtc/Kconfig`.
Place it alphabetically among existing entries.

```kconfig
config RTC_DRV_<DEVNAME>
	tristate "<Vendor> <DEVNAME>"
	depends on I2C        # or SPI_MASTER for SPI devices
	select REGMAP_I2C     # or REGMAP_SPI
	help
	  If you say yes here you get support for the <Vendor>
	  <DEVNAME> RTC.

	  This driver can also be built as a module. If so, the module
	  will be called rtc-<devname>.
```

Notes:
- `depends on I2C` or `depends on SPI_MASTER` matches the bus.
- `select REGMAP_I2C` (or `REGMAP_SPI`) pulls in regmap support
  automatically so users do not need to enable it by hand.
- Do not add `default y` -- RTC drivers are user-selected.

---

## 5. Makefile

Add one line to `drivers/rtc/Makefile`, alphabetically:

```makefile
obj-$(CONFIG_RTC_DRV_<DEVNAME>)	+= rtc-<devname>.o
```

---

## 6. Driver Source (`rtc-<devname>.c`)

### 6.1 Header and Includes

```c
// SPDX-License-Identifier: GPL-2.0
/*
 * RTC driver for the <Vendor> <DEVNAME>
 *
 * Copyright (C) YYYY <Vendor>
 *
 * Author Name <author.email@example.com>
 */

#include <linux/bcd.h>
#include <linux/i2c.h>           /* or <linux/spi/spi.h> */
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/rtc.h>
```

### 6.2 Register Map Definitions

```c
#define <DEVNAME>_REG_SEC          0x00
#define <DEVNAME>_REG_MIN          0x01
#define <DEVNAME>_REG_HOUR         0x02
#define <DEVNAME>_REG_WDAY         0x03
#define <DEVNAME>_REG_MDAY         0x04
#define <DEVNAME>_REG_MONTH        0x05
#define <DEVNAME>_REG_YEAR         0x06

#define <DEVNAME>_REG_ALARM_MIN    0x07
#define <DEVNAME>_REG_ALARM_HOUR   0x08
#define <DEVNAME>_REG_ALARM_DAY    0x09

#define <DEVNAME>_REG_STATUS       0x0E
#define <DEVNAME>_REG_CTRL1        0x0F
#define <DEVNAME>_REG_CTRL2        0x10

/* Status bits */
#define <DEVNAME>_STATUS_AF        BIT(2)   /* Alarm flag */

/* Control bits */
#define <DEVNAME>_CTRL2_AIE        BIT(3)   /* Alarm interrupt enable */
```

### 6.3 Private Data Structure

```c
struct <devname>_data {
	struct regmap      *regmap;
	struct rtc_device  *rtc;
};
```

### 6.4 `read_time` / `set_time`

Time registers are typically in BCD format.  Use `bcd2bin()` /
`bin2bcd()` from `<linux/bcd.h>` to convert.

```c
static int <devname>_get_time(struct device *dev, struct rtc_time *tm)
{
	struct <devname>_data *data = dev_get_drvdata(dev);
	u8 buf[7];
	int ret;

	ret = regmap_bulk_read(data->regmap, <DEVNAME>_REG_SEC,
			       buf, sizeof(buf));
	if (ret)
		return ret;

	tm->tm_sec  = bcd2bin(buf[0] & 0x7f);
	tm->tm_min  = bcd2bin(buf[1] & 0x7f);
	tm->tm_hour = bcd2bin(buf[2] & 0x3f);
	tm->tm_wday = buf[3] & 0x07;
	tm->tm_mday = bcd2bin(buf[4] & 0x3f);
	tm->tm_mon  = bcd2bin(buf[5] & 0x1f) - 1;
	tm->tm_year = bcd2bin(buf[6]) + 100;  /* Years since 1900 */

	return 0;
}

static int <devname>_set_time(struct device *dev, struct rtc_time *tm)
{
	struct <devname>_data *data = dev_get_drvdata(dev);
	u8 buf[7];

	buf[0] = bin2bcd(tm->tm_sec);
	buf[1] = bin2bcd(tm->tm_min);
	buf[2] = bin2bcd(tm->tm_hour);
	buf[3] = tm->tm_wday;
	buf[4] = bin2bcd(tm->tm_mday);
	buf[5] = bin2bcd(tm->tm_mon + 1);
	buf[6] = bin2bcd(tm->tm_year - 100);

	return regmap_bulk_write(data->regmap, <DEVNAME>_REG_SEC,
				 buf, sizeof(buf));
}
```

`struct rtc_time` fields:
- `tm_sec`  (0-59), `tm_min` (0-59), `tm_hour` (0-23)
- `tm_mday` (1-31), `tm_mon` (0-11), `tm_year` (years since 1900)
- `tm_wday` (0-6, Sunday = 0)

### 6.5 `read_alarm` / `set_alarm` / `alarm_irq_enable`

```c
static int <devname>_get_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct <devname>_data *data = dev_get_drvdata(dev);
	u8 alarmvals[3];
	int ret, status, ctrl;

	ret = regmap_bulk_read(data->regmap, <DEVNAME>_REG_ALARM_MIN,
			       alarmvals, sizeof(alarmvals));
	if (ret)
		return ret;

	ret = regmap_read(data->regmap, <DEVNAME>_REG_STATUS, &status);
	if (ret)
		return ret;

	ret = regmap_read(data->regmap, <DEVNAME>_REG_CTRL2, &ctrl);
	if (ret)
		return ret;

	alrm->time.tm_sec  = 0;
	alrm->time.tm_min  = bcd2bin(alarmvals[0] & 0x7f);
	alrm->time.tm_hour = bcd2bin(alarmvals[1] & 0x3f);
	alrm->time.tm_mday = bcd2bin(alarmvals[2] & 0x3f);

	alrm->enabled = !!(ctrl & <DEVNAME>_CTRL2_AIE);
	alrm->pending = (status & <DEVNAME>_STATUS_AF) && alrm->enabled;

	return 0;
}

static int <devname>_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct <devname>_data *data = dev_get_drvdata(dev);
	u8 alarmvals[3];
	int ret;

	/* Most RTCs have no seconds in alarm; round up to next minute */
	if (alrm->time.tm_sec) {
		time64_t alarm_time = rtc_tm_to_time64(&alrm->time);

		alarm_time += 60 - alrm->time.tm_sec;
		rtc_time64_to_tm(alarm_time, &alrm->time);
	}

	/* Disable alarm interrupt while reconfiguring */
	ret = regmap_update_bits(data->regmap, <DEVNAME>_REG_CTRL2,
				 <DEVNAME>_CTRL2_AIE, 0);
	if (ret)
		return ret;

	alarmvals[0] = bin2bcd(alrm->time.tm_min);
	alarmvals[1] = bin2bcd(alrm->time.tm_hour);
	alarmvals[2] = bin2bcd(alrm->time.tm_mday);

	/* Clear alarm flag */
	ret = regmap_update_bits(data->regmap, <DEVNAME>_REG_STATUS,
				 <DEVNAME>_STATUS_AF, 0);
	if (ret)
		return ret;

	ret = regmap_bulk_write(data->regmap, <DEVNAME>_REG_ALARM_MIN,
				alarmvals, sizeof(alarmvals));
	if (ret)
		return ret;

	/* Re-enable alarm interrupt if requested */
	if (alrm->enabled)
		ret = regmap_update_bits(data->regmap, <DEVNAME>_REG_CTRL2,
					 <DEVNAME>_CTRL2_AIE,
					 <DEVNAME>_CTRL2_AIE);

	return ret;
}

static int <devname>_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct <devname>_data *data = dev_get_drvdata(dev);

	return regmap_update_bits(data->regmap, <DEVNAME>_REG_CTRL2,
				  <DEVNAME>_CTRL2_AIE,
				  enabled ? <DEVNAME>_CTRL2_AIE : 0);
}
```

### 6.6 `rtc_class_ops`

```c
static const struct rtc_class_ops <devname>_rtc_ops = {
	.read_time        = <devname>_get_time,
	.set_time         = <devname>_set_time,
	.read_alarm       = <devname>_get_alarm,
	.set_alarm        = <devname>_set_alarm,
	.alarm_irq_enable = <devname>_alarm_irq_enable,
};
```

Additional optional ops:
- `.read_offset` / `.set_offset` -- frequency calibration
- `.ioctl` -- device-specific ioctls
- `.param_get` / `.param_set` -- backup switch mode, calibration params

### 6.7 Regmap Configuration

```c
static const struct regmap_config <devname>_regmap_config = {
	.reg_bits   = 8,
	.val_bits   = 8,
	.max_register = 0x37,    /* Adjust to last register address */
};
```

### 6.8 Probe Function

```c
static int <devname>_probe(struct i2c_client *client)
{
	struct <devname>_data *data;
	int ret;

	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->regmap = devm_regmap_init_i2c(client, &<devname>_regmap_config);
	if (IS_ERR(data->regmap))
		return PTR_ERR(data->regmap);

	i2c_set_clientdata(client, data);

	/* ---- Allocate RTC device ---- */
	data->rtc = devm_rtc_allocate_device(&client->dev);
	if (IS_ERR(data->rtc))
		return PTR_ERR(data->rtc);

	/* ---- IRQ / alarm setup ---- */
	if (client->irq > 0) {
		unsigned long flags;

		if (dev_fwnode(&client->dev))
			flags = 0;    /* Use flags from DT */
		else
			flags = IRQF_TRIGGER_LOW;

		ret = devm_request_threaded_irq(&client->dev, client->irq,
						NULL, <devname>_handle_irq,
						flags | IRQF_ONESHOT,
						"<devname>", data);
		if (ret) {
			dev_warn(&client->dev,
				 "unable to request IRQ, alarms disabled\n");
			client->irq = 0;
		}
	}
	if (!client->irq)
		clear_bit(RTC_FEATURE_ALARM, data->rtc->features);

	/* ---- Parse DT properties (trickle charger, etc.) ---- */
	<devname>_parse_dt(data, client);

	/* ---- Set valid time range ---- */
	data->rtc->range_min = RTC_TIMESTAMP_BEGIN_2000;
	data->rtc->range_max = RTC_TIMESTAMP_END_2099;
	data->rtc->ops = &<devname>_rtc_ops;

	/* ---- Register RTC device ---- */
	ret = devm_rtc_register_device(data->rtc);
	if (ret)
		return ret;

	return 0;
}
```

Key probe sequence:
1. `devm_kzalloc()` -- allocate private data.
2. `devm_regmap_init_i2c()` -- create regmap (or `_spi` for SPI devices).
3. `devm_rtc_allocate_device()` -- allocate the `rtc_device`.
4. `devm_request_threaded_irq()` -- set up alarm IRQ (if available).
5. Parse devicetree properties (trickle charger, clock output, etc.).
6. Set `range_min` / `range_max` and assign `ops`.
7. `devm_rtc_register_device()` -- register with the RTC subsystem.

### 6.9 ID Tables and Module Registration

```c
static const struct of_device_id <devname>_of_match[] = {
	{ .compatible = "<vendor>,<devname>" },
	{ }
};
MODULE_DEVICE_TABLE(of, <devname>_of_match);

static const struct i2c_device_id <devname>_id_table[] = {
	{ .name = "<devname>" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, <devname>_id_table);

static struct i2c_driver <devname>_driver = {
	.driver = {
		.name           = "rtc-<devname>",
		.of_match_table = of_match_ptr(<devname>_of_match),
	},
	.id_table = <devname>_id_table,
	.probe    = <devname>_probe,
};
module_i2c_driver(<devname>_driver);

MODULE_AUTHOR("Your Name <your.email@example.com>");
MODULE_DESCRIPTION("<Vendor> <DEVNAME> RTC driver");
MODULE_LICENSE("GPL");
```

For SPI devices, use `struct spi_driver`, `module_spi_driver()`,
`spi_device_id`, and `devm_regmap_init_spi()` instead.

---

## 7. Alarm & IRQ Support

### 7.1 IRQ Handler

```c
static irqreturn_t <devname>_handle_irq(int irq, void *dev_id)
{
	struct <devname>_data *data = dev_id;
	unsigned long events = 0;
	u32 status = 0;

	if (regmap_read(data->regmap, <DEVNAME>_REG_STATUS, &status) < 0 ||
	    status == 0)
		return IRQ_NONE;

	if (status & <DEVNAME>_STATUS_AF) {
		events |= RTC_AF | RTC_IRQF;
	}

	if (events) {
		rtc_update_irq(data->rtc, 1, events);

		/* Clear handled flags */
		regmap_update_bits(data->regmap, <DEVNAME>_REG_STATUS,
				   status & <DEVNAME>_STATUS_AF, 0);

		/* Disable the fired interrupt source */
		regmap_update_bits(data->regmap, <DEVNAME>_REG_CTRL2,
				   <DEVNAME>_CTRL2_AIE, 0);
	}

	return IRQ_HANDLED;
}
```

### 7.2 Alarm Feature Negotiation

- If no IRQ is provided (no `interrupts` in DT), call
  `clear_bit(RTC_FEATURE_ALARM, rtc->features)` in probe to disable
  alarm support gracefully.
- `rtc_update_irq(rtc, 1, events)` wakes any process blocked on
  `read()` / `poll()` of `/dev/rtcN` and triggers the alarm.
- Event flags: `RTC_AF` (alarm), `RTC_UF` (update), `RTC_PF` (periodic),
  `RTC_IRQF` (interrupt fired -- always OR this in).

---

## 8. Devicetree Parsing

### 8.1 Trickle Charger

Many RTCs have a trickle charger for a supercap or rechargeable battery.
Use the standard `trickle-resistor-ohms` and `aux-voltage-chargeable`
properties defined in `rtc.yaml`.

```c
static int <devname>_parse_dt(struct <devname>_data *data,
			      struct i2c_client *client)
{
	u32 ohms, chargeable;

	/* Trickle charger resistor */
	if (!device_property_read_u32(&client->dev,
				      "trickle-resistor-ohms", &ohms)) {
		/* Map ohms value to register field and program it */
	}

	/* Backup power chargeable flag */
	if (!device_property_read_u32(&client->dev,
				      "aux-voltage-chargeable",
				      &chargeable)) {
		/* 0 = not chargeable, 1 = chargeable */
	}

	return 0;
}
```

### 8.2 Clock Output

If the device has a configurable CLKOUT pin (e.g., 32.768 kHz square
wave), register it via the Common Clock Framework:

```c
#ifdef CONFIG_COMMON_CLK
#include <linux/clk-provider.h>

/* Implement clk_hw ops: .recalc_rate, .is_enabled, .enable, .disable */

static void <devname>_clkout_register(struct <devname>_data *data,
				      struct i2c_client *client)
{
	struct clk *clk;
	struct device_node *node = client->dev.of_node;

	data->clkout_hw.init = &(struct clk_init_data){
		.name  = "<devname>-clkout",
		.ops   = &<devname>_clkout_ops,
	};

	clk = devm_clk_register(&client->dev, &data->clkout_hw);
	if (!IS_ERR(clk))
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
}
#endif
```

The DT binding must include `#clock-cells = <0>` when the clock output
is present.

---

## 9. Test & Debug

### 9.1 Verify Device Node

```sh
ls -l /dev/rtc*
# Expect: /dev/rtc0 -> rtc0 (or /dev/rtc1 if another RTC exists)

cat /sys/class/rtc/rtc0/name
# Should print the driver name (e.g., "rtc-rv3028")
```

### 9.2 Read/Set Time

```sh
# Read hardware clock
hwclock -r --rtc /dev/rtc0

# Set hardware clock from system time
hwclock -w --rtc /dev/rtc0

# Set hardware clock to a specific time
hwclock --set --date="2025-06-15 12:00:00" --rtc /dev/rtc0
```

### 9.3 Test Alarm

```sh
# Read current alarm
cat /sys/class/rtc/rtc0/wakealarm

# Set alarm 30 seconds from now
echo "+30" > /sys/class/rtc/rtc0/wakealarm

# Wait for alarm (poll /dev/rtc0 or check dmesg)
```

### 9.4 Sysfs Attributes

```sh
ls /sys/class/rtc/rtc0/
# date, time, since_epoch, max_user_freq, wakealarm, offset, ...
```

### 9.5 Kernel Messages

```sh
dmesg | grep rtc
# Look for probe success / failure, oscillator stop warnings, etc.
```

---

## 10. Key Conventions

| Convention | Detail |
|---|---|
| License | `GPL-2.0` (SPDX in first line) |
| BCD helpers | `bcd2bin()` / `bin2bcd()` from `<linux/bcd.h>` -- never do manual BCD math |
| Memory allocation | Always `devm_*` variants: `devm_kzalloc`, `devm_regmap_init_i2c`, `devm_request_threaded_irq`, `devm_rtc_allocate_device`, `devm_rtc_register_device` |
| Range setting | Set `rtc->range_min` and `rtc->range_max` before `devm_rtc_register_device()` (e.g., `RTC_TIMESTAMP_BEGIN_2000` / `RTC_TIMESTAMP_END_2099`) |
| No `.remove` needed | `devm_*` resources are cleaned up automatically; a `.remove` callback is not required |
| 12/24-hour mode | Force the hardware to 24-hour mode during probe; the RTC core handles user-facing conversion |
| Oscillator stop flag | Check the oscillator-stop flag (OSF / PORF) in `read_time` and return `-EINVAL` if set -- the stored time is unreliable |
| IRQ flags | Use `IRQF_ONESHOT` for threaded IRQ handlers; use `flags = 0` when the IRQ configuration comes from devicetree |
| Module macros | `module_i2c_driver()` (or `module_spi_driver()`) eliminates boilerplate `init`/`exit` |
| Header ordering | `#include` headers in alphabetical order |
| Coding style | Follow `Documentation/process/coding-style.rst` (tabs, 80-col soft limit, kernel naming conventions) |

---

## 11. Commit Message Format

Use the `rtc:` subsystem prefix.

```
rtc: <devname>: add driver for <Vendor> <DEVNAME>

Add support for the <Vendor> <DEVNAME> I2C real-time clock.
Features include time/date, alarms, and trickle-charger
configuration via devicetree.

Signed-off-by: Your Name <your.email@example.com>
```

For DT bindings submitted as a separate patch:

```
dt-bindings: rtc: add <vendor>,<devname>

Add devicetree binding documentation for the <Vendor> <DEVNAME>
I2C real-time clock.

Signed-off-by: Your Name <your.email@example.com>
```

A typical series for a new RTC driver is two patches:
1. `dt-bindings: rtc: add <vendor>,<devname>` -- binding YAML only.
2. `rtc: <devname>: add driver for <Vendor> <DEVNAME>` -- Kconfig +
   Makefile + driver source.
