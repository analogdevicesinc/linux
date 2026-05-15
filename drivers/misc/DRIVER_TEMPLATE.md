# Linux Misc Driver Template

This template covers no-OS device types that do not map to a standard Linux
kernel subsystem (IIO, HWMON, input, etc.).  These devices typically end up in
`drivers/misc/`, `drivers/nvmem/`, `drivers/auxdisplay/`, `drivers/media/`,
or as custom `platform_driver` / `spi_driver` / `i2c_driver` implementations
with a sysfs or chardev userspace interface.

---

## Device Type Reference

| no-OS Type | Linux Subsystem / Location | Driver Base | Userspace Interface | Reference |
|---|---|---|---|---|
| motor/stepper | `drivers/misc/` or custom platform | `spi_driver` / `platform_driver` | sysfs + PWM | Custom motor control sysfs |
| mcs (machine control) | `drivers/misc/` | `platform_driver` | sysfs (GPIO-exported signals) | `adi-axi-tdd.c` pattern |
| io-link | `drivers/misc/` | `spi_driver` | chardev (`miscdevice`) or sysfs | Custom SPI protocol |
| gmsl (ser/des) | `drivers/media/i2c/` or `drivers/misc/` | `i2c_driver` | V4L2 subdev or sysfs | `max9286.c`, `max96714.c` |
| eeprom | `drivers/misc/eeprom/` or `drivers/nvmem/` | `i2c_driver` / `spi_driver` | nvmem sysfs (`/sys/bus/nvmem/`) | `at24.c`, `at25.c` |
| display | `drivers/auxdisplay/` or `drivers/gpu/drm/` | `spi_driver` / `i2c_driver` | `charlcd` framework or DRM | `lcd2s.c`, `hd44780.c` |

---

## 1. Purpose & Subsystem Mapping

Devices in this category lack a well-defined Linux subsystem with a standard
API (unlike IIO for ADCs/DACs, or HWMON for temperature sensors).  The
approach for each device type:

**Motor/stepper controllers** -- Implement as a `spi_driver` (or
`platform_driver` for FPGA-based controllers).  Expose motion parameters
(velocity, position, acceleration, ramp mode) through custom sysfs attributes.
If the controller uses PWM outputs, integrate with the kernel PWM subsystem
(`drivers/pwm/`).

**MCS (machine control signals)** -- Implement as a `platform_driver` with
GPIO descriptors.  Expose synchronisation signals through sysfs.  When
part of a JESD204 topology, the driver coordinates with other subsystem
drivers via shared GPIOs or interrupt lines.

**IO-Link** -- Implement as a `spi_driver` with a `miscdevice` registration
for the IO-Link master protocol.  Userspace communicates through a chardev
interface using `read()`/`write()` or `ioctl()` for ISDU (Indexed Service
Data Unit) transactions.

**GMSL (serializer/deserializer)** -- Implement as an `i2c_driver`.  For
camera applications, register as a V4L2 subdevice under `drivers/media/i2c/`.
For non-camera use cases (e.g., display serialisation, sensor aggregation),
use `drivers/misc/` with sysfs attributes.

**EEPROM** -- Implement as an `i2c_driver` or `spi_driver` registering with
the nvmem subsystem (`nvmem-provider.h`).  The nvmem framework provides a
standard sysfs interface at `/sys/bus/nvmem/devices/*/nvmem` and an in-kernel
API for other drivers to consume cells.

**Display** -- Implement as an `i2c_driver` or `spi_driver`.  Character LCDs
use the `charlcd` framework in `drivers/auxdisplay/`.  Small OLED/TFT displays
use the DRM subsystem with `drm_simple_display_pipe` helpers.

---

## 2. File Checklist

```
drivers/misc/<devname>/               # or drivers/nvmem/, drivers/auxdisplay/, etc.
    <devname>.c                        # Driver source
    Kconfig                            # Build configuration
    Makefile                           # Build rule

Documentation/devicetree/bindings/
    misc/<vendor>,<devname>.yaml       # DT binding (or eeprom/, auxdisplay/, etc.)

include/uapi/linux/<devname>.h         # Optional: userspace ioctl definitions
```

For nvmem-based EEPROM drivers, the binding YAML goes under
`Documentation/devicetree/bindings/eeprom/` or
`Documentation/devicetree/bindings/nvmem/`.

---

## 3. Devicetree Binding (`.yaml`)

### 3.1 GPIO-controlled platform device (MCS / motor control)

```yaml
# SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)
# Copyright 2024 Analog Devices Inc.
%YAML 1.2
---
$id: http://devicetree.org/schemas/misc/adi,<devname>.yaml#
$schema: http://devicetree.org/meta-schemas/core.yaml#

title: Analog Devices <DEVNAME> Controller

maintainers:
  - Your Name <your.name@analog.com>

description: |
  The <DEVNAME> is a ... <brief description>.

properties:
  compatible:
    enum:
      - adi,<devname>

  reg:
    maxItems: 1

  clocks:
    items:
      - description: AXI bus clock
      - description: Interface clock

  clock-names:
    items:
      - const: s_axi_aclk
      - const: intf_clk

  reset-gpios:
    maxItems: 1
    description: Active-low hardware reset.

required:
  - compatible
  - reg
  - clocks
  - clock-names

unevaluatedProperties: false

examples:
  - |
    <devname>@84a00000 {
        compatible = "adi,<devname>";
        reg = <0x84a00000 0x10000>;
        clocks = <&clk_bus>, <&clk_intf>;
        clock-names = "s_axi_aclk", "intf_clk";
    };
...
```

### 3.2 SPI device (IO-Link master)

```yaml
# SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)
%YAML 1.2
---
$id: http://devicetree.org/schemas/misc/adi,<devname>.yaml#
$schema: http://devicetree.org/meta-schemas/core.yaml#

title: Analog Devices <DEVNAME> IO-Link Master

maintainers:
  - Your Name <your.name@analog.com>

properties:
  compatible:
    enum:
      - adi,<devname>

  reg:
    maxItems: 1

  spi-max-frequency:
    maximum: 10000000

  interrupts:
    maxItems: 1

  reset-gpios:
    maxItems: 1

required:
  - compatible
  - reg

additionalProperties: false

examples:
  - |
    #include <dt-bindings/interrupt-controller/irq.h>
    spi {
        #address-cells = <1>;
        #size-cells = <0>;

        iolink@0 {
            compatible = "adi,<devname>";
            reg = <0>;
            spi-max-frequency = <10000000>;
            interrupt-parent = <&gpio>;
            interrupts = <25 IRQ_TYPE_EDGE_FALLING>;
            reset-gpios = <&gpio 10 GPIO_ACTIVE_LOW>;
        };
    };
...
```

### 3.3 I2C device (EEPROM via nvmem)

```yaml
# SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)
%YAML 1.2
---
$id: http://devicetree.org/schemas/eeprom/adi,<devname>.yaml#
$schema: http://devicetree.org/meta-schemas/core.yaml#

title: Analog Devices <DEVNAME> EEPROM

maintainers:
  - Your Name <your.name@analog.com>

allOf:
  - $ref: /schemas/nvmem/nvmem.yaml

properties:
  compatible:
    enum:
      - adi,<devname>

  reg:
    maxItems: 1

  pagesize:
    description: Size of a page in bytes (write granularity).
    $ref: /schemas/types.yaml#/definitions/uint32
    enum: [8, 16, 32, 64, 128, 256]

  size:
    description: Total size of the EEPROM in bytes.
    $ref: /schemas/types.yaml#/definitions/uint32

  read-only:
    description: Mark the EEPROM as read-only.
    type: boolean

  vcc-supply:
    description: Power supply regulator.

required:
  - compatible
  - reg

unevaluatedProperties: false

examples:
  - |
    i2c {
        #address-cells = <1>;
        #size-cells = <0>;

        eeprom@50 {
            compatible = "adi,<devname>";
            reg = <0x50>;
            pagesize = <32>;
            size = <4096>;
        };
    };
...
```

---

## 4. Kconfig

### 4.1 Standalone misc device

```kconfig
# SPDX-License-Identifier: GPL-2.0-only
config ADI_<DEVNAME>
	tristate "Analog Devices <DEVNAME> support"
	depends on SPI_MASTER
	select REGMAP_SPI
	help
	  Say yes here to build support for the Analog Devices <DEVNAME>
	  <brief one-line description>.

	  To compile this driver as a module, choose M here: the
	  module will be called <devname>.
```

### 4.2 EEPROM (under `drivers/misc/eeprom/`)

```kconfig
config EEPROM_<DEVNAME>
	tristate "Analog Devices <DEVNAME> EEPROM"
	depends on I2C && SYSFS
	select NVMEM
	select NVMEM_SYSFS
	select REGMAP_I2C
	help
	  Enable read/write support for the Analog Devices <DEVNAME>
	  I2C EEPROM (<size> of non-volatile storage).

	  This driver can also be built as a module.  If so, the module
	  will be called <devname>.
```

### 4.3 Platform/AXI device

```kconfig
config ADI_AXI_<DEVNAME>
	tristate "Analog Devices AXI <DEVNAME>"
	depends on HAS_IOMEM
	depends on OF
	select REGMAP_MMIO
	help
	  Say yes here to build support for the Analog Devices AXI
	  <DEVNAME> core.  This is part of the ADI HDL reference designs.

	  To compile this driver as a module, choose M here: the
	  module will be called adi-axi-<devname>.
```

---

## 5. Makefile

```makefile
# SPDX-License-Identifier: GPL-2.0
obj-$(CONFIG_ADI_<DEVNAME>)	+= <devname>.o
```

For multi-file drivers:
```makefile
obj-$(CONFIG_ADI_<DEVNAME>)	+= adi-<devname>.o
adi-<devname>-y			:= <devname>-core.o <devname>-spi.o
```

When adding to an existing directory (e.g., `drivers/misc/Makefile`),
append to the existing file:
```makefile
obj-$(CONFIG_ADI_<DEVNAME>)	+= <devname>.o
```

---

## 6. Driver Source (`.c`)

### 6.1 Platform driver skeleton (MCS / AXI peripherals)

Based on `adi-axi-tdd.c` -- a memory-mapped platform device with sysfs.

```c
// SPDX-License-Identifier: GPL-2.0
/*
 * <DEVNAME> driver
 *
 * Copyright 2024 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/sysfs.h>

/* ---- Register Map ------------------------------------------------- */
#define <DEVNAME>_REG_VERSION		0x0000
#define <DEVNAME>_REG_SCRATCH		0x0008
#define <DEVNAME>_REG_CONTROL		0x0040

/* Control Register Bits */
#define <DEVNAME>_ENABLE		BIT(0)

/* ---- Driver State ------------------------------------------------- */
struct <devname>_state {
	struct regmap *regs;
	struct clk *clk;
	struct mutex lock;	/* protects register access */
};

/* ---- Regmap Config ------------------------------------------------ */
static const struct regmap_config <devname>_regmap_cfg = {
	.name = "<devname>",
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
};

/* ---- Sysfs Attributes --------------------------------------------- */
static ssize_t enable_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct <devname>_state *st = dev_get_drvdata(dev);
	u32 val;
	int ret;

	ret = regmap_read(st->regs, <DEVNAME>_REG_CONTROL, &val);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%u\n", !!(val & <DEVNAME>_ENABLE));
}

static ssize_t enable_store(struct device *dev,
			    struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct <devname>_state *st = dev_get_drvdata(dev);
	bool enable;
	int ret;

	ret = kstrtobool(buf, &enable);
	if (ret)
		return ret;

	mutex_lock(&st->lock);
	ret = regmap_update_bits(st->regs, <DEVNAME>_REG_CONTROL,
				 <DEVNAME>_ENABLE,
				 enable ? <DEVNAME>_ENABLE : 0);
	mutex_unlock(&st->lock);

	return ret ?: count;
}
static DEVICE_ATTR_RW(enable);

static struct attribute *<devname>_attrs[] = {
	&dev_attr_enable.attr,
	NULL,
};
ATTRIBUTE_GROUPS(<devname>);

/* ---- Probe / Remove ----------------------------------------------- */
static int <devname>_probe(struct platform_device *pdev)
{
	struct <devname>_state *st;
	void __iomem *base;
	int ret;

	st = devm_kzalloc(&pdev->dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	st->regs = devm_regmap_init_mmio(&pdev->dev, base,
					 &<devname>_regmap_cfg);
	if (IS_ERR(st->regs))
		return PTR_ERR(st->regs);

	st->clk = devm_clk_get_enabled(&pdev->dev, "s_axi_aclk");
	if (IS_ERR(st->clk))
		return PTR_ERR(st->clk);

	mutex_init(&st->lock);
	platform_set_drvdata(pdev, st);

	ret = devm_device_add_groups(&pdev->dev, <devname>_groups);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "Failed to create sysfs attributes\n");

	dev_info(&pdev->dev, "Probed successfully\n");

	return 0;
}

/* ---- Match Table -------------------------------------------------- */
static const struct of_device_id <devname>_of_match[] = {
	{ .compatible = "adi,<devname>" },
	{ }
};
MODULE_DEVICE_TABLE(of, <devname>_of_match);

static struct platform_driver <devname>_driver = {
	.driver = {
		.name = "<devname>",
		.of_match_table = <devname>_of_match,
	},
	.probe = <devname>_probe,
};
module_platform_driver(<devname>_driver);

MODULE_AUTHOR("Your Name <your.name@analog.com>");
MODULE_DESCRIPTION("Analog Devices <DEVNAME> driver");
MODULE_LICENSE("GPL");
```

### 6.2 EEPROM / nvmem pattern (I2C)

Based on `at24.c` -- an I2C EEPROM registered with the nvmem subsystem.

```c
// SPDX-License-Identifier: GPL-2.0
/*
 * <DEVNAME> I2C EEPROM driver
 *
 * Copyright 2024 Analog Devices Inc.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nvmem-provider.h>
#include <linux/property.h>
#include <linux/regmap.h>

#define <DEVNAME>_PAGE_SIZE	32
#define <DEVNAME>_SIZE		4096
#define <DEVNAME>_WRITE_TIME_MS	5

struct <devname>_data {
	struct mutex lock;
	struct regmap *regmap;
	struct nvmem_device *nvmem;
	unsigned int page_size;
	unsigned int size;
};

static int <devname>_read(void *priv, unsigned int off,
			  void *val, size_t count)
{
	struct <devname>_data *data = priv;
	int ret;

	mutex_lock(&data->lock);
	ret = regmap_bulk_read(data->regmap, off, val, count);
	mutex_unlock(&data->lock);

	return ret;
}

static int <devname>_write(void *priv, unsigned int off,
			   void *val, size_t count)
{
	struct <devname>_data *data = priv;
	unsigned int page_off, chunk;
	u8 *buf = val;
	int ret;

	mutex_lock(&data->lock);
	while (count) {
		page_off = off % data->page_size;
		chunk = min_t(size_t, count, data->page_size - page_off);

		ret = regmap_bulk_write(data->regmap, off, buf, chunk);
		if (ret)
			break;

		/* Wait for internal write cycle. */
		msleep(<DEVNAME>_WRITE_TIME_MS);

		off += chunk;
		buf += chunk;
		count -= chunk;
	}
	mutex_unlock(&data->lock);

	return ret;
}

static int <devname>_probe(struct i2c_client *client)
{
	struct <devname>_data *data;
	struct nvmem_config nvmem_cfg = {};
	struct regmap_config regmap_cfg = {
		.reg_bits = 16,
		.val_bits = 8,
	};

	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	mutex_init(&data->lock);

	data->regmap = devm_regmap_init_i2c(client, &regmap_cfg);
	if (IS_ERR(data->regmap))
		return PTR_ERR(data->regmap);

	data->page_size = <DEVNAME>_PAGE_SIZE;
	data->size = <DEVNAME>_SIZE;

	/* Allow DT to override defaults. */
	device_property_read_u32(&client->dev, "pagesize", &data->page_size);
	device_property_read_u32(&client->dev, "size", &data->size);

	nvmem_cfg.dev = &client->dev;
	nvmem_cfg.name = "<devname>";
	nvmem_cfg.id = NVMEM_DEVID_NONE;
	nvmem_cfg.type = NVMEM_TYPE_EEPROM;
	nvmem_cfg.reg_read = <devname>_read;
	nvmem_cfg.reg_write = <devname>_write;
	nvmem_cfg.size = data->size;
	nvmem_cfg.word_size = 1;
	nvmem_cfg.stride = 1;
	nvmem_cfg.priv = data;
	nvmem_cfg.read_only = device_property_read_bool(&client->dev,
							"read-only");

	data->nvmem = devm_nvmem_register(&client->dev, &nvmem_cfg);
	if (IS_ERR(data->nvmem))
		return dev_err_probe(&client->dev, PTR_ERR(data->nvmem),
				     "Failed to register nvmem\n");

	dev_info(&client->dev, "EEPROM registered (%u bytes, %u-byte pages)\n",
		 data->size, data->page_size);

	return 0;
}

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
MODULE_DESCRIPTION("Analog Devices <DEVNAME> EEPROM driver");
MODULE_LICENSE("GPL");
```

### 6.3 SPI device with miscdevice chardev (IO-Link)

For custom protocols that need a userspace character device interface.

```c
// SPDX-License-Identifier: GPL-2.0
/*
 * <DEVNAME> IO-Link master driver
 *
 * Copyright 2024 Analog Devices Inc.
 */

#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>

struct <devname>_data {
	struct spi_device *spi;
	struct miscdevice miscdev;
	struct mutex lock;	/* serialise userspace access */
	u8 rx_buf[256];
	u8 tx_buf[256];
};

static int <devname>_open(struct inode *inode, struct file *filp)
{
	struct <devname>_data *data = container_of(filp->private_data,
						   struct <devname>_data,
						   miscdev);
	filp->private_data = data;
	return 0;
}

static ssize_t <devname>_read(struct file *filp, char __user *buf,
			      size_t count, loff_t *ppos)
{
	struct <devname>_data *data = filp->private_data;
	int ret;

	if (count > sizeof(data->rx_buf))
		count = sizeof(data->rx_buf);

	mutex_lock(&data->lock);
	ret = spi_read(data->spi, data->rx_buf, count);
	mutex_unlock(&data->lock);

	if (ret)
		return ret;

	if (copy_to_user(buf, data->rx_buf, count))
		return -EFAULT;

	return count;
}

static ssize_t <devname>_write(struct file *filp, const char __user *buf,
			       size_t count, loff_t *ppos)
{
	struct <devname>_data *data = filp->private_data;
	int ret;

	if (count > sizeof(data->tx_buf))
		return -EINVAL;

	if (copy_from_user(data->tx_buf, buf, count))
		return -EFAULT;

	mutex_lock(&data->lock);
	ret = spi_write(data->spi, data->tx_buf, count);
	mutex_unlock(&data->lock);

	return ret ?: count;
}

static const struct file_operations <devname>_fops = {
	.owner = THIS_MODULE,
	.open = <devname>_open,
	.read = <devname>_read,
	.write = <devname>_write,
};

static int <devname>_probe(struct spi_device *spi)
{
	struct <devname>_data *data;

	data = devm_kzalloc(&spi->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->spi = spi;
	mutex_init(&data->lock);

	data->miscdev.minor = MISC_DYNAMIC_MINOR;
	data->miscdev.name = dev_name(&spi->dev);
	data->miscdev.fops = &<devname>_fops;
	data->miscdev.parent = &spi->dev;

	spi_set_drvdata(spi, data);

	return devm_misc_register(&spi->dev, &data->miscdev);
}

static const struct of_device_id <devname>_of_match[] = {
	{ .compatible = "adi,<devname>" },
	{ }
};
MODULE_DEVICE_TABLE(of, <devname>_of_match);

static const struct spi_device_id <devname>_spi_id[] = {
	{ "<devname>" },
	{ }
};
MODULE_DEVICE_TABLE(spi, <devname>_spi_id);

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
MODULE_DESCRIPTION("Analog Devices <DEVNAME> IO-Link master driver");
MODULE_LICENSE("GPL");
```

### 6.4 PWM motor control pattern

For motor controllers that expose PWM outputs and motion parameters.

```c
// SPDX-License-Identifier: GPL-2.0
/*
 * <DEVNAME> motor controller driver
 *
 * Copyright 2024 Analog Devices Inc.
 */

#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>
#include <linux/sysfs.h>

/* ---- Register Map ------------------------------------------------- */
#define <DEVNAME>_REG_STATUS		0x00
#define <DEVNAME>_REG_VELOCITY		0x04
#define <DEVNAME>_REG_POSITION		0x08
#define <DEVNAME>_REG_ACCEL		0x0C
#define <DEVNAME>_REG_MODE		0x10

#define <DEVNAME>_MODE_MSK		GENMASK(1, 0)
#define <DEVNAME>_MODE_VELOCITY		0
#define <DEVNAME>_MODE_POSITION		1

struct <devname>_state {
	struct spi_device *spi;
	struct regmap *regmap;
	struct mutex lock;
};

static const struct regmap_config <devname>_regmap_cfg = {
	.reg_bits = 8,
	.val_bits = 32,
	.max_register = 0x7F,
};

/* ---- Sysfs Attributes --------------------------------------------- */
static ssize_t velocity_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct <devname>_state *st = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = regmap_read(st->regmap, <DEVNAME>_REG_VELOCITY, &val);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%u\n", val);
}

static ssize_t velocity_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct <devname>_state *st = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;

	mutex_lock(&st->lock);
	ret = regmap_write(st->regmap, <DEVNAME>_REG_VELOCITY, val);
	mutex_unlock(&st->lock);

	return ret ?: count;
}
static DEVICE_ATTR_RW(velocity);

static ssize_t position_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct <devname>_state *st = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = regmap_read(st->regmap, <DEVNAME>_REG_POSITION, &val);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%u\n", val);
}

static ssize_t position_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct <devname>_state *st = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;

	mutex_lock(&st->lock);
	ret = regmap_write(st->regmap, <DEVNAME>_REG_POSITION, val);
	mutex_unlock(&st->lock);

	return ret ?: count;
}
static DEVICE_ATTR_RW(position);

static struct attribute *<devname>_attrs[] = {
	&dev_attr_velocity.attr,
	&dev_attr_position.attr,
	NULL,
};
ATTRIBUTE_GROUPS(<devname>);

/* ---- Probe -------------------------------------------------------- */
static int <devname>_probe(struct spi_device *spi)
{
	struct <devname>_state *st;

	st = devm_kzalloc(&spi->dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->spi = spi;
	mutex_init(&st->lock);

	st->regmap = devm_regmap_init_spi(spi, &<devname>_regmap_cfg);
	if (IS_ERR(st->regmap))
		return PTR_ERR(st->regmap);

	spi_set_drvdata(spi, st);

	return devm_device_add_groups(&spi->dev, <devname>_groups);
}

static const struct of_device_id <devname>_of_match[] = {
	{ .compatible = "adi,<devname>" },
	{ }
};
MODULE_DEVICE_TABLE(of, <devname>_of_match);

static const struct spi_device_id <devname>_spi_id[] = {
	{ "<devname>" },
	{ }
};
MODULE_DEVICE_TABLE(spi, <devname>_spi_id);

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
MODULE_DESCRIPTION("Analog Devices <DEVNAME> motor controller driver");
MODULE_LICENSE("GPL");
```

---

## 7. Userspace Interface

### 7.1 Sysfs attributes

The preferred approach for simple control interfaces.  Each attribute maps
to a register or computed value.

```
/sys/bus/platform/devices/<devname>/
    enable              # RW: 0 or 1
    velocity            # RW: integer
    position            # RW: integer
    version             # RO: x.y.z
    status              # RO: integer
```

Use `DEVICE_ATTR_RW()`, `DEVICE_ATTR_RO()`, and `DEVICE_ATTR_WO()` macros.
Always use `sysfs_emit()` (not `sprintf`) for output.  Always validate input
with `kstrtouint()`, `kstrtobool()`, etc.

### 7.2 Chardev (`miscdevice` + `file_operations`)

For byte-stream or message-oriented protocols (IO-Link, custom framing).

```c
static const struct file_operations <devname>_fops = {
	.owner          = THIS_MODULE,
	.open           = <devname>_open,
	.release        = <devname>_release,
	.read           = <devname>_read,
	.write          = <devname>_write,
	.unlocked_ioctl = <devname>_ioctl,
};
```

Register with `devm_misc_register()`.  The device node appears at
`/dev/<devname>`.  Define ioctl commands in a UAPI header:

```c
/* include/uapi/linux/<devname>.h */
#ifndef _UAPI_<DEVNAME>_H_
#define _UAPI_<DEVNAME>_H_

#include <linux/ioctl.h>
#include <linux/types.h>

#define <DEVNAME>_IOC_MAGIC	'X'

struct <devname>_transfer {
	__u8 port;
	__u16 index;
	__u16 subindex;
	__u16 length;
	__u8 data[256];
};

#define <DEVNAME>_IOC_ISDU_READ	_IOWR(<DEVNAME>_IOC_MAGIC, 0x01, \
					      struct <devname>_transfer)
#define <DEVNAME>_IOC_ISDU_WRITE	_IOW(<DEVNAME>_IOC_MAGIC, 0x02, \
					     struct <devname>_transfer)
#define <DEVNAME>_IOC_GET_STATUS	_IOR(<DEVNAME>_IOC_MAGIC, 0x03, __u32)

#endif /* _UAPI_<DEVNAME>_H_ */
```

### 7.3 nvmem sysfs (EEPROM)

The nvmem subsystem provides the interface automatically:

```
/sys/bus/nvmem/devices/<devname>0/nvmem    # binary file, read/write
```

Read/write with standard file tools:
```bash
# Read first 32 bytes
hexdump -C /sys/bus/nvmem/devices/<devname>0/nvmem | head

# Write data at offset 0
echo -n "hello" | dd of=/sys/bus/nvmem/devices/<devname>0/nvmem bs=1
```

In-kernel consumers access cells defined in devicetree:
```c
struct nvmem_cell *cell;
cell = devm_nvmem_cell_get(dev, "mac-address");
```

---

## 8. Devicetree Parsing

Use the unified `device_property_*` API (works for both DT and ACPI):

```c
static int <devname>_parse_dt(struct device *dev,
			      struct <devname>_state *st)
{
	int ret;

	/* Required property -- fail if missing. */
	ret = device_property_read_u32(dev, "adi,sample-rate",
				       &st->sample_rate);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Missing adi,sample-rate property\n");

	/* Optional property -- use default if missing. */
	st->page_size = 32;
	device_property_read_u32(dev, "pagesize", &st->page_size);

	/* Boolean property. */
	st->read_only = device_property_read_bool(dev, "read-only");

	/* Optional GPIO. */
	st->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						  GPIOD_OUT_LOW);
	if (IS_ERR(st->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(st->reset_gpio),
				     "Failed to get reset GPIO\n");

	/* Optional regulator. */
	st->vcc = devm_regulator_get_optional(dev, "vcc");
	if (IS_ERR(st->vcc)) {
		if (PTR_ERR(st->vcc) == -ENODEV)
			st->vcc = NULL;
		else
			return PTR_ERR(st->vcc);
	}

	return 0;
}
```

For iterating child nodes (e.g., multi-port devices):
```c
struct fwnode_handle *child;

device_for_each_child_node(dev, child) {
	u32 port;

	ret = fwnode_property_read_u32(child, "reg", &port);
	if (ret) {
		fwnode_handle_put(child);
		return ret;
	}
	/* configure port ... */
}
```

---

## 9. Test & Debug

### 9.1 Custom sysfs

Add a `scratch` register attribute for basic communication validation:

```c
static ssize_t scratch_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct <devname>_state *st = dev_get_drvdata(dev);
	u32 val;

	regmap_read(st->regs, <DEVNAME>_REG_SCRATCH, &val);
	return sysfs_emit(buf, "0x%08x\n", val);
}

static ssize_t scratch_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct <devname>_state *st = dev_get_drvdata(dev);
	u32 val;
	int ret;

	ret = kstrtou32(buf, 0, &val);
	if (ret)
		return ret;

	regmap_write(st->regs, <DEVNAME>_REG_SCRATCH, val);
	return count;
}
static DEVICE_ATTR_RW(scratch);
```

### 9.2 Debugfs

For developer-only debug information, use debugfs instead of sysfs:

```c
#include <linux/debugfs.h>

static int <devname>_regs_show(struct seq_file *s, void *unused)
{
	struct <devname>_state *st = s->private;
	u32 val;
	int i;

	for (i = 0; i <= 0x40; i += 4) {
		regmap_read(st->regs, i, &val);
		seq_printf(s, "0x%04x: 0x%08x\n", i, val);
	}
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(<devname>_regs);

static void <devname>_debugfs_init(struct <devname>_state *st,
				   struct device *dev)
{
	struct dentry *dir;

	dir = debugfs_create_dir(dev_name(dev), NULL);
	debugfs_create_file("registers", 0444, dir, st,
			    &<devname>_regs_fops);
}
```

### 9.3 Testing patterns

```bash
# Verify probe succeeded
dmesg | grep <devname>

# Check sysfs attributes
ls /sys/bus/platform/devices/*<devname>*/

# Read/write scratch register
echo 0xDEADBEEF > /sys/bus/platform/devices/*<devname>*/scratch
cat /sys/bus/platform/devices/*<devname>*/scratch

# EEPROM: verify nvmem device
ls /sys/bus/nvmem/devices/
hexdump -C /sys/bus/nvmem/devices/<devname>0/nvmem | head -5

# Chardev: verify device node
ls -la /dev/<devname>*

# Regmap debugfs (if regmap is used)
cat /sys/kernel/debug/regmap/*<devname>*/registers
```

---

## 10. Key Conventions

1. **License** -- Use `GPL-2.0` (SPDX identifier at top of file).  DT
   bindings use `(GPL-2.0-only OR BSD-2-Clause)`.

2. **Managed resources** -- Use `devm_*` variants everywhere:
   `devm_kzalloc()`, `devm_regmap_init_*()`, `devm_clk_get_enabled()`,
   `devm_platform_ioremap_resource()`, `devm_misc_register()`,
   `devm_nvmem_register()`, `devm_gpiod_get()`.  This eliminates the
   need for explicit `remove()` callbacks in most cases.

3. **Locking** -- Use `struct mutex` for register access that may sleep
   (I2C, SPI).  Use `spinlock_t` only for interrupt context or
   truly non-sleeping paths.  Document what each lock protects.

4. **Error handling** -- Use `dev_err_probe()` in probe functions (it
   handles `-EPROBE_DEFER` logging automatically).  Return negative
   `errno` values.

5. **Regmap** -- Prefer `regmap` over raw register I/O:
   - `devm_regmap_init_mmio()` for memory-mapped
   - `devm_regmap_init_spi()` for SPI
   - `devm_regmap_init_i2c()` for I2C

6. **Sysfs output** -- Always use `sysfs_emit()` or `sysfs_emit_at()`
   (never `sprintf()` or `snprintf()`).

7. **Module helpers** -- Use the one-line module registration macros:
   - `module_platform_driver()` for platform devices
   - `module_spi_driver()` for SPI devices
   - `module_i2c_driver()` for I2C devices

8. **Devicetree matching** -- Always provide `of_match_table` in the
   driver struct.  Use `MODULE_DEVICE_TABLE(of, ...)` for autoloading.
   For I2C/SPI also provide an `id_table` for non-DT systems.

9. **Naming** -- Driver name in the `driver` struct must match the
   compatible string (minus the vendor prefix).  Sysfs attribute names
   use `snake_case`.

10. **Data flow** -- Store private state with `platform_set_drvdata()`,
    `spi_set_drvdata()`, or `i2c_set_clientdata()`.  Retrieve in
    callbacks with `dev_get_drvdata()`.

---

## 11. Commit Message Format

Use a subsystem-specific prefix matching the target directory.

### Platform/AXI device in `drivers/misc/`

```
misc: adi-<devname>: add <DEVNAME> driver

Add support for the Analog Devices <DEVNAME>, a <brief description>.
The driver registers as a platform device and exposes control
through sysfs attributes.

Signed-off-by: Your Name <your.name@analog.com>
```

### EEPROM in `drivers/misc/eeprom/`

```
eeprom: <devname>: add Analog Devices <DEVNAME> support

Add support for the <DEVNAME> I2C EEPROM with <size> of non-volatile
storage.  The driver registers with the nvmem subsystem.

Signed-off-by: Your Name <your.name@analog.com>
```

### IO-Link / SPI misc device

```
misc: <devname>: add <DEVNAME> IO-Link master driver

Add support for the Analog Devices <DEVNAME> IO-Link master
transceiver.  The driver provides a miscdevice chardev interface
for userspace IO-Link communication.

Signed-off-by: Your Name <your.name@analog.com>
```

### GMSL serializer/deserializer in `drivers/media/i2c/`

```
media: i2c: <devname>: add <DEVNAME> GMSL serializer driver

Add support for the Analog Devices <DEVNAME> GMSL serializer.
The driver registers as a V4L2 I2C subdevice.

Signed-off-by: Your Name <your.name@analog.com>
```

### Display in `drivers/auxdisplay/`

```
auxdisplay: <devname>: add <DEVNAME> LCD driver

Add support for the <DEVNAME> character LCD module using the
charlcd framework.

Signed-off-by: Your Name <your.name@analog.com>
```

### Devicetree binding (always a separate commit)

```
dt-bindings: misc: add Analog Devices <DEVNAME>

Add devicetree binding documentation for the Analog Devices
<DEVNAME>.

Signed-off-by: Your Name <your.name@analog.com>
```
