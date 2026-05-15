# Linux IIO Health Driver Template: ECG / Biopotential Devices

This template covers IIO health-class drivers for ECG and biopotential
measurement devices such as the ADAS1000 and AD8232. Health drivers live under
`drivers/iio/health/` and are characterized by multi-channel voltage input
for ECG leads, high-speed continuous buffered streaming, and hardware
data-ready (DRDY) triggered capture.

---

## 1. Purpose

Health / ECG devices acquire biopotential signals from multiple electrode
leads. In the IIO subsystem they map to:

| Aspect                 | Detail                                                       |
|------------------------|--------------------------------------------------------------|
| IIO subdirectory       | `iio/health/`                                                |
| Primary channel type   | `IIO_VOLTAGE` (input) -- one channel per ECG lead            |
| Alternative types      | `IIO_ELECTRICALCONDUCTIVITY` (impedance / lead-off detect), `IIO_INTENSITY` (PPG) |
| Typical info_mask      | `RAW`, `SCALE`, `SAMP_FREQ`                                 |
| Data format            | 24-bit signed samples, packed into 32-bit storage            |
| Buffer model           | Continuous streaming via IIO triggered buffer or kfifo       |
| Trigger source         | Hardware DRDY interrupt from the device                      |

ECG front-ends typically sample all leads simultaneously at rates from
128 SPS to 32 kSPS. The driver must support continuous streaming with
minimal latency -- single-shot reads are secondary.

---

## 2. File Checklist

| File                                                                  | Action   | Required |
|-----------------------------------------------------------------------|----------|----------|
| `drivers/iio/health/<devname>.c`                                      | Create   | Yes      |
| `drivers/iio/health/Kconfig`                                          | Modify   | Yes      |
| `drivers/iio/health/Makefile`                                         | Modify   | Yes      |
| `Documentation/devicetree/bindings/iio/health/adi,<devname>.yaml`     | Create   | Yes      |
| `include/dt-bindings/iio/health/adi,<devname>.h`                      | Create   | Optional |

A dt-bindings header is only needed when the binding defines enum constants
(e.g. electrode configuration modes) shared between the YAML and the driver.

---

## 3. Devicetree Binding

Health/ECG devices are typically SPI, use a DRDY interrupt for continuous
data streaming, and may expose electrode configuration properties.

```yaml
# SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)
%YAML 1.2
---
$id: http://devicetree.org/schemas/iio/health/adi,adas1000.yaml#
$schema: http://devicetree.org/meta-schemas/core.yaml#

title: Analog Devices ADAS1000 ECG Analog Front End

maintainers:
  - First Last <first.last@analog.com>

description: |
  The ADAS1000 is a low power, five-electrode ECG analog front end
  with respiration measurement and lead-off detection. It provides
  simultaneous sampling of all lead inputs at up to 16 kSPS with
  24-bit resolution.

properties:
  compatible:
    enum:
      - adi,adas1000
      - adi,adas1000-1
      - adi,adas1000-2

  reg:
    maxItems: 1

  spi-max-frequency:
    maximum: 20000000

  spi-cpha: true

  interrupts:
    maxItems: 1
    description:
      DRDY interrupt, active low. Required for continuous ECG
      streaming via the IIO buffer.

  interrupt-names:
    const: drdy

  clocks:
    maxItems: 1

  clock-names:
    const: mclk

  avdd-supply: true
  dvdd-supply: true
  refin-supply: true

  adi,lead-off-detect:
    type: boolean
    description:
      Enable lead-off detection circuitry. When set, the driver
      exposes additional channels for electrode impedance.

  adi,right-leg-drive:
    type: boolean
    description:
      Enable right-leg drive (RLD) output for common-mode rejection.

  adi,wilson-center:
    type: boolean
    description:
      Enable Wilson Central Terminal for unipolar lead derivation.

  adi,electrode-count:
    $ref: /schemas/types.yaml#/definitions/uint32
    enum: [3, 5]
    default: 5
    description:
      Number of electrodes connected. Determines which lead
      channels are active (3-lead or 5-lead configuration).

  adi,sample-rate-hz:
    $ref: /schemas/types.yaml#/definitions/uint32
    enum: [2000, 16000, 128000]
    default: 2000
    description:
      Initial ECG data output rate in Hz.

required:
  - compatible
  - reg
  - interrupts

additionalProperties: false

examples:
  - |
    #include <linux/interrupt-controller/irq.h>

    spi {
        #address-cells = <1>;
        #size-cells = <0>;

        ecg@0 {
            compatible = "adi,adas1000";
            reg = <0>;
            spi-max-frequency = <20000000>;
            spi-cpha;
            interrupts = <25 IRQ_TYPE_EDGE_FALLING>;
            interrupt-parent = <&gpio>;
            interrupt-names = "drdy";
            avdd-supply = <&avdd_5v>;
            dvdd-supply = <&dvdd_1v8>;
            adi,electrode-count = <5>;
            adi,right-leg-drive;
            adi,wilson-center;
        };
    };
```

### Key Health/ECG Binding Patterns

- **DRDY interrupt** is essential -- continuous ECG streaming depends on it.
- **Electrode configuration** properties (`adi,electrode-count`,
  `adi,right-leg-drive`, `adi,wilson-center`) control the analog front-end
  topology. These are device-specific and do not have per-channel sub-nodes
  because all leads are sampled simultaneously.
- Unlike ADC drivers, health devices rarely use `channel@N` sub-nodes. The
  lead mapping (Lead I, II, III, aVR, aVL, aVF, V1-V6) is fixed by the
  hardware topology.

---

## 4. Kconfig

Add the entry to `drivers/iio/health/Kconfig` in alphabetical order, within
the `menu "Heart Rate Monitors"` or a new `menu "ECG Front Ends"` section.

```kconfig
config ADAS1000
	tristate "Analog Devices ADAS1000 ECG analog front end driver"
	depends on SPI_MASTER
	select REGMAP_SPI
	select IIO_BUFFER
	select IIO_TRIGGERED_BUFFER
	help
	  Say yes here to build support for Analog Devices ADAS1000,
	  ADAS1000-1, and ADAS1000-2 ECG analog front ends with 24-bit
	  resolution and simultaneous lead sampling.

	  To compile this driver as a module, choose M here: the module
	  will be called adas1000.
```

### Notes

- Always select `IIO_BUFFER` and `IIO_TRIGGERED_BUFFER` -- health devices
  are fundamentally streaming devices.
- Use `select IIO_KFIFO_BUF` instead of `IIO_TRIGGERED_BUFFER` when the
  device has an internal FIFO and the driver pushes data from an ISR without
  using the IIO trigger framework (see the MAX30102 pattern).
- Add `select REGMAP_SPI` or `select REGMAP_I2C` when using regmap.

---

## 5. Makefile

Add to `drivers/iio/health/Makefile` in alphabetical order:

```makefile
obj-$(CONFIG_ADAS1000)		+= adas1000.o
```

For multi-file drivers:

```makefile
obj-$(CONFIG_ADAS1000) += adas1000.o
adas1000-y := adas1000-core.o adas1000-trigger.o
```

---

## 6. Driver Source

### Complete ECG Driver Skeleton (SPI)

```c
// SPDX-License-Identifier: GPL-2.0+
/*
 * ADAS1000 ECG Analog Front End driver
 *
 * Copyright 2024 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

/* ------------------------------------------------------------------ */
/* Register Definitions                                                */
/* ------------------------------------------------------------------ */

#define ADAS1000_REG_NOP		0x00
#define ADAS1000_REG_ECGCTL		0x01
#define ADAS1000_REG_LOFFCTL		0x02
#define ADAS1000_REG_RESPCTL		0x03
#define ADAS1000_REG_FRMCTL		0x0A
#define ADAS1000_REG_LA_DATA		0x11
#define ADAS1000_REG_LL_DATA		0x12
#define ADAS1000_REG_RA_DATA		0x13
#define ADAS1000_REG_V1_DATA		0x14
#define ADAS1000_REG_V2_DATA		0x15
#define ADAS1000_REG_STATUS		0x40

#define ADAS1000_ECGCTL_RATE		GENMASK(31, 30)
#define ADAS1000_ECGCTL_GAIN		GENMASK(29, 28)

/* Frame rate: 0 = 2 kHz, 1 = 16 kHz, 2 = 128 kHz */
#define ADAS1000_RATE_2K		0
#define ADAS1000_RATE_16K		1
#define ADAS1000_RATE_128K		2

/* ------------------------------------------------------------------ */
/* Device State                                                        */
/* ------------------------------------------------------------------ */

struct adas1000_chip_info {
	const char		*name;
	unsigned int		num_leads;
	unsigned int		resolution;
};

struct adas1000_state {
	struct spi_device	*spi;
	struct regmap		*regmap;
	const struct adas1000_chip_info *chip_info;
	struct iio_trigger	*trig;
	struct mutex		lock;	/* Protect device state */
	unsigned int		samp_freq;
	int			irq;
	/*
	 * Buffer for all ECG lead samples + timestamp.
	 * 5 leads x 32-bit + 64-bit timestamp.
	 * Must be at the end and cache-line aligned for DMA safety.
	 */
	struct {
		s32 leads[5];
		s64 timestamp __aligned(8);
	} scan __aligned(IIO_DMA_MINALIGN);
};

/* ------------------------------------------------------------------ */
/* IIO Channel Specification                                           */
/* ------------------------------------------------------------------ */

/*
 * ECG channels: one IIO_VOLTAGE input per lead.
 * 24-bit signed data in 32-bit storage. All leads share the same
 * sample rate (set via SAMP_FREQ on any channel or shared_by_all).
 */
#define ADAS1000_ECG_CHANNEL(_idx, _addr, _name) {		\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.channel = (_idx),					\
	.address = (_addr),					\
	.extend_name = (_name),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
		BIT(IIO_CHAN_INFO_SCALE),			\
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),\
	.scan_index = (_idx),					\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 24,					\
		.storagebits = 32,				\
		.shift = 0,					\
		.endianness = IIO_BE,				\
	},							\
}

static const struct iio_chan_spec adas1000_channels[] = {
	ADAS1000_ECG_CHANNEL(0, ADAS1000_REG_LA_DATA, "lead_la"),
	ADAS1000_ECG_CHANNEL(1, ADAS1000_REG_LL_DATA, "lead_ll"),
	ADAS1000_ECG_CHANNEL(2, ADAS1000_REG_RA_DATA, "lead_ra"),
	ADAS1000_ECG_CHANNEL(3, ADAS1000_REG_V1_DATA, "lead_v1"),
	ADAS1000_ECG_CHANNEL(4, ADAS1000_REG_V2_DATA, "lead_v2"),
	IIO_CHAN_SOFT_TIMESTAMP(5),
};

/* ------------------------------------------------------------------ */
/* Sample Rate Table                                                   */
/* ------------------------------------------------------------------ */

static const int adas1000_samp_freq_table[] = { 2000, 16000, 128000 };

static int adas1000_set_samp_freq(struct adas1000_state *st,
				  unsigned int freq)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(adas1000_samp_freq_table); i++) {
		if (adas1000_samp_freq_table[i] == freq) {
			st->samp_freq = freq;
			return regmap_update_bits(st->regmap,
						  ADAS1000_REG_ECGCTL,
						  ADAS1000_ECGCTL_RATE,
						  FIELD_PREP(ADAS1000_ECGCTL_RATE, i));
		}
	}

	return -EINVAL;
}

/* ------------------------------------------------------------------ */
/* read_raw / write_raw Callbacks                                      */
/* ------------------------------------------------------------------ */

static int adas1000_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int *val, int *val2, long info)
{
	struct adas1000_state *st = iio_priv(indio_dev);
	unsigned int regval;
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		if (!iio_device_claim_direct(indio_dev))
			return -EBUSY;

		mutex_lock(&st->lock);
		ret = regmap_read(st->regmap, chan->address, &regval);
		mutex_unlock(&st->lock);

		iio_device_release_direct(indio_dev);

		if (ret)
			return ret;

		/* Sign-extend 24-bit to 32-bit */
		*val = sign_extend32(regval, 23);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		/*
		 * Scale: Vref / (2^23 * Gain)
		 * For 1.8V reference and gain of 1.4:
		 *   1800 mV / (8388608 * 1.4) = ~0.000153 mV/LSB
		 * Return as IIO_VAL_INT_PLUS_NANO for precision.
		 */
		*val = 0;
		*val2 = 153;  /* nV per LSB, device-specific */
		return IIO_VAL_INT_PLUS_NANO;

	case IIO_CHAN_INFO_SAMP_FREQ:
		mutex_lock(&st->lock);
		*val = st->samp_freq;
		mutex_unlock(&st->lock);
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

static int adas1000_write_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int val, int val2, long info)
{
	struct adas1000_state *st = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (val <= 0)
			return -EINVAL;

		if (!iio_device_claim_direct(indio_dev))
			return -EBUSY;

		guard(mutex)(&st->lock);
		iio_device_release_direct(indio_dev);

		return adas1000_set_samp_freq(st, val);

	default:
		return -EINVAL;
	}
}

static int adas1000_read_avail(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       const int **vals, int *type, int *length,
			       long info)
{
	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*vals = adas1000_samp_freq_table;
		*type = IIO_VAL_INT;
		*length = ARRAY_SIZE(adas1000_samp_freq_table);
		return IIO_AVAIL_LIST;
	default:
		return -EINVAL;
	}
}

/* ------------------------------------------------------------------ */
/* debugfs Register Access                                             */
/* ------------------------------------------------------------------ */

static int adas1000_reg_access(struct iio_dev *indio_dev,
			       unsigned int reg, unsigned int writeval,
			       unsigned int *readval)
{
	struct adas1000_state *st = iio_priv(indio_dev);

	if (readval)
		return regmap_read(st->regmap, reg, readval);

	return regmap_write(st->regmap, reg, writeval);
}

/* ------------------------------------------------------------------ */
/* iio_info                                                            */
/* ------------------------------------------------------------------ */

static const struct iio_info adas1000_info = {
	.read_raw = adas1000_read_raw,
	.write_raw = adas1000_write_raw,
	.read_avail = adas1000_read_avail,
	.debugfs_reg_access = adas1000_reg_access,
};

/* ------------------------------------------------------------------ */
/* Trigger & Buffer (see Section 7)                                    */
/* ------------------------------------------------------------------ */

static irqreturn_t adas1000_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct adas1000_state *st = iio_priv(indio_dev);
	int ret, i, j = 0;

	memset(&st->scan, 0, sizeof(st->scan));

	iio_for_each_active_channel(indio_dev, i) {
		unsigned int val;

		ret = regmap_read(st->regmap,
				  adas1000_channels[i].address, &val);
		if (ret)
			goto done;

		st->scan.leads[j++] = sign_extend32(val, 23);
	}

	iio_push_to_buffers_with_timestamp(indio_dev, &st->scan,
					   iio_get_time_ns(indio_dev));

done:
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

/* ------------------------------------------------------------------ */
/* Probe                                                               */
/* ------------------------------------------------------------------ */

static const struct regmap_config adas1000_regmap_config = {
	.reg_bits = 8,
	.val_bits = 24,
	.max_register = 0x40,
};

static int adas1000_probe(struct spi_device *spi)
{
	const struct adas1000_chip_info *info;
	struct device *dev = &spi->dev;
	struct adas1000_state *st;
	struct iio_dev *indio_dev;
	int ret;

	info = spi_get_device_match_data(spi);
	if (!info)
		return dev_err_probe(dev, -ENODEV,
				     "Failed to get match data\n");

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;
	st->chip_info = info;
	st->irq = spi->irq;

	ret = devm_mutex_init(dev, &st->lock);
	if (ret)
		return ret;

	st->regmap = devm_regmap_init_spi(spi, &adas1000_regmap_config);
	if (IS_ERR(st->regmap))
		return dev_err_probe(dev, PTR_ERR(st->regmap),
				     "Failed to init regmap\n");

	indio_dev->name = info->name;
	indio_dev->info = &adas1000_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = adas1000_channels;
	indio_dev->num_channels = ARRAY_SIZE(adas1000_channels);

	/* Power supplies */
	ret = devm_regulator_get_enable(dev, "avdd");
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to enable avdd supply\n");

	ret = devm_regulator_get_enable(dev, "dvdd");
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to enable dvdd supply\n");

	/* Hardware init: reset, set default sample rate */
	st->samp_freq = 2000;

	/* Parse electrode configuration from devicetree */
	/* ... see Section 8 ... */

	/* Hardware DRDY trigger */
	if (st->irq > 0) {
		st->trig = devm_iio_trigger_alloc(dev, "%s-dev%d",
						   indio_dev->name,
						   iio_device_id(indio_dev));
		if (!st->trig)
			return -ENOMEM;

		iio_trigger_set_drvdata(st->trig, indio_dev);

		ret = devm_iio_trigger_register(dev, st->trig);
		if (ret)
			return dev_err_probe(dev, ret,
					     "Failed to register trigger\n");

		ret = devm_request_threaded_irq(dev, st->irq,
						iio_trigger_generic_data_rdy_poll,
						NULL, IRQF_ONESHOT,
						info->name, st->trig);
		if (ret)
			return dev_err_probe(dev, ret,
					     "Failed to request DRDY IRQ\n");
	}

	/* Triggered buffer for continuous ECG streaming */
	ret = devm_iio_triggered_buffer_setup(dev, indio_dev,
					      &iio_pollfunc_store_time,
					      adas1000_trigger_handler,
					      NULL);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to setup triggered buffer\n");

	return devm_iio_device_register(dev, indio_dev);
}

/* ------------------------------------------------------------------ */
/* Match Tables                                                        */
/* ------------------------------------------------------------------ */

static const struct adas1000_chip_info adas1000_chip_info = {
	.name = "adas1000",
	.num_leads = 5,
	.resolution = 24,
};

static const struct adas1000_chip_info adas1000_1_chip_info = {
	.name = "adas1000-1",
	.num_leads = 3,
	.resolution = 24,
};

static const struct of_device_id adas1000_of_match[] = {
	{ .compatible = "adi,adas1000", .data = &adas1000_chip_info },
	{ .compatible = "adi,adas1000-1", .data = &adas1000_1_chip_info },
	{ }
};
MODULE_DEVICE_TABLE(of, adas1000_of_match);

static const struct spi_device_id adas1000_ids[] = {
	{ "adas1000", (kernel_ulong_t)&adas1000_chip_info },
	{ "adas1000-1", (kernel_ulong_t)&adas1000_1_chip_info },
	{ }
};
MODULE_DEVICE_TABLE(spi, adas1000_ids);

static struct spi_driver adas1000_driver = {
	.driver = {
		.name = "adas1000",
		.of_match_table = adas1000_of_match,
	},
	.probe = adas1000_probe,
	.id_table = adas1000_ids,
};
module_spi_driver(adas1000_driver);

MODULE_AUTHOR("First Last <first.last@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADAS1000 ECG analog front end driver");
MODULE_LICENSE("GPL");
```

### ECG-Specific Driver Notes

- **Channel type**: Use `IIO_VOLTAGE` for ECG lead signals. Each lead
  (LA, LL, RA, V1, V2, etc.) is a separate indexed voltage input channel.
- **extend_name**: Use `extend_name` (e.g. `"lead_la"`, `"lead_v1"`) to
  identify which electrode each channel corresponds to in sysfs.
- **24-bit signed data**: ECG signals are bipolar. Set `.sign = 's'`,
  `.realbits = 24`, `.storagebits = 32`. Use `sign_extend32(val, 23)` when
  reading raw values.
- **SAMP_FREQ shared_by_all**: All ECG leads are sampled simultaneously at
  the same rate. Use `info_mask_shared_by_all` for `SAMP_FREQ` so that a
  single `sampling_frequency` sysfs file controls the global rate.
- **No per-channel DT nodes**: Unlike generic ADC drivers, ECG front-ends
  sample a fixed set of leads. Channel configuration is determined by the
  electrode count, not by per-channel devicetree nodes.

---

## 7. Buffer & Trigger

### Why Buffered Streaming Is Critical for Health

ECG applications require continuous, uninterrupted data capture at fixed
sample rates. Single-shot `read_raw` access is only for debug. Production
use cases rely entirely on the IIO buffer path.

### Hardware DRDY Trigger

The device asserts a DRDY interrupt each time a new frame of lead data is
available. This drives the IIO trigger:

```c
/* Allocate and register a hardware trigger */
st->trig = devm_iio_trigger_alloc(dev, "%s-dev%d",
				   indio_dev->name,
				   iio_device_id(indio_dev));
if (!st->trig)
	return -ENOMEM;

iio_trigger_set_drvdata(st->trig, indio_dev);

ret = devm_iio_trigger_register(dev, st->trig);
if (ret)
	return dev_err_probe(dev, ret, "Failed to register trigger\n");

/* Connect the DRDY IRQ to the trigger */
ret = devm_request_threaded_irq(dev, st->irq,
				iio_trigger_generic_data_rdy_poll,
				NULL, IRQF_ONESHOT,
				info->name, st->trig);
if (ret)
	return dev_err_probe(dev, ret, "Failed to request DRDY IRQ\n");
```

### Triggered Buffer Setup

```c
ret = devm_iio_triggered_buffer_setup(dev, indio_dev,
				      &iio_pollfunc_store_time,
				      adas1000_trigger_handler,
				      NULL);
```

### Trigger Handler -- Multi-Lead Read

```c
static irqreturn_t adas1000_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct adas1000_state *st = iio_priv(indio_dev);
	int ret, i, j = 0;

	memset(&st->scan, 0, sizeof(st->scan));

	iio_for_each_active_channel(indio_dev, i) {
		unsigned int val;

		ret = regmap_read(st->regmap,
				  adas1000_channels[i].address, &val);
		if (ret)
			goto done;

		st->scan.leads[j++] = sign_extend32(val, 23);
	}

	iio_push_to_buffers_with_timestamp(indio_dev, &st->scan,
					   iio_get_time_ns(indio_dev));

done:
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}
```

### scan_type for 24-Bit ECG Data

```c
.scan_type = {
	.sign = 's',          /* Signed -- ECG is bipolar */
	.realbits = 24,       /* 24-bit ADC resolution */
	.storagebits = 32,    /* Stored in 32-bit words */
	.shift = 0,           /* No shift needed */
	.endianness = IIO_BE, /* SPI devices are typically big-endian */
},
```

### Alternative: kfifo Buffer for Devices with Internal FIFO

Some health devices (e.g. MAX30102) have an internal sample FIFO. In this
case, use `devm_iio_kfifo_buffer_setup()` instead of
`devm_iio_triggered_buffer_setup()` and read the FIFO directly from the
ISR:

```c
/* In probe: */
ret = devm_iio_kfifo_buffer_setup(dev, indio_dev,
				  &adas1000_buffer_setup_ops);

/* ISR reads FIFO and pushes samples: */
static irqreturn_t adas1000_interrupt_handler(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct adas1000_state *st = iio_priv(indio_dev);

	/* Read all available samples from device FIFO */
	while (samples_available) {
		/* ... read sample ... */
		iio_push_to_buffers(indio_dev, st->buffer);
	}

	return IRQ_HANDLED;
}
```

With kfifo, select `IIO_KFIFO_BUF` in Kconfig instead of
`IIO_TRIGGERED_BUFFER`.

### Kconfig Selects for Buffer Support

```kconfig
	select IIO_BUFFER
	select IIO_TRIGGERED_BUFFER
```

Or for kfifo-based buffering:

```kconfig
	select IIO_BUFFER
	select IIO_KFIFO_BUF
```

---

## 8. Devicetree Parsing

### Lead / Electrode Configuration

ECG front-ends configure their analog input topology through DT properties
rather than per-channel sub-nodes:

```c
static int adas1000_parse_dt(struct iio_dev *indio_dev,
			     struct device *dev)
{
	struct adas1000_state *st = iio_priv(indio_dev);
	u32 electrode_count;
	int ret;

	/* Number of electrodes: 3-lead or 5-lead */
	ret = device_property_read_u32(dev, "adi,electrode-count",
				       &electrode_count);
	if (ret)
		electrode_count = st->chip_info->num_leads;

	if (electrode_count != 3 && electrode_count != 5)
		return dev_err_probe(dev, -EINVAL,
				     "Invalid electrode count %u\n",
				     electrode_count);

	/* Adjust channel count based on electrode configuration */
	if (electrode_count == 3) {
		/* 3-lead: only LA, LL, RA active */
		indio_dev->num_channels = 3 + 1; /* 3 leads + timestamp */
	}

	/* Right-leg drive for common-mode rejection */
	if (device_property_read_bool(dev, "adi,right-leg-drive")) {
		/* Enable RLD in hardware */
	}

	/* Wilson Central Terminal */
	if (device_property_read_bool(dev, "adi,wilson-center")) {
		/* Enable WCT in hardware */
	}

	/* Lead-off detection */
	if (device_property_read_bool(dev, "adi,lead-off-detect")) {
		/* Enable lead-off detection, add impedance channels */
	}

	/* Initial sample rate */
	ret = device_property_read_u32(dev, "adi,sample-rate-hz",
				       &st->samp_freq);
	if (ret)
		st->samp_freq = 2000;

	return adas1000_set_samp_freq(st, st->samp_freq);
}
```

### Common Health DT Properties

| Property                | API                                  | Purpose                                |
|-------------------------|--------------------------------------|----------------------------------------|
| `adi,electrode-count`   | `device_property_read_u32()`         | 3-lead vs 5-lead ECG topology          |
| `adi,right-leg-drive`   | `device_property_read_bool()`        | Enable RLD for noise rejection         |
| `adi,wilson-center`     | `device_property_read_bool()`        | Enable Wilson Central Terminal          |
| `adi,lead-off-detect`   | `device_property_read_bool()`        | Enable electrode lead-off detection    |
| `adi,sample-rate-hz`    | `device_property_read_u32()`         | Initial data output rate               |

---

## 9. Test & Debug

### Streaming ECG Data with iio_readdev

The primary test for an ECG driver is continuous data streaming. Use
`iio_readdev` to capture buffered ECG samples:

```sh
# List available devices and channels
iio_info

# Enable all ECG lead channels and stream data
# -b: buffer size, -s: number of samples to capture
iio_readdev -b 256 -s 4096 iio:device0 > ecg_data.bin

# Stream indefinitely (Ctrl+C to stop)
iio_readdev -b 256 iio:device0 > ecg_data.bin

# Stream only specific leads
iio_readdev -b 256 -s 1000 iio:device0 voltage0 voltage1 voltage2
```

### Check and Set Sample Rate

```sh
# Read current sample rate
cat /sys/bus/iio/devices/iio:device0/sampling_frequency

# See available sample rates
cat /sys/bus/iio/devices/iio:device0/sampling_frequency_available

# Set sample rate to 16 kHz
echo 16000 > /sys/bus/iio/devices/iio:device0/sampling_frequency
```

### Single-Shot Channel Reads (Debug Only)

```sh
# Read a single lead value (not for production ECG capture)
cat /sys/bus/iio/devices/iio:device0/in_voltage0_lead_la_raw
cat /sys/bus/iio/devices/iio:device0/in_voltage0_lead_la_scale
```

### Trigger Verification

```sh
# List available triggers
cat /sys/bus/iio/devices/trigger0/name

# Verify the DRDY trigger is assigned
cat /sys/bus/iio/devices/iio:device0/trigger/current_trigger
```

### debugfs Register Access

```sh
# Read register 0x01 (ECGCTL)
echo 0x01 > /sys/kernel/debug/iio/iio:device0/direct_reg_access
cat /sys/kernel/debug/iio/iio:device0/direct_reg_access

# Write 0x100000 to register 0x01
echo 0x01 0x100000 > /sys/kernel/debug/iio/iio:device0/direct_reg_access
```

### Verifying Data Integrity

```sh
# Check for buffer overruns
cat /sys/bus/iio/devices/iio:device0/buffer/watermark
cat /sys/bus/iio/devices/iio:device0/buffer/data_available

# Inspect captured binary data (5 x s32 leads per sample)
hexdump -C ecg_data.bin | head -20

# Python snippet to parse and plot ECG data
python3 -c "
import numpy as np
data = np.fromfile('ecg_data.bin', dtype=np.int32)
leads = data.reshape(-1, 6)  # 5 leads + timestamp(2xu32)
print(f'Samples: {leads.shape[0]}')
print(f'Lead I range: [{leads[:,0].min()}, {leads[:,0].max()}]')
"
```

---

## 10. Key Conventions

### License

All new ADI IIO drivers must use GPL-2.0 or GPL-2.0+:

```c
// SPDX-License-Identifier: GPL-2.0+
```

`MODULE_LICENSE("GPL")` at the bottom must match.

### Continuous Streaming Focus

Health drivers are fundamentally streaming devices. Design priorities:

1. **Buffer path is primary** -- `read_raw` is for debug only; all
   production use cases go through the IIO buffer.
2. **Low latency** -- minimize time between DRDY assertion and data push.
   Read all lead registers in a tight loop within the trigger handler.
3. **No data loss** -- size the IIO buffer appropriately and use high
   trigger priority. For very high sample rates (128 kSPS), consider
   DMA-based SPI transfers.
4. **All channels sample simultaneously** -- ECG leads must be captured
   from the same conversion frame. Never read leads across multiple
   conversion cycles.

### Memory Management

Use `devm_*` exclusively. Key functions:

| Function                            | Purpose                              |
|-------------------------------------|--------------------------------------|
| `devm_iio_device_alloc()`           | Allocate IIO device + private data   |
| `devm_iio_device_register()`        | Register IIO device (auto-unregister)|
| `devm_iio_triggered_buffer_setup()` | Set up triggered buffer              |
| `devm_iio_trigger_alloc()`          | Allocate IIO trigger                 |
| `devm_iio_trigger_register()`       | Register IIO trigger                 |
| `devm_request_threaded_irq()`       | Request DRDY interrupt               |
| `devm_regmap_init_spi()`            | Initialize SPI regmap                |
| `devm_regulator_get_enable()`       | Get and enable a power supply        |
| `devm_mutex_init()`                 | Initialize mutex with devm cleanup   |

### DMA-Safe Buffers

The scan buffer used in the trigger handler must be DMA-safe:

```c
struct {
	s32 leads[5];
	s64 timestamp __aligned(8);
} scan __aligned(IIO_DMA_MINALIGN);
```

Place it at the end of the state structure and align with
`IIO_DMA_MINALIGN`.

### Coding Style

- Follow kernel coding style (`Documentation/process/coding-style.rst`).
- Run `scripts/checkpatch.pl` before submitting.
- Use `FIELD_GET()` / `FIELD_PREP()` with `GENMASK()` for register fields.
- Use `dev_err_probe()` for all error returns in `probe()`.
- Include headers in alphabetical order within each group.
- Use `guard(mutex)(&st->lock)` for scoped locking where possible.

---

## 11. Commit Message Format

### Subject Line Prefix

```
iio: health: <devname>: <brief description>
```

### Examples

```
iio: health: adas1000: add support for ADAS1000 ECG front end
iio: health: adas1000: fix sample rate configuration
iio: health: ad8232: add triggered buffer support
iio: health: adas1000: use devm_iio_triggered_buffer_setup
```

### Patch Series for a New ECG Driver

1. `dt-bindings: iio: health: add adi,adas1000.yaml` -- DT binding
2. `iio: health: adas1000: add support for ADAS1000 ECG front end` -- Driver
3. `MAINTAINERS: add entry for ADAS1000 IIO driver` -- Maintainer entry

### Full Commit Example

```
iio: health: adas1000: add support for ADAS1000 ECG front end

The ADAS1000 is a low power, 5-electrode ECG analog front end with
respiration measurement and lead-off detection. It provides simultaneous
sampling of all lead inputs at up to 128 kSPS with 24-bit resolution.

This driver supports:
  - 3-lead and 5-lead electrode configurations
  - Configurable sample rate (2 kHz, 16 kHz, 128 kHz)
  - Continuous ECG data streaming via IIO triggered buffer
  - Hardware DRDY interrupt trigger
  - Right-leg drive and Wilson Central Terminal configuration
  - Lead-off detection via devicetree

Signed-off-by: First Last <first.last@analog.com>
```
