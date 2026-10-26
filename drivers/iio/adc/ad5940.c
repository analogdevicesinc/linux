// SPDX-License-Identifier: GPL-2.0
/*
 * AD5940/AD5941 SPI ADC driver
 *
 * Copyright (C) 2019 Song Qiang <songqiang1304521@gmail.com>
 */

#include <linux/array_size.h>
#include <linux/bitfield.h>
#include <linux/bitmap.h>
#include <linux/bitops.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/unaligned.h>

#include <linux/iio/iio.h>

#define AD5940_CHANNEL_AINP_MSK		GENMASK(5, 0)
#define AD5940_CHANNEL_AINP(x)		FIELD_PREP(AD5940_CHANNEL_AINP_MSK, x)
#define AD5940_CHANNEL_AINN_MSK		GENMASK(12, 8)
#define AD5940_CHANNEL_AINN(x)		FIELD_PREP(AD5940_CHANNEL_AINN_MSK, x)

#define AD5940_SPICMD_SETADDR		0x20
#define AD5940_SPICMD_READREG		0x6d
#define AD5940_SPICMD_WRITEREG		0x2d

#define AD5940_REG_AFECON		0x00002000
#define AD5940_AFECON_ADCCONV_MSK	BIT(8)
#define AD5940_AFECON_ADCCONV_EN	FIELD_PREP(AD5940_AFECON_ADCCONV_MSK, 1)
#define AD5940_AFECON_ADCCONV_DIS	FIELD_PREP(AD5940_AFECON_ADCCONV_MSK, 0)
#define AD5940_AFECON_ADCEN_MSK		BIT(7)
#define AD5940_AFECON_ADCEN		FIELD_PREP(AD5940_AFECON_ADCEN_MSK, 1)

#define AD5940_REG_ADCDAT		0x00002074

#define AD5940_REG_INTCPOL		0x00003000 /* Interrupt Polarity. */
#define AD5940_INTCPOL_MSK		BIT(0)
#define AD5940_INTCPOL_POS		FIELD_PREP(AD5940_INTCPOL_MSK, 1)
#define AD5940_INTCPOL_NEG		FIELD_PREP(AD5940_INTCPOL_MSK, 0)

#define AD5940_REG_PMBW			0x000022F0
#define	AD5940_PMBW_SYSHS_MSK		BIT(0)
#define	AD5940_PMBW_LP			FIELD_PREP(AD5940_PMBW_SYSHS_MSK, 0)
#define AD5940_PMBW_SYSBW_MSK		GENMASK(3, 2)
#define	AD5940_PMBW_BW250		FIELD_PREP(AD5940_PMBW_SYSBW_MSK, 3)

#define AD5940_REG_ADCCON		0x000021A8
#define AD5940_ADCCON_PGA_MSK		GENMASK(18, 16)
#define AD5940_ADCCON_PGA_1		FIELD_PREP(AD5940_ADCCON_PGA_MSK, 0)
#define AD5940_ADCCON_MUX_MSK		(GENMASK(12, 8) | GENMASK(5, 0))

#define AD5940_REG_INTCLR		0x00003004
#define AD5940_REG_INTCSEL0		0x00003008
#define AD5940_INTC_ADC			BIT(0)

#define AD5940_REG_GP0CON		0x00000000
#define AD5940_GP0CON_3_MSK		GENMASK(7, 6)
#define AD5940_GP0CON_3_INT		FIELD_PREP(AD5940_GP0CON_3_MSK, 3)
#define AD5940_GP0CON_6_MSK		GENMASK(13, 12)
#define AD5940_GP0CON_6_INT		FIELD_PREP(AD5940_GP0CON_6_MSK, 3)

#define AD5940_REG_GP0OEN		0x00000004

#define AD5940_AFECON_CHIPID		0x00000404
#define AD5940_CHIPID_PARTID_MSK	GENMASK(15, 4)
#define AD5940_PARTID			0x550
#define AD5940_VREF_MV			1820
#define AD5940_ADC_MIDSCALE		32768
#define AD5940_AIN6_MUXP		22
#define AD5940_AIN6_MUXN		14

/* Registers in this range are 32 bits wide, all others are 16 bits wide. */
#define AD5940_REG32_MIN		0x1000
#define AD5940_REG32_MAX		0x3014
#define AD5940_MAX_REGISTER		0xffff

#define AD5940_RESET_PULSE_US		10
#define AD5940_RESET_WAIT_MS		30

struct ad5940_channel_config {
	u32 ain;
	const char *channel_name;
};

struct ad5940_chip_info {
	const char *name;
	unsigned int num_gpios;
	bool has_ain6;
};

struct ad5940_state {
	struct spi_device *spi;
	struct regmap *regmap;
	struct gpio_desc *reset_gpio;
	/* This mutex is for protecting the ADC conversion sequence. */
	struct mutex lock;

	struct completion complete;

	const struct ad5940_chip_info *info;
	int num_channels;
	struct ad5940_channel_config *channel_config;
};

static const struct ad5940_chip_info ad5940_chip_info = {
	.name = "ad5940",
	.num_gpios = 8,
	.has_ain6 = true,
};

static const struct ad5940_chip_info ad5941_chip_info = {
	.name = "ad5941",
	.num_gpios = 3,
	.has_ain6 = false,
};

/*
 * The AD5940 SPI protocol needs a first transfer to set the register address
 * and a second one, with chip select toggled in between, to access it.
 */
struct ad5940_bus {
	struct spi_device *spi;
	/* DMA safe buffers, accesses are serialized by the regmap lock. */
	u8 setaddr[3] __aligned(IIO_DMA_MINALIGN);
	u8 wr[5];
	u8 rd_cmd[2];
	/* Rx buffer in its own cache line, away from the CPU written tx data. */
	u8 rd[4] __aligned(IIO_DMA_MINALIGN);
};

static bool ad5940_reg_is_32bit(unsigned int reg)
{
	return reg >= AD5940_REG32_MIN && reg <= AD5940_REG32_MAX;
}

static void ad5940_bus_set_addr(struct ad5940_bus *bus,
				struct spi_transfer *xfer, unsigned int reg)
{
	bus->setaddr[0] = AD5940_SPICMD_SETADDR;
	put_unaligned_be16(reg, &bus->setaddr[1]);

	xfer->tx_buf = bus->setaddr;
	xfer->len = sizeof(bus->setaddr);
	xfer->cs_change = 1;
	xfer->cs_change_delay.value = 1;
	xfer->cs_change_delay.unit = SPI_DELAY_UNIT_USECS;
}

static int ad5940_bus_write(void *context, const void *data, size_t count)
{
	struct ad5940_bus *bus = context;
	struct spi_transfer xfers[2] = { };
	unsigned int reg, len;

	/* data is a big-endian 16-bit address followed by a 32-bit value. */
	if (count != 6)
		return -EINVAL;

	reg = get_unaligned_be16(data);
	ad5940_bus_set_addr(bus, &xfers[0], reg);

	/* 16-bit registers only use the two least significant value bytes. */
	len = ad5940_reg_is_32bit(reg) ? 4 : 2;
	bus->wr[0] = AD5940_SPICMD_WRITEREG;
	memcpy(&bus->wr[1], data + 2 + (4 - len), len);

	xfers[1].tx_buf = bus->wr;
	xfers[1].len = 1 + len;

	return spi_sync_transfer(bus->spi, xfers, ARRAY_SIZE(xfers));
}

static int ad5940_bus_read(void *context, const void *reg_buf, size_t reg_size,
			   void *val_buf, size_t val_size)
{
	struct ad5940_bus *bus = context;
	struct spi_transfer xfers[3] = { };
	unsigned int reg, len;
	int ret;

	if (reg_size != 2 || val_size != 4)
		return -EINVAL;

	reg = get_unaligned_be16(reg_buf);
	ad5940_bus_set_addr(bus, &xfers[0], reg);

	len = ad5940_reg_is_32bit(reg) ? 4 : 2;
	/* The command is followed by one dummy byte before data is returned. */
	bus->rd_cmd[0] = AD5940_SPICMD_READREG;
	bus->rd_cmd[1] = 0;

	xfers[1].tx_buf = bus->rd_cmd;
	xfers[1].len = sizeof(bus->rd_cmd);
	xfers[2].rx_buf = bus->rd;
	xfers[2].len = len;

	ret = spi_sync_transfer(bus->spi, xfers, ARRAY_SIZE(xfers));
	if (ret)
		return ret;

	memset(val_buf, 0, val_size);
	memcpy(val_buf + (val_size - len), bus->rd, len);

	return 0;
}

static const struct regmap_bus ad5940_regmap_bus = {
	.write = ad5940_bus_write,
	.read = ad5940_bus_read,
	.reg_format_endian_default = REGMAP_ENDIAN_BIG,
	.val_format_endian_default = REGMAP_ENDIAN_BIG,
};

static const struct regmap_config ad5940_regmap_config = {
	.reg_bits = 16,
	.val_bits = 32,
	.max_register = AD5940_MAX_REGISTER,
	.use_single_read = true,
	.use_single_write = true,
	.cache_type = REGCACHE_NONE,
};

static int ad5940_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			     unsigned int writeval, unsigned int *readval)
{
	struct ad5940_state *st = iio_priv(indio_dev);

	if (readval)
		return regmap_read(st->regmap, reg, readval);

	return regmap_write(st->regmap, reg, writeval);
}

static int ad5940_read_label(struct iio_dev *indio_dev,
			     const struct iio_chan_spec *chan, char *label)
{
	struct ad5940_state *st = iio_priv(indio_dev);
	const char *name = st->channel_config[chan->address].channel_name;

	if (name)
		return sysfs_emit(label, "%s\n", name);

	return sysfs_emit(label, "channel%lu\n", chan->address);
}

static const struct iio_chan_spec ad5940_channel_template = {
	.type = IIO_VOLTAGE,
	.differential = 1,
	.indexed = 1,
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |
				    BIT(IIO_CHAN_INFO_OFFSET),
};

static int ad5940_clear_ready(struct ad5940_state *st)
{
	int ret;

	ret = regmap_update_bits(st->regmap, AD5940_REG_AFECON,
				 AD5940_AFECON_ADCCONV_MSK,
				 AD5940_AFECON_ADCCONV_DIS);
	if (ret < 0)
		return ret;

	return regmap_write(st->regmap, AD5940_REG_INTCLR, AD5940_INTC_ADC);
}

static irqreturn_t ad5940_irq_handler(int irq, void *private)
{
	struct ad5940_state *st = private;
	int ret;

	/* The reader gets the error, if any, when it reads the result. */
	ret = ad5940_clear_ready(st);
	if (ret < 0)
		dev_err_ratelimited(&st->spi->dev,
				    "Failed to clear the ADC interrupt: %d\n",
				    ret);

	complete(&st->complete);

	return IRQ_HANDLED;
}

static int ad5940_scan_direct(struct ad5940_state *st, u32 mux, int *val)
{
	unsigned int result;
	int ret;

	guard(mutex)(&st->lock);

	ret = regmap_update_bits(st->regmap, AD5940_REG_ADCCON,
				 AD5940_ADCCON_MUX_MSK, mux);
	if (ret < 0)
		return ret;

	reinit_completion(&st->complete);
	ret = regmap_update_bits(st->regmap, AD5940_REG_AFECON,
				 AD5940_AFECON_ADCCONV_MSK,
				 AD5940_AFECON_ADCCONV_EN);
	if (ret < 0)
		return ret;

	if (!wait_for_completion_timeout(&st->complete,
					 msecs_to_jiffies(1000))) {
		ad5940_clear_ready(st);
		return -ETIMEDOUT;
	}

	ret = regmap_read(st->regmap, AD5940_REG_ADCDAT, &result);
	if (ret < 0)
		return ret;

	*val = result & 0xffff;

	return 0;
}

static int ad5940_read_raw(struct iio_dev *indio_dev,
			   const struct iio_chan_spec *chan, int *val,
			   int *val2, long info)
{
	struct ad5940_state *st = iio_priv(indio_dev);
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		ret = ad5940_scan_direct(st,
					 st->channel_config[chan->address].ain,
					 val);
		if (ret < 0)
			return ret;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_OFFSET:
		/* Codes are offset binary, 0 V is mid-scale. */
		*val = -AD5940_ADC_MIDSCALE;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = AD5940_VREF_MV;
		*val2 = 15;
		return IIO_VAL_FRACTIONAL_LOG2;
	default:
		return -EINVAL;
	}
}

static const struct iio_info ad5940_info = {
	.read_raw = &ad5940_read_raw,
	.read_label = &ad5940_read_label,
	.debugfs_reg_access = &ad5940_reg_access,
};

static const u8 ad5940_channel_p[] = {
	0, 1, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 16, 18, 19,
	20, 22, 23, 24, 25, 26, 31, 33, 35, 36
};

static const u8 ad5940_channel_n[] = {
	0, 1, 2, 4, 5, 6, 7, 8, 11, 12, 14, 16, 17, 20
};

static const u8 ad5940_int_io[] = { 0, 3, 6 };

static bool ad5940_in_list(const u8 *list, size_t len, u32 val)
{
	size_t i;

	for (i = 0; i < len; i++)
		if (list[i] == val)
			return true;

	return false;
}

static int ad5940_check_channel_indexes(struct ad5940_state *st,
					struct device *dev, u32 *ain)
{
	if (!ad5940_in_list(ad5940_channel_p, ARRAY_SIZE(ad5940_channel_p),
			    ain[0]) ||
	    (!st->info->has_ain6 && ain[0] == AD5940_AIN6_MUXP))
		return dev_err_probe(dev, -EINVAL,
				     "Invalid positive input %u\n", ain[0]);

	if (!ad5940_in_list(ad5940_channel_n, ARRAY_SIZE(ad5940_channel_n),
			    ain[1]) ||
	    (!st->info->has_ain6 && ain[1] == AD5940_AIN6_MUXN))
		return dev_err_probe(dev, -EINVAL,
				     "Invalid negative input %u\n", ain[1]);

	return 0;
}

static int ad5940_parse_channel_config(struct iio_dev *indio_dev)
{
	struct ad5940_state *st = iio_priv(indio_dev);
	struct device *dev = indio_dev->dev.parent;
	struct iio_chan_spec *chan;
	unsigned long *seen;
	u32 channel, ain[2];
	int ret;

	st->num_channels = device_get_child_node_count(dev);
	if (!st->num_channels)
		return dev_err_probe(dev, -ENODEV, "no channel children\n");

	chan = devm_kcalloc(dev, st->num_channels, sizeof(*chan), GFP_KERNEL);
	if (!chan)
		return -ENOMEM;

	st->channel_config = devm_kcalloc(dev, st->num_channels,
					  sizeof(*st->channel_config),
					  GFP_KERNEL);
	if (!st->channel_config)
		return -ENOMEM;

	seen = devm_bitmap_zalloc(dev, st->num_channels, GFP_KERNEL);
	if (!seen)
		return -ENOMEM;

	indio_dev->channels = chan;
	indio_dev->num_channels = st->num_channels;

	device_for_each_child_node_scoped(dev, child) {
		ret = fwnode_property_read_u32(child, "reg", &channel);
		if (ret)
			return dev_err_probe(dev, ret,
					     "Missing channel reg property\n");

		if (channel >= st->num_channels)
			return dev_err_probe(dev, -EINVAL,
					     "Invalid channel index %u\n",
					     channel);

		if (test_and_set_bit(channel, seen))
			return dev_err_probe(dev, -EINVAL,
					     "Duplicate channel index %u\n",
					     channel);

		ret = fwnode_property_read_u32_array(child, "diff-channels",
						     ain, 2);
		if (ret)
			return dev_err_probe(dev, ret,
					     "Missing diff-channels property\n");

		/* The label is optional, see ad5940_read_label(). */
		fwnode_property_read_string(child, "label",
					    &st->channel_config[channel].channel_name);

		ret = ad5940_check_channel_indexes(st, dev, ain);
		if (ret)
			return ret;

		st->channel_config[channel].ain = AD5940_CHANNEL_AINP(ain[0]) |
						  AD5940_CHANNEL_AINN(ain[1]);

		*chan = ad5940_channel_template;
		chan->address = channel;
		chan->channel = ain[0];
		chan->channel2 = ain[1];

		chan++;
	}

	return 0;
}

static int ad5940_config_polarity(struct ad5940_state *st, u32 polarity)
{
	u32 val;

	if (polarity == IRQF_TRIGGER_RISING)
		val = AD5940_INTCPOL_POS;
	else
		val = AD5940_INTCPOL_NEG;

	return regmap_update_bits(st->regmap, AD5940_REG_INTCPOL,
				  AD5940_INTCPOL_MSK, val);
}

static int ad5940_config_int_io(struct ad5940_state *st, u8 int_io)
{
	int ret = 0;

	if (int_io == 3)
		ret = regmap_update_bits(st->regmap, AD5940_REG_GP0CON,
					 AD5940_GP0CON_3_MSK,
					 AD5940_GP0CON_3_INT);
	else if (int_io == 6)
		ret = regmap_update_bits(st->regmap, AD5940_REG_GP0CON,
					 AD5940_GP0CON_6_MSK,
					 AD5940_GP0CON_6_INT);
	if (ret < 0)
		return ret;

	return regmap_write(st->regmap, AD5940_REG_GP0OEN, BIT(int_io));
}

/* Initialization sequence required after each reset, see Table 14. */
static const u32 ad5940_powerup_setting[][2] = {
	{ 0x0908, 0x02c9 },
	{ 0x0c08, 0x206c },
	{ 0x21f0, 0x0010 },
	{ 0x0410, 0x02c9 },
	{ 0x0a28, 0x0009 },
	{ 0x238c, 0x0104 },
	{ 0x0a04, 0x4859 },
	{ 0x0a04, 0xf27b },
	{ 0x0a00, 0x8009 },
	{ 0x22f0, 0x0000 },
};

static void ad5940_hw_reset(struct ad5940_state *st)
{
	if (!st->reset_gpio)
		return;

	gpiod_set_value_cansleep(st->reset_gpio, 1);
	fsleep(AD5940_RESET_PULSE_US);
	gpiod_set_value_cansleep(st->reset_gpio, 0);
	msleep(AD5940_RESET_WAIT_MS);
}

static int ad5940_setup(struct ad5940_state *st, u8 int_io, u32 trig_type)
{
	struct device *dev = &st->spi->dev;
	unsigned int chip_id;
	int ret;
	u8 i;

	ad5940_hw_reset(st);

	/*
	 * The AFE may be hibernating and wakes up on bus activity, so the
	 * result of the first register access is discarded.
	 */
	regmap_read(st->regmap, AD5940_AFECON_CHIPID, &chip_id);

	for (i = 0; i < ARRAY_SIZE(ad5940_powerup_setting); i++) {
		ret = regmap_write(st->regmap, ad5940_powerup_setting[i][0],
				   ad5940_powerup_setting[i][1]);
		if (ret < 0)
			return ret;
	}

	ret = regmap_read(st->regmap, AD5940_AFECON_CHIPID, &chip_id);
	if (ret < 0)
		return ret;
	/* The silicon revision may differ, only check the part ID. */
	if (FIELD_GET(AD5940_CHIPID_PARTID_MSK, chip_id) != AD5940_PARTID)
		return dev_err_probe(dev, -ENXIO,
				     "Wrong chip ID 0x%x\n", chip_id);

	ret = ad5940_config_polarity(st, trig_type);
	if (ret < 0)
		return ret;

	ret = regmap_write(st->regmap, AD5940_REG_PMBW,
			   AD5940_PMBW_LP | AD5940_PMBW_BW250);
	if (ret < 0)
		return ret;

	ret = ad5940_config_int_io(st, int_io);
	if (ret < 0)
		return ret;

	ret = ad5940_clear_ready(st);
	if (ret < 0)
		return ret;

	ret = regmap_write(st->regmap, AD5940_REG_INTCSEL0, AD5940_INTC_ADC);
	if (ret < 0)
		return ret;

	ret = regmap_update_bits(st->regmap, AD5940_REG_ADCCON,
				 AD5940_ADCCON_PGA_MSK, AD5940_ADCCON_PGA_1);
	if (ret < 0)
		return ret;

	return regmap_update_bits(st->regmap, AD5940_REG_AFECON,
				  AD5940_AFECON_ADCEN_MSK, AD5940_AFECON_ADCEN);
}

static void ad5940_power_down(void *data)
{
	struct ad5940_state *st = data;

	regmap_update_bits(st->regmap, AD5940_REG_AFECON,
			   AD5940_AFECON_ADCEN_MSK | AD5940_AFECON_ADCCONV_MSK, 0);
}

static int ad5940_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct iio_dev *indio_dev;
	struct ad5940_state *st;
	struct ad5940_bus *bus;
	u32 trig_type;
	u32 int_io;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->info = spi_get_device_match_data(spi);
	if (!st->info)
		return -ENODEV;

	ret = device_property_read_u32(dev, "adi,interrupt-io", &int_io);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to read adi,interrupt-io\n");

	if (!ad5940_in_list(ad5940_int_io, ARRAY_SIZE(ad5940_int_io), int_io) ||
	    int_io >= st->info->num_gpios)
		return dev_err_probe(dev, -EINVAL,
				     "Invalid interrupt output pin %u\n",
				     int_io);

	st->spi = spi;
	mutex_init(&st->lock);
	init_completion(&st->complete);

	bus = devm_kzalloc(dev, sizeof(*bus), GFP_KERNEL);
	if (!bus)
		return -ENOMEM;

	bus->spi = spi;

	st->regmap = devm_regmap_init(dev, &ad5940_regmap_bus, bus,
				      &ad5940_regmap_config);
	if (IS_ERR(st->regmap))
		return dev_err_probe(dev, PTR_ERR(st->regmap),
				     "Failed to initialize regmap\n");

	st->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(st->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(st->reset_gpio),
				     "Failed to get reset GPIO\n");

	trig_type = irq_get_trigger_type(spi->irq);
	if (trig_type != IRQF_TRIGGER_RISING &&
	    trig_type != IRQF_TRIGGER_FALLING)
		return dev_err_probe(dev, -EINVAL,
				     "Trigger type must be rising or falling\n");

	ret = devm_regulator_get_enable(dev, "avdd");
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable AVDD\n");

	/* DVDD must be driven before IOVDD is enabled. */
	ret = devm_regulator_get_enable(dev, "dvdd");
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable DVDD\n");

	ret = devm_regulator_get_enable(dev, "iovdd");
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable IOVDD\n");

	indio_dev->name = st->info->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ad5940_info;

	ret = ad5940_parse_channel_config(indio_dev);
	if (ret < 0)
		return ret;

	ret = ad5940_setup(st, int_io, trig_type);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, ad5940_power_down, st);
	if (ret)
		return ret;

	ret = devm_request_threaded_irq(dev, spi->irq, NULL,
					ad5940_irq_handler,
					trig_type | IRQF_ONESHOT,
					dev_name(dev), st);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to request IRQ\n");

	return devm_iio_device_register(dev, indio_dev);
}

static const struct of_device_id ad5940_of_match[] = {
	{ .compatible = "adi,ad5940", .data = &ad5940_chip_info },
	{ .compatible = "adi,ad5941", .data = &ad5941_chip_info },
	{ }
};
MODULE_DEVICE_TABLE(of, ad5940_of_match);

static const struct spi_device_id ad5940_id[] = {
	{ "ad5940", (kernel_ulong_t)&ad5940_chip_info },
	{ "ad5941", (kernel_ulong_t)&ad5941_chip_info },
	{ }
};
MODULE_DEVICE_TABLE(spi, ad5940_id);

static struct spi_driver ad5940_driver = {
	.driver = {
		.name = "ad5940",
		.of_match_table = ad5940_of_match,
	},
	.probe = ad5940_probe,
	.id_table = ad5940_id,
};
module_spi_driver(ad5940_driver);

MODULE_AUTHOR("Song Qiang <songqiang1304521@gmail.com>");
MODULE_DESCRIPTION("Analog Devices AD5940/AD5941 ADC driver");
MODULE_LICENSE("GPL");
