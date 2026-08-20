// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2022 Analog Devices, Inc.
 * Author: Cosmin Tanislav <cosmin.tanislav@analog.com>
 */

#include <linux/atomic.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/component.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dmaengine.h>
#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/spi/legacy-spi-engine.h>
#include <linux/units.h>
#include <linux/err.h>

#include <linux/iio/buffer.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/iio/iio.h>

#include <linux/iio/sysfs.h>

#include <linux/spi/offload/consumer.h>
#include <linux/spi/offload/types.h>

/*
 * Data interface timing with gated DCLK, slave mode (datasheet Table 3),
 * expressed in fSYSCLK periods where applicable (tDIGCLK = 2/fSYSCLK):
 *
 *   t1  ODR high                    >= 3 x tDIGCLK = 6/fSYSCLK
 *   t3  ODR fall -> first DCLK rise >= 8 ns
 *   t4  last DCLK fall -> ODR rise  >= 2 x tDCLK
 *
 * The t1 figure of 6 is a floor, not a target: at fSYSCLK = 48 MHz it lands on
 * 125.0 ns, which is the spec limit with zero margin. Program 7 (145.8 ns) so
 * the falling edge keeps one full sysclk period of slack. Raising this costs
 * ODR period budget, see ad4134_min_odr_period_ns().
 */
#define AD4134_ODR_HIGH_CYCLES			7
#define AD4134_ODR_FALL_TO_DCLK_RISE_NS		8

/*
 * DEBUG INSTRUMENTATION - remove once the frame-slip investigation closes.
 *
 * Module-wide, so an access to the slave while the master is streaming is
 * still tagged as happening during a capture. regmap calls writeable_reg /
 * readable_reg on every single access (no cache, no register tables), so
 * those two callbacks see all SPI register traffic to either chip, whatever
 * the source: this driver, sysfs, debugfs or iio_reg.
 */
static atomic_t ad4134_capture_active = ATOMIC_INIT(0);

#define AD4134_MIN_ODR_FREQ_HZ			10
#define AD4134_MAX_ODR_FREQ_HZ			(1496 * HZ_PER_KHZ)

#define AD4134_SPI_MAX_XFER_LEN			3
#define AD4134_NUM_CHANNELS			4
#define AD4134_CHAN_PRECISION_BITS		24

#define AD4134_NAME				"ad4134"

#define AD4134_IF_CONFIG_B_REG				0x01
#define AD4134_IF_CONFIG_B_SINGLE_INSTR			BIT(7)
#define AD4134_IF_CONFIG_B_MASTER_SLAVE_RD_CTRL		BIT(5)
#define AD4134_IF_CONFIG_B_RESET			BIT(1)

#define AD4134_DEVICE_CONFIG_REG		0x02
#define AD4134_DEVICE_CONFIG_POWER_MODE_MASK	BIT(0)
#define AD4134_POWER_MODE_HIGH_PERF		0b1

/*
 * DEVICE_STATUS (datasheet Table, reg 0x15). STAT_PLL_LOCK (bit 0) reads 1 once
 * the ASRC digital PLL has locked to the ODR input in slave mode. The datasheet
 * (ASRC Slave Mode / Multidevice Synchronization) requires polling this bit
 * before reading data; any ODR change unlocks and re-locks the PLL.
 */
#define AD4134_DEVICE_STATUS_REG		0x15
#define AD4134_STAT_PLL_LOCK			BIT(0)
#define AD4134_PLL_LOCK_POLL_US			1000
#define AD4134_PLL_LOCK_TIMEOUT_US		(2 * MEGA)

#define AD4134_DATA_PACKET_CONFIG_REG		0x11
#define AD4134_DATA_PACKET_CONFIG_FRAME_MASK	GENMASK(5, 4)
#define AD4134_DATA_PACKET_16BIT_FRAME		0x0
#define AD4134_DATA_PACKET_16BIT_CRC6_FRAME	0x1
#define AD4134_DATA_PACKET_24BIT_FRAME		0x2
#define AD4134_DATA_PACKET_24BIT_CRC6_FRAME	0x3

#define AD4134_DIG_IF_CFG_REG			0x12
#define AD4134_DIF_IF_CFG_FORMAT_MASK		GENMASK(1, 0)
#define AD4134_DATA_FORMAT_QUAD_CH_PARALLEL	0b10

/*
 * Sequence.txt §F.8.c: write 0x02 to register 0x13 powers down the chip's
 * internal LDO regulators. The eval board provides external regulators
 * (avdd5/avdd1v8/iovdd/refin) so the internal LDOs are unused; powering
 * them down reduces noise.
 */
#define AD4134_POWER_CONFIG_REG			0x13
#define AD4134_POWER_CONFIG_LDO_PD		0x02

#define AD4134_CHAN_DIG_FILTER_SEL_REG			0x1E
#define AD4134_CHAN_DIG_FILTER_SEL_MASK			GENMASK(7, 0)
#define AD4134_CHAN_DIG_FILTER_SEL_FRAME_MASK_CH0	GENMASK(1, 0)
#define AD4134_CHAN_DIG_FILTER_SEL_FRAME_MASK_CH1	GENMASK(3, 2)
#define AD4134_CHAN_DIG_FILTER_SEL_FRAME_MASK_CH2	GENMASK(5, 4)
#define AD4134_CHAN_DIG_FILTER_SEL_FRAME_MASK_CH3	GENMASK(7, 6)

#define AD4134_GPIO_INPUT(x)			0x00
#define AD4134_GPIO_OUTPUT(x)			BIT(x)
#define AD4134_GPIO_DIR_CONTROL			0x20
#define AD4134_GPIO_DATA			0x21

#define AD4134_SINC6_FILTER			0b01010101

#define AD4134_ODR_MIN				10
#define AD4134_ODR_MAX				1496000

#define AD4134_RESET_TIME_US			1000

/*
 * clkin_aligner IP register map (HDL: hdl/library/clkin_aligner). The IP is a
 * side-band controller for the 48 MHz XTAL2_CLKIN that gates clk_in through a
 * BUFGCE and tracks /32-divider phase + delivered edge counts. It implements
 * the hardware side of Sequence.txt: §E (one-time 36-cycle startup) and §F
 * (deterministic stop-at-/32-boundary across SPI power-mode writes, then
 * resume and IRQ at EDGE_TARGET). EDGE_TARGET is 137 in the bitstream: it
 * anchors odr_sync so the first ODR pulse lands on CLKin edge 146 (the
 * AD7134's first dig_clk rising edge) after the measured +9.5-cycle
 * anchor-to-ODR pipeline. The IRQ therefore fires at edge 137, ~187 ns
 * before dig_clk rises — used only as a "clock aligned, proceed" signal.
 *
 * Offsets are byte addresses on the AXI-Lite slave (word offset * 4).
 */
#define AD4134_CLKIN_RSTN			0x040 /* word 0x10 */
#define AD4134_CLKIN_CONTROL			0x044 /* word 0x11 */
#define   AD4134_CLKIN_CTRL_STARTUP_FIRE		BIT(0)
#define   AD4134_CLKIN_CTRL_ARM_STOP_AT_ALIGN		BIT(1)
#define   AD4134_CLKIN_CTRL_RESUME			BIT(2)
#define   AD4134_CLKIN_CTRL_GATE_FORCE_OFF		BIT(3)
#define   AD4134_CLKIN_CTRL_GATE_FORCE_ON		BIT(4)
#define AD4134_CLKIN_STATUS			0x048 /* word 0x12 */
#define AD4134_CLKIN_STARTUP_CYCLES		0x04C /* word 0x13 — bitstream default 39 (Fused_part_sequence) */
#define AD4134_CLKIN_EDGE_TARGET		0x050 /* word 0x14 — bitstream default 137 (anchors ODR to CLKin edge 146) */
#define AD4134_CLKIN_EDGE_COUNT			0x054 /* word 0x15 */
#define AD4134_CLKIN_DIV32_PHASE		0x058 /* word 0x16 */
#define AD4134_CLKIN_IRQ_PENDING		0x05C /* word 0x17, RW1C */
#define   AD4134_CLKIN_IRQ_EDGE_TARGET_HIT		BIT(0)
#define   AD4134_CLKIN_IRQ_STOP_AT_ALIGN_DONE		BIT(1)
#define   AD4134_CLKIN_IRQ_STARTUP_DONE			BIT(2)
#define AD4134_CLKIN_IRQ_MASK			0x060 /* word 0x18 */

enum {
	ODR_SET_FREQ,
};

enum ad4134_regulators {
	AD4134_AVDD5_REGULATOR,
	AD4134_AVDD1V8_REGULATOR,
	AD4134_IOVDD_REGULATOR,
	AD4134_REFIN_REGULATOR,
	AD4134_NUM_REGULATORS
};

/* maps adi,adc-frame property value to enum */
static const char * const ad4134_frame_config[] = {
	[AD4134_DATA_PACKET_16BIT_FRAME] = "16-bit",
	[AD4134_DATA_PACKET_16BIT_CRC6_FRAME] = "16-bit+CRC",
	[AD4134_DATA_PACKET_24BIT_FRAME] = "24-bit",
	[AD4134_DATA_PACKET_24BIT_CRC6_FRAME] = "24-bit+CRC",
};

enum ad7134_flt_type {
	WIDEBAND,
	SINC6,
	SINC3,
	SINC3_REJECTION
};

static const char * const ad7134_filter_enum[] = {
	[WIDEBAND] = "WIDEBAND",
	[SINC6] = "SINC6",
	[SINC3] = "SINC3",
	[SINC3_REJECTION] = "SINC3_REJECTION",
};

#define AD4134_CHANNEL(_index, _realbits, _storebits, _ext_info) {		\
	.type = IIO_VOLTAGE,							\
	.indexed = 1,								\
	.channel = (_index),							\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ) |		\
				    BIT(IIO_CHAN_INFO_SCALE),			\
	.info_mask_shared_by_type_available = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = (_index),							\
	.scan_type = {								\
		.sign = 's',							\
		.realbits = (_realbits),					\
		.storagebits = 32,						\
		.shift = 0,							\
		.endianness = IIO_CPU,						\
	},									\
	.ext_info = _ext_info,							\
}

static ssize_t ad7134_set_sync(struct iio_dev *indio_dev, uintptr_t private,
			       const struct iio_chan_spec *chan,
			       const char *buf, size_t len);
static ssize_t ad7134_get_sync(struct iio_dev *indio_dev, uintptr_t private,
			       const struct iio_chan_spec *chan, char *buf);
static int ad7134_set_dig_fil(struct iio_dev *dev,
			      const struct iio_chan_spec *chan,
			      unsigned int filter);
static int ad7134_get_dig_fil(struct iio_dev *dev,
			      const struct iio_chan_spec *chan);

static const struct iio_enum ad7134_flt_type_iio_enum = {
	.items = ad7134_filter_enum,
	.num_items = ARRAY_SIZE(ad7134_filter_enum),
	.set = ad7134_set_dig_fil,
	.get = ad7134_get_dig_fil,
};

static struct iio_chan_spec_ext_info ad7134_ext_info[] = {
	IIO_ENUM("filter_type", IIO_SHARED_BY_ALL, &ad7134_flt_type_iio_enum),
	IIO_ENUM_AVAILABLE("filter_type", IIO_SHARED_BY_ALL, &ad7134_flt_type_iio_enum),
	{
	 .name = "ad4134_sync",
	 .write = ad7134_set_sync,
	 .read = ad7134_get_sync,
	 .shared = IIO_SHARED_BY_ALL,
	 },
	{ },
};

#define AD4134_CHAN_SET(_realbits, _storebits) {				\
	AD4134_CHANNEL(0, _realbits, _storebits, ad7134_ext_info),		\
	AD4134_CHANNEL(1, _realbits, _storebits, ad7134_ext_info),		\
	AD4134_CHANNEL(2, _realbits, _storebits, ad7134_ext_info),		\
	AD4134_CHANNEL(3, _realbits, _storebits, ad7134_ext_info),		\
}

#define AD4134_DUO_CHAN_SET(_realbits, _storebits) {				\
	AD4134_CHANNEL(0, _realbits, _storebits, ad7134_ext_info),		\
	AD4134_CHANNEL(1, _realbits, _storebits, ad7134_ext_info),		\
	AD4134_CHANNEL(2, _realbits, _storebits, ad7134_ext_info),		\
	AD4134_CHANNEL(3, _realbits, _storebits, ad7134_ext_info),		\
	AD4134_CHANNEL(4, _realbits, _storebits, ad7134_ext_info),		\
	AD4134_CHANNEL(5, _realbits, _storebits, ad7134_ext_info),		\
	AD4134_CHANNEL(6, _realbits, _storebits, ad7134_ext_info),		\
	AD4134_CHANNEL(7, _realbits, _storebits, ad7134_ext_info),		\
}

static const struct iio_chan_spec ad4134_16_chan_set[] = AD4134_CHAN_SET(16, 16);
static const struct iio_chan_spec ad4134_16CRC_chan_set[] = AD4134_CHAN_SET(16, 24);
static const struct iio_chan_spec ad4134_24_chan_set[] = AD4134_CHAN_SET(24, 24);
static const struct iio_chan_spec ad4134_24CRC_chan_set[] = AD4134_CHAN_SET(24, 32);

static const struct iio_chan_spec ad4134_16_duo_chan_set[] = AD4134_DUO_CHAN_SET(16, 16);
static const struct iio_chan_spec ad4134_16CRC_duo_chan_set[] = AD4134_DUO_CHAN_SET(16, 24);
static const struct iio_chan_spec ad4134_24_duo_chan_set[] = AD4134_DUO_CHAN_SET(24, 24);
static const struct iio_chan_spec ad4134_24CRC_duo_chan_set[] = AD4134_DUO_CHAN_SET(24, 32);

static const unsigned long ad4134_channel_masks[] = {
	GENMASK(ARRAY_SIZE(ad4134_16_chan_set) - 1, 0),
	0,
};

static const unsigned long ad4134_duo_channel_masks[] = {
	GENMASK(ARRAY_SIZE(ad4134_16_duo_chan_set) - 1, 0),
	0,
};

struct ad4134_state {
	struct fwnode_handle		*spi_engine_fwnode;
	struct regmap			*regmap;
	struct spi_device		*spi;
	struct spi_device		*spi_engine;

	unsigned long sys_clk_hz;

	struct spi_transfer xfers;
	struct spi_message msg;

	struct spi_offload *offload;
	struct spi_offload_trigger *offload_trigger;
	struct spi_offload_trigger_config offload_trigger_config;
	struct pwm_device *odr_trigger;
	struct pwm_waveform odr_wf;
	unsigned int odr_hz;

	struct regulator_bulk_data	regulators[AD4134_NUM_REGULATORS];

	/*
	 * Synchronize access to members the of driver state, and ensure
	 * atomicity of consecutive regmap operations.
	 */
	struct mutex			lock;

	struct spi_message		buf_read_msg;
	struct gpio_desc		*cs_gpio;
	struct gpio_desc		*reset_gpio;
	struct gpio_chip		gpiochip;

	unsigned int			filter_type;
	unsigned long			sys_clk_rate;
	int				refin_mv;
	int				output_frame;
	u8 num_dout_lines;
	bool ad4134_duo;

	/*
	 * clkin_aligner side-band controller (Sequence.txt §E/§F).
	 * clkin_present is set true only after probe-time mapping and IRQ
	 * request both succeed; all clkin_aligner accesses are gated on it
	 * so the driver still works on bitstreams without the IP.
	 */
	void __iomem		*clkin_base;
	int			clkin_irq;
	struct completion	clkin_startup_done;
	struct completion	clkin_align_stopped;
	struct completion	clkin_edge_hit;
	bool			clkin_present;
};

static ssize_t ad7134_get_sync(struct iio_dev *indio_dev, uintptr_t private,
			       const struct iio_chan_spec *chan, char *buf)
{
	return sprintf(buf, "enable\n");
}

static int ad7134_sync(struct iio_dev *indio_dev)
{
	struct ad4134_state *st = iio_priv(indio_dev);
	int ret;

	/*
	 * Per AD7134 datasheet (Multidevice Synchronization section):
	 *   "This DIG_IF_RESET command must be given to all the slaves
	 *    simultaneously using one single SPI write command."
	 *
	 * We use a plain regmap_write (NOT regmap_update_bits) because:
	 *   - regmap_update_bits is a read-modify-write — two SPI transactions,
	 *     not one. The datasheet explicitly requires a single write.
	 *   - In broadcast-CS mode (cs_gpio HIGH) the read in update_bits has
	 *     MISO bus contention from both chips driving SDO simultaneously,
	 *     producing garbage that flips bit 5 (MASTER_SLAVE_RD_CTRL) to
	 *     random values and toggles chip readback configuration between
	 *     sync calls. This was observed in the field as alternating
	 *     "aligned / large-delay" behaviour on consecutive sync presses.
	 *
	 * Value 0x82 explicitly:
	 *   bit 7 SINGLE_INSTR        = 1 (preserve chip's reset default)
	 *   bit 5 MASTER_SLAVE_RD_CTRL = 0 (force default; slave-FF readback)
	 *   bit 1 DIG_IF_RESET         = 1 (trigger the sync — self-clearing)
	 *   all other bits             = 0 (defaults / reserved)
	 */
	gpiod_set_value_cansleep(st->cs_gpio, 1);
	ret = regmap_write(st->regmap, AD4134_IF_CONFIG_B_REG, 0x82);
	if (ret)
		return ret;

	gpiod_set_value_cansleep(st->cs_gpio, 0);

	return 0;
}

static ssize_t ad7134_set_sync(struct iio_dev *indio_dev, uintptr_t private,
			       const struct iio_chan_spec *chan,
			       const char *buf, size_t len)
{
	int ret;

	ret = ad7134_sync(indio_dev);

	return ret ? ret : len;
}

static int ad7134_set_dig_fil(struct iio_dev *dev,
			      const struct iio_chan_spec *chan,
			      unsigned int filter)
{
	struct ad4134_state *st = iio_priv(dev);
	int ret;

	st->filter_type = filter;
	gpiod_set_value_cansleep(st->cs_gpio, 1);

	ret = regmap_update_bits(st->regmap, AD4134_CHAN_DIG_FILTER_SEL_REG,
				 AD4134_CHAN_DIG_FILTER_SEL_MASK,
				 FIELD_PREP(AD4134_CHAN_DIG_FILTER_SEL_FRAME_MASK_CH0, filter) |
				 FIELD_PREP(AD4134_CHAN_DIG_FILTER_SEL_FRAME_MASK_CH1, filter) |
				 FIELD_PREP(AD4134_CHAN_DIG_FILTER_SEL_FRAME_MASK_CH2, filter) |
				 FIELD_PREP(AD4134_CHAN_DIG_FILTER_SEL_FRAME_MASK_CH3, filter));

	gpiod_set_value_cansleep(st->cs_gpio, 0);

	if (ret)
		return ret;
	return 0;
}

static int ad7134_get_dig_fil(struct iio_dev *dev,
			      const struct iio_chan_spec *chan)
{
	struct ad4134_state *st = iio_priv(dev);
	int ret;
	unsigned int readval;

	ret = regmap_read(st->regmap, AD4134_CHAN_DIG_FILTER_SEL_REG, &readval);
	if (ret)
		return ret;

	return FIELD_GET(AD4134_CHAN_DIG_FILTER_SEL_FRAME_MASK_CH0, readval);
}

static int ad4134_samp_freq_avail[] = { AD4134_ODR_MIN, 1, AD4134_ODR_MAX };

/*
 * Hardcoded 32-bit storagebits because the currently available HDL only
 * supports that.
 */
#define AD4134_OFFLOAD_CHANNEL(_index) {					\
	.type = IIO_VOLTAGE,							\
	.indexed = 1,								\
	.channel = (_index),							\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),				\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),			\
	.scan_index = (_index),							\
	.scan_type = {								\
		.sign = 's',							\
		.storagebits = 32,						\
		.realbits = AD4134_CHAN_PRECISION_BITS,				\
		.endianness = IIO_CPU,						\
	},									\
}

/*
 * It's not possible for software to record when offloaded SPI transfers run so
 * no additional timestamp channel is added.
 */
static const struct iio_chan_spec ad4134_offload_chan_set[] = {
	AD4134_OFFLOAD_CHANNEL(0),
	AD4134_OFFLOAD_CHANNEL(1),
	AD4134_OFFLOAD_CHANNEL(2),
	AD4134_OFFLOAD_CHANNEL(3),
};

/* The chip converts and outputs all 4 channels on each sample request */
static const unsigned long ad4134_offload_scan_masks[] = {
	GENMASK(3, 0),
	0
};

/* DCLK cycles per data output lane in one frame (datasheet Table 34). */
static unsigned int ad4134_frame_bits(struct ad4134_state *st)
{
	switch (st->output_frame) {
	case 0:
		return 16;
	case 1:
	case 2:
		return 24;
	case 3:
		return 32;
	default:
		return 0;
	}
}

/*
 * Gated DCLK requires the whole frame plus turnaround to fit inside one ODR
 * period (datasheet, DCLK ERROR):
 *
 *   ODR period > tDCLK x Frame Size + 6 x max(tDCLK, tDIGCLK)
 *
 * Violating it sets ERR_DCLK (INTERNAL_ERROR bit 3). Returns 0 if the bound
 * cannot be computed.
 */
static u64 ad4134_min_odr_period_ns(struct ad4134_state *st)
{
	u64 tdclk_ps, tdigclk_ps;
	unsigned int frame_bits;
	u32 dclk_hz;

	frame_bits = ad4134_frame_bits(st);
	dclk_hz = st->spi_engine ? st->spi_engine->max_speed_hz : 0;
	if (!frame_bits || !dclk_hz || !st->sys_clk_hz)
		return 0;

	tdclk_ps = div64_ul(PICO, dclk_hz);
	tdigclk_ps = div64_ul(2 * (u64)PICO, st->sys_clk_hz);

	return DIV_ROUND_UP_ULL(frame_bits * tdclk_ps +
				6 * max(tdclk_ps, tdigclk_ps), 1000);
}

static int ad4134_setup_odr(struct ad4134_state *st, unsigned int freq_hz)
{
	struct pwm_waveform odr_wf = { };
	u64 odr_high_time_ns;
	unsigned int odr_hz;
	u64 target = 10;
	int ret;

	dev_info(&st->spi->dev, "%s: Updating conversion rate to %u Hz\n",
		 __func__, freq_hz);

	/*
	 * Every ODR pulse causes each of the 4 ADCs within the AD4134 chip to
	 * take a sample simultaneously. The peripheral then outputs the data
	 * from all those channels over one, two, or four data output lanes. If
	 * the controller can fetch data from multiple lanes, the throughput is
	 * increased proportionally to the number of data lanes in use.
	 * Conversely, when multiple data lanes are enabled, the requested
	 * sampling frequency can be reached with slower ODR frequencies. ?
	 */
	odr_hz = freq_hz;
	dev_info(&st->spi->dev, "ODR frequency: %u Hz (DOUT lines: %u)\n",
		odr_hz, st->num_dout_lines);
	if (odr_hz < AD4134_MIN_ODR_FREQ_HZ || odr_hz > AD4134_MAX_ODR_FREQ_HZ) {
		dev_err(&st->spi->dev, "ODR %u Hz out of range [%d, %lu]\n",
			odr_hz, AD4134_MIN_ODR_FREQ_HZ,
			(unsigned long)AD4134_MAX_ODR_FREQ_HZ);
		return -EINVAL;
	}

	odr_wf.period_length_ns = DIV_ROUND_UP_ULL(NSEC_PER_SEC, odr_hz);
	/*
	 * For an arbitrary system clock (fSYSCLK), the t1 minimum ODR high time
	 * is 6/fSYSCLK. Program AD4134_ODR_HIGH_CYCLES/fSYSCLK so the falling
	 * edge is not sitting exactly on the spec limit. Set the PWM duty cycle
	 * to keep ODR up for at least that long. If the rounded PWM's value is
	 * less than the minimum required, increase the target value by 10 and
	 * attempt to round the waveform again, until the minimum is reached.
	 */
	odr_high_time_ns = DIV_ROUND_UP_ULL(AD4134_ODR_HIGH_CYCLES * (u64)NANO,
					    st->sys_clk_hz);
	do {
		odr_wf.duty_length_ns = target;
		ret = pwm_round_waveform_might_sleep(st->odr_trigger, &odr_wf);
		if (ret)
			return ret;
		target += 10;
	} while (odr_wf.duty_length_ns < odr_high_time_ns);

	{
		u64 min_period_ns = DIV_ROUND_UP_ULL(NSEC_PER_SEC, odr_hz);
		u64 try_period = min_period_ns;

		while (odr_wf.period_length_ns < min_period_ns) {
			try_period += 10;
			odr_wf.period_length_ns = try_period;
			ret = pwm_round_waveform_might_sleep(st->odr_trigger,
							     &odr_wf);
			if (ret)
				return ret;
		}
	}

	/*
	 * PWM waveform rounding might also change the wave period. Double check
	 * the resulting ODR PWM period is valid.
	 */
	if (odr_wf.period_length_ns < 2 * odr_high_time_ns)
		return -EINVAL;

	ret = pwm_set_waveform_might_sleep(st->odr_trigger, &odr_wf, false);

	return 0;
}

static const struct spi_offload_config ad4134_offload_config = {
	.capability_flags = SPI_OFFLOAD_CAP_TRIGGER |
			    SPI_OFFLOAD_CAP_RX_STREAM_DMA,
};

static int ad4134_update_conversion_rate(struct ad4134_state *st,
					 unsigned int freq_hz)
{
	struct spi_offload_trigger_config *config = &st->offload_trigger_config;
	struct pwm_waveform odr_wf = { };
	u64 offload_offset_ns;
	u64 odr_high_time_ns;
	u64 min_offset_ns;
	u64 min_gated_ns;
	unsigned int odr_hz;
	u64 target = 10;
	int ret;

	/*
	 * Every ODR pulse causes each of the 4 ADCs within the AD4134 chip to
	 * take a sample simultaneously. The peripheral then outputs the data
	 * from all those channels over one, two, or four data output lanes. If
	 * the controller can fetch data from multiple lanes, the throughput is
	 * increased proportionally to the number of data lanes in use.
	 * Conversely, when multiple data lanes are enabled, the requested
	 * sampling frequency can be reached with slower ODR frequencies.
	 */
	odr_hz = freq_hz;
	if (odr_hz < AD4134_MIN_ODR_FREQ_HZ || odr_hz > AD4134_MAX_ODR_FREQ_HZ)
		return -EINVAL;

	odr_wf.period_length_ns = DIV_ROUND_UP_ULL(NSEC_PER_SEC, odr_hz);
	/*
	 * For an arbitrary system clock (fSYSCLK), the t1 minimum ODR high time
	 * is 6/fSYSCLK. Program AD4134_ODR_HIGH_CYCLES/fSYSCLK so the falling
	 * edge is not sitting exactly on the spec limit. Set the PWM duty cycle
	 * to keep ODR up for at least that long. If the rounded PWM's value is
	 * less than the minimum required, increase the target value by 10 and
	 * attempt to round the waveform again, until the minimum is reached.
	 */
	odr_high_time_ns = DIV_ROUND_UP_ULL(AD4134_ODR_HIGH_CYCLES * (u64)NANO,
					    st->sys_clk_hz);
	do {
		odr_wf.duty_length_ns = target;
		ret = pwm_round_waveform_might_sleep(st->odr_trigger, &odr_wf);
		if (ret)
			return ret;
		target += 10;
	} while (odr_wf.duty_length_ns < odr_high_time_ns);

	{
		u64 min_period_ns = DIV_ROUND_UP_ULL(NSEC_PER_SEC, odr_hz);
		u64 try_period = min_period_ns;

		while (odr_wf.period_length_ns < min_period_ns) {
			try_period += 10;
			odr_wf.period_length_ns = try_period;
			ret = pwm_round_waveform_might_sleep(st->odr_trigger,
							     &odr_wf);
			if (ret)
				return ret;
		}
	}

	dev_dbg(&st->spi->dev, "ad7134: ODR period=%llu ns, duty=%llu ns, odr_high_min=%llu ns\n",
		 odr_wf.period_length_ns, odr_wf.duty_length_ns, odr_high_time_ns);

	ret = pwm_set_waveform_might_sleep(st->odr_trigger, &odr_wf, false);
	if (ret)
		return -EINVAL;

	/*
	 * PWM waveform rounding might also change the wave period. Double check
	 * the resulting ODR PWM period is valid.
	 */
	if (odr_wf.period_length_ns < 2 * odr_high_time_ns)
		return -EINVAL;

	config->periodic.frequency_hz = div64_u64(NSEC_PER_SEC, odr_wf.period_length_ns);

	/*
	 * For gated DCLK, the minimum required time between ODR rising edge
	 * and DCLK rising edge is the sum of ODR high time and ODR falling
	 * edge to DCLK rising edge time (t3). Delay the offload trigger by at
	 * least that much so DCLK does not start before ODR has fallen.
	 *
	 * Reference the ODR high time the PWM actually rounded to, not the
	 * requested one.
	 */
	min_offset_ns = odr_wf.duty_length_ns + AD4134_ODR_FALL_TO_DCLK_RISE_NS;
	offload_offset_ns = min_offset_ns;
	do {
		config->periodic.offset_ns = offload_offset_ns;
		ret = spi_offload_trigger_validate(st->offload_trigger, config);
		if (ret)
			return ret;

		offload_offset_ns += 10;
	} while (config->periodic.offset_ns < min_offset_ns);

	/*
	 * round_waveform_fromhw() rounds up, so period_length_ns can overstate
	 * the real hardware period by up to 1 ns. Equation 1 is a strict
	 * inequality, so compare with <= to also flag a period sitting exactly
	 * on the bound.
	 */
	min_gated_ns = ad4134_min_odr_period_ns(st);
	if (min_gated_ns && odr_wf.period_length_ns <= min_gated_ns)
		dev_warn(&st->spi->dev,
			 "ODR period %llu ns is at or below the gated-DCLK minimum of %llu ns; expect ERR_DCLK and inter-device frame slip\n",
			 odr_wf.period_length_ns, min_gated_ns);

	dev_dbg(&st->spi->dev, "ad7134: trigger offset=%llu ns, trigger freq=%llu Hz, offload_period=%llu ns\n",
		 config->periodic.offset_ns, config->periodic.frequency_hz, odr_wf.period_length_ns);

	st->odr_wf = odr_wf;
	st->odr_hz = div64_u64(NSEC_PER_SEC, odr_wf.period_length_ns);

	return 0;
}

static ssize_t sampling_frequency_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct ad4134_state *st = iio_priv(dev_to_iio_dev(dev));

	return sysfs_emit(buf, "%u\n", st->odr_hz);
}

static ssize_t sampling_frequency_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad4134_state *st = iio_priv(indio_dev);
	unsigned int val;
	int ret;

	if (!iio_device_claim_direct(indio_dev))
		return -EBUSY;

	ret = kstrtouint(buf, 10, &val);
	if (ret)
		goto out_store;

	ret = ad4134_update_conversion_rate(st, val);

out_store:
	iio_device_release_direct(indio_dev);
	return ret ?: len;
}

static IIO_DEVICE_ATTR_RW(sampling_frequency, 0);

static ssize_t sampling_frequency_available_show(struct device *dev,
						 struct device_attribute *attr,
						 char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad4134_state *st = iio_priv(indio_dev);

	return sysfs_emit(buf, "[%u %u %lu]\n",
			  AD4134_MIN_ODR_FREQ_HZ * st->num_dout_lines, 1,
			  AD4134_MAX_ODR_FREQ_HZ * st->num_dout_lines);
}

static IIO_DEVICE_ATTR_RO(sampling_frequency_available, 0);

static struct attribute *ad4134_offload_attributes[] = {
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	NULL,
};

const struct attribute_group ad4134_offload_attribute_group = {
	.attrs = ad4134_offload_attributes,
};

static void ad4134_prepare_offload_msg(struct iio_dev *indio_dev)
{
	struct ad4134_state *st = iio_priv(indio_dev);
	unsigned int base_len = roundup_pow_of_two(BITS_TO_BYTES(AD4134_CHAN_PRECISION_BITS));
	unsigned int num_devices;
	unsigned int bpw;

	bpw = ad4134_frame_bits(st);
	if (!bpw) {
		dev_err(&st->spi->dev, "invalid adi,adc-frame: %d\n", st->output_frame);
		return;
	}
	num_devices = st->ad4134_duo ? 2 : 1;

	st->xfers.bits_per_word = bpw;
	st->xfers.len = base_len * st->num_dout_lines * num_devices;
	if (st->num_dout_lines > 1)
		st->xfers.multi_lane_mode = SPI_MULTI_LANE_MODE_STRIPE;

	st->xfers.offload_flags = SPI_OFFLOAD_XFER_RX_STREAM;

	spi_message_init_with_transfers(&st->msg, &st->xfers, 1);
}

static int ad4134_offload_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ad4134_state *st = iio_priv(indio_dev);
	int ret;

	ad4134_prepare_offload_msg(indio_dev);
	st->msg.offload = st->offload;
	ret = spi_optimize_message(st->spi_engine, &st->msg);
	if (ret)
		return ret;

	ret = spi_offload_trigger_enable(st->offload, st->offload_trigger,
					 &st->offload_trigger_config);
	if (ret)
		goto out_unoptimize;

	atomic_inc(&ad4134_capture_active);
	dev_info(&st->spi->dev, "ad7134: ===== CAPTURE START =====\n");

	return 0;

out_unoptimize:
	spi_unoptimize_message(&st->msg);

	return 0;
}

static int ad4134_offload_buffer_predisable(struct iio_dev *indio_dev)
{
	struct ad4134_state *st = iio_priv(indio_dev);

	dev_info(&st->spi->dev, "ad7134: ===== CAPTURE STOP =====\n");
	atomic_dec(&ad4134_capture_active);

	spi_offload_trigger_disable(st->offload, st->offload_trigger);

	spi_unoptimize_message(&st->msg);

	return 0;
}

static const struct iio_buffer_setup_ops ad4134_offload_buffer_setup_ops = {
	.postenable = &ad4134_offload_buffer_postenable,
	.predisable = &ad4134_offload_buffer_predisable,
};

static int ad4134_spi_offload_setup(struct iio_dev *indio_dev,
				    struct ad4134_state *st)
{
	struct device *offload_dev = &st->spi_engine->dev;
	struct device *dev = &st->spi->dev;
	struct dma_chan *rx_dma;

	st->offload_trigger = devm_spi_offload_trigger_get(offload_dev, st->offload,
							   SPI_OFFLOAD_TRIGGER_PERIODIC);
	if (IS_ERR(st->offload_trigger))
		return dev_err_probe(dev, PTR_ERR(st->offload_trigger),
				     "failed to get offload trigger\n");

	st->offload_trigger_config.type = SPI_OFFLOAD_TRIGGER_PERIODIC;

	rx_dma = devm_spi_offload_rx_stream_request_dma_chan(offload_dev, st->offload);
	if (IS_ERR(rx_dma))
		return dev_err_probe(offload_dev, PTR_ERR(rx_dma),
				     "failed to get offload RX DMA\n");

	return devm_iio_dmaengine_buffer_setup_with_handle(offload_dev,
							   indio_dev, rx_dma,
							   IIO_BUFFER_DIRECTION_IN);
}

static int ad4134_pwm_get(struct ad4134_state *st)
{
	struct device *dev = &st->spi->dev;

	st->odr_trigger = devm_pwm_get(dev, NULL);
	if (IS_ERR(st->odr_trigger))
		return dev_err_probe(dev, PTR_ERR(st->odr_trigger),
				     "failed to get ODR PWM\n");

	return 0;
}

static int ad4134_offload_buffer_setup(struct iio_dev *indio_dev, struct spi_device *spi)
{
	struct ad4134_state *st = iio_priv(indio_dev);
	struct device *offload_dev = &st->spi_engine->dev;
	struct device *dev = &spi->dev;
	int ret;

	st->offload = devm_spi_offload_get(offload_dev, st->spi_engine, &ad4134_offload_config);
	ret = PTR_ERR_OR_ZERO(st->offload);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get offload\n");

	ret = ad4134_spi_offload_setup(indio_dev, st);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to setup SPI offload\n");

	return 0;
}

static int ad4134_input_gpio(struct gpio_chip *chip, unsigned int offset)
{
	struct ad4134_state *st = gpiochip_get_data(chip);
	int ret;

	mutex_lock(&st->lock);
	ret = regmap_update_bits(st->regmap, AD4134_GPIO_DIR_CONTROL,
				 BIT(offset), AD4134_GPIO_INPUT(offset));

	mutex_unlock(&st->lock);

	return ret;
}

static int ad4134_output_gpio(struct gpio_chip *chip,
			      unsigned int offset, int value)
{
	struct ad4134_state *st = gpiochip_get_data(chip);
	int ret;

	mutex_lock(&st->lock);

	ret = regmap_update_bits(st->regmap, AD4134_GPIO_DIR_CONTROL,
				 BIT(offset), AD4134_GPIO_OUTPUT(offset));
	if (ret < 0)
		goto out;

	ret = regmap_update_bits(st->regmap, AD4134_GPIO_DATA, BIT(offset),
				 (value << offset));
out:
	mutex_unlock(&st->lock);

	return ret;
}

static int ad4134_get_gpio(struct gpio_chip *chip, unsigned int offset)
{
	struct ad4134_state *st = gpiochip_get_data(chip);
	unsigned int val;
	int ret;

	mutex_lock(&st->lock);
	ret = regmap_read(st->regmap, AD4134_GPIO_DIR_CONTROL, &val);
	if (ret < 0)
		goto out;

	ret = regmap_read(st->regmap, AD4134_GPIO_DATA, &val);
	if (ret < 0)
		goto out;

	ret = !!(val & BIT(offset));

out:
	mutex_unlock(&st->lock);

	return ret;
}

static void ad4134_set_gpio(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct ad4134_state *st = gpiochip_get_data(chip);
	unsigned int val;
	int ret;

	mutex_lock(&st->lock);
	ret = regmap_read(st->regmap, AD4134_GPIO_DIR_CONTROL, &val);
	if (ret < 0)
		goto out;

	if (val & BIT(offset))
		regmap_update_bits(st->regmap, AD4134_GPIO_DATA, BIT(offset),
				   (value << offset));

out:
	mutex_unlock(&st->lock);
}

static int ad4134_gpio_setup(struct ad4134_state *st)
{
	st->gpiochip.label = "ad4134";
	st->gpiochip.base = -1;
	st->gpiochip.ngpio = 8;
	st->gpiochip.parent = &st->spi->dev;
	st->gpiochip.can_sleep = true;
	st->gpiochip.direction_input = ad4134_input_gpio;
	st->gpiochip.direction_output = ad4134_output_gpio;
	st->gpiochip.get = ad4134_get_gpio;
	st->gpiochip.set = ad4134_set_gpio;

	return devm_gpiochip_add_data(&st->spi->dev, &st->gpiochip, st);
}

static int ad4134_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long info)
{
	struct ad4134_state *st = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_SCALE:
		*val = st->refin_mv;
		*val2 = chan->scan_type.realbits - 1;

		return IIO_VAL_FRACTIONAL_LOG2;
	case IIO_CHAN_INFO_SAMP_FREQ:
		mutex_lock(&st->lock);
		*val = st->odr_hz;
		mutex_unlock(&st->lock);

		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int ad4134_read_avail(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     const int **vals, int *type, int *length,
			     long info)
{
	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*vals = ad4134_samp_freq_avail;
		*type = IIO_VAL_INT;

		return IIO_AVAIL_RANGE;
	default:
		return -EINVAL;
	}
}

static int ad4134_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long info)
{
	struct ad4134_state *st = iio_priv(indio_dev);
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;

		mutex_lock(&st->lock);
		ret = ad4134_update_conversion_rate(st, val);
		mutex_unlock(&st->lock);

		iio_device_release_direct_mode(indio_dev);

		return ret;
	default:
		return -EINVAL;
	}
}

static int ad4134_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			     unsigned int writeval, unsigned int *readval)
{
	struct ad4134_state *st = iio_priv(indio_dev);

	if (readval)
		return regmap_read(st->regmap, reg, readval);

	return regmap_write(st->regmap, reg, writeval);
}

static const struct iio_info ad4134_info = {
	.read_raw = ad4134_read_raw,
	.read_avail = ad4134_read_avail,
	.write_raw = ad4134_write_raw,
	.attrs = &ad4134_offload_attribute_group,
	.debugfs_reg_access = ad4134_reg_access,
};

static int ad4134_get_ADC_count(struct ad4134_state *st)
{
	struct device *controller_dev = &st->spi->controller->dev;
	unsigned int adc_count = 0;

	device_for_each_child_node_scoped(controller_dev, child) {
		if (fwnode_property_match_string(child, "compatible",
						 "adi,ad4134") >= 0)
			adc_count++;
		if (fwnode_property_match_string(child, "compatible",
						 "adi,ad7134") >= 0)
			adc_count++;
	}
	return adc_count;
}

/*
 * Slave-side check: does a sibling ADC node carry the clkin_aligner phandle?
 * If so this device is the slave and must defer §E/§F to that master rather
 * than releasing its own reset against the free-running gate.
 */
static bool ad4134_master_has_clkin(struct ad4134_state *st)
{
	struct device *controller_dev = &st->spi->controller->dev;
	struct fwnode_handle *self = dev_fwnode(&st->spi->dev);

	device_for_each_child_node_scoped(controller_dev, child) {
		if (child == self)
			continue;
		if (fwnode_property_present(child, "adi,clkin-aligner"))
			return true;
	}
	return false;
}

/*
 * Master-side lookup: walk the SPI controller's children and return the
 * sibling ADC's reset GPIO so the master can release both chips' /RESETN
 * simultaneously during §E. Returns NULL until the sibling has probed far
 * enough to publish st->reset_gpio (the master EPROBE_DEFERs on that).
 */
struct ad4134_slave_reset_ctx {
	struct spi_device *self_spi;
	struct gpio_desc *reset_gpio;
};

static int ad4134_find_slave_reset(struct device *dev, void *data)
{
	struct ad4134_slave_reset_ctx *ctx = data;
	struct iio_dev *indio_dev;
	struct ad4134_state *sib;

	if (!dev->driver)
		return 0;

	if (to_spi_device(dev) == ctx->self_spi)
		return 0;

	indio_dev = dev_get_drvdata(dev);
	if (!indio_dev)
		return 0;

	sib = iio_priv(indio_dev);
	if (!sib->reset_gpio)
		return 0;

	ctx->reset_gpio = sib->reset_gpio;
	return 1;	/* stop iterating */
}

static struct gpio_desc *ad4134_get_slave_reset(struct ad4134_state *st)
{
	struct ad4134_slave_reset_ctx ctx = { .self_spi = st->spi };

	device_for_each_child(&st->spi->controller->dev, &ctx,
			      ad4134_find_slave_reset);
	return ctx.reset_gpio;
}

static void ad4134_disable_regulators(void *data)
{
	struct ad4134_state *st = data;

	regulator_bulk_disable(ARRAY_SIZE(st->regulators), st->regulators);
}

/*
 * Reapply the five config registers to the sibling ADC after the shared
 * /RESETN pulse wiped its state. Walked from the master node via
 * device_for_each_child(); skip self and any child whose driver hasn't
 * populated drvdata/regmap yet.
 *
 * This path exists only as a fallback for boards without the broadcast
 * cs_gpio (ad7134_fmc/zed has cs_gpio so the broadcast block in
 * ad4134_setup() handles slave configuration directly). The previous
 * PDN retry loop here was a workaround for the slave's PLL failing to
 * lock under free-running XTAL2_CLKIN — that failure mode is resolved
 * by §E giving both chips a deterministic clock startup, so the loop
 * is no longer needed.
 */
/*
 * Threaded ISR for the clkin_aligner IP.
 *
 * Each bit in CLKIN_IRQ_PENDING corresponds to one of the three Sequence.txt
 * events. We RW1C the latched bits and signal the matching completion so the
 * caller blocked in wait_for_completion_timeout() proceeds.
 *
 * Note on ordering: complete() after the RW1C write guarantees the line has
 * de-asserted by the time the waiter resumes — a subsequent reinit_completion
 * + arm sequence cannot accidentally re-trigger on a stale pending bit.
 */
static irqreturn_t ad4134_clkin_isr(int irq, void *data)
{
	struct ad4134_state *st = data;
	u32 pending;

	pending = readl(st->clkin_base + AD4134_CLKIN_IRQ_PENDING);
	if (!pending)
		return IRQ_NONE;

	/* RW1C: write 1s back to clear the latched event(s). */
	writel(pending, st->clkin_base + AD4134_CLKIN_IRQ_PENDING);

	if (pending & AD4134_CLKIN_IRQ_STARTUP_DONE)
		complete(&st->clkin_startup_done);
	if (pending & AD4134_CLKIN_IRQ_STOP_AT_ALIGN_DONE)
		complete(&st->clkin_align_stopped);
	if (pending & AD4134_CLKIN_IRQ_EDGE_TARGET_HIT)
		complete(&st->clkin_edge_hit);

	return IRQ_HANDLED;
}

/*
 * Mask all clkin_aligner interrupt sources. Registered via
 * devm_add_action_or_reset *after* devm_request_threaded_irq, which means it
 * runs *before* the IRQ is released on driver detach (devm unwinds in reverse
 * registration order). This guarantees no event can fire between the time the
 * driver state is being torn down and the time the IRQ line is freed.
 */
static void ad4134_clkin_mask(void *data)
{
	struct ad4134_state *st = data;

	writel(0, st->clkin_base + AD4134_CLKIN_IRQ_MASK);
}

static void ad4134_put_device(void *data)
{
	put_device(data);
}

/*
 * Look up the clkin_aligner platform device via the adi,clkin-aligner
 * phandle, map its AXI register block, and wire up the IRQ.  Sets
 * st->clkin_present on success so the rest of the driver can opt in to
 * Sequence.txt §E/§F flows when available, or fall back to legacy
 * uncontrolled XTAL2_CLKIN behaviour when the IP is not in the bitstream.
 *
 * The phandle target is a "raw" of_platform_populate node (no driver
 * binds to "adi,clkin-aligner"), so we acquire a reference via
 * of_find_device_by_node and release it on driver detach.
 */
static int ad4134_clkin_init(struct ad4134_state *st)
{
	struct device *dev = &st->spi->dev;
	struct platform_device *pdev;
	struct device_node *np;
	struct resource *res;
	void __iomem *base;
	int irq, ret;

	np = of_parse_phandle(dev->of_node, "adi,clkin-aligner", 0);
	if (!np) {
		dev_dbg(dev, "ad4134: no adi,clkin-aligner phandle, skipping\n");
		return 0;
	}

	pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!pdev)
		return -EPROBE_DEFER;

	/*
	 * Tie the pdev reference to our (SPI) device's devm so it is
	 * released on driver detach regardless of return path.
	 */
	ret = devm_add_action_or_reset(dev, ad4134_put_device, &pdev->dev);
	if (ret)
		return ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return dev_err_probe(dev, -EINVAL,
				     "clkin_aligner: missing MEM resource\n");

	base = devm_ioremap_resource(dev, res);
	if (IS_ERR(base))
		return dev_err_probe(dev, PTR_ERR(base),
				     "clkin_aligner: ioremap failed\n");

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return dev_err_probe(dev, irq,
				     "clkin_aligner: get_irq failed\n");

	st->clkin_base = base;
	st->clkin_irq  = irq;

	init_completion(&st->clkin_startup_done);
	init_completion(&st->clkin_align_stopped);
	init_completion(&st->clkin_edge_hit);

	/* Clear any latched events from a prior bitstream/driver load. */
	writel(AD4134_CLKIN_IRQ_STARTUP_DONE |
	       AD4134_CLKIN_IRQ_STOP_AT_ALIGN_DONE |
	       AD4134_CLKIN_IRQ_EDGE_TARGET_HIT,
	       base + AD4134_CLKIN_IRQ_PENDING);

	/* Unmask all three event sources. */
	writel(AD4134_CLKIN_IRQ_STARTUP_DONE |
	       AD4134_CLKIN_IRQ_STOP_AT_ALIGN_DONE |
	       AD4134_CLKIN_IRQ_EDGE_TARGET_HIT,
	       base + AD4134_CLKIN_IRQ_MASK);

	ret = devm_request_threaded_irq(dev, irq, NULL, ad4134_clkin_isr,
					IRQF_ONESHOT, "ad4134-clkin", st);
	if (ret)
		return dev_err_probe(dev, ret,
				     "clkin_aligner: request_irq failed\n");

	/*
	 * Order matters: this is registered after request_threaded_irq, so on
	 * detach the IRQ_MASK=0 write executes BEFORE the IRQ is freed.
	 */
	ret = devm_add_action_or_reset(dev, ad4134_clkin_mask, st);
	if (ret)
		return ret;

	st->clkin_present = true;
	return 0;
}

/*
 * Sequence.txt §E — one-time controlled startup of XTAL2_CLKIN.
 *
 * The chip's internal clock chain has an RC oscillator that hands over to
 * the external 48 MHz on the FIRST 36 edges of XTAL2_CLKIN it sees after
 * POR. If we let the chip wake up while the BUFGCE gate is open, that
 * handover happens at a non-deterministic point — defeating the purpose
 * of every later §F alignment. So this function owns the reset GPIO
 * release: gate is forced OFF before reset is dropped, so the chip
 * exits POR running on RC only, then we deliver exactly 36 cycles
 * through clkin_aligner.
 *
 * Caller has acquired reset_gpio with GPIOD_OUT_HIGH (chip in reset).
 *
 *   1. GATE_FORCE_OFF                         — block XTAL at the gate
 *   2. hold reset AD4134_RESET_TIME_US, release   — chip wakes on RC
 *   3. fsleep §E.2 POR (~3 ms conservative)
 *   4. STARTUP_FIRE (clears GATE_FORCE_OFF in the same write, since
 *      writel writes the full word and the strobe is bit[0] only)
 *      → FSM IDLE→STARTUP, 36 edges delivered, STOPPED_LOW; IRQ[2]
 *   5. fsleep §E.4 (20 µs clock-switch handshake)
 *   6. RESUME — XTAL runs continuously from known /32 phase per §C
 */
static int ad4134_clkin_startup(struct ad4134_state *st,
				struct gpio_desc *reset_gpio)
{
	struct device *dev = &st->spi->dev;
	struct gpio_desc *slave_reset = ad4134_get_slave_reset(st);
	unsigned long timeout;

	/*
	 * §E.0 — soft-reset the clkin_aligner FSM so STARTUP_FIRE is taken
	 * from ST_IDLE. On a driver reload (modprobe) a prior §E left the FSM
	 * in ST_RUNNING (RESUME), where STARTUP_FIRE is ignored and §E.3 hangs
	 * (startup_done TIMEOUT, DIV32=0x10x, EDGE=0xffff). RSTN bit[0] is
	 * active-high (1=assert, 0=release) and crosses into the ext_clk
	 * domain, so settle on both edges. STARTUP_CYCLES/EDGE_TARGET live in
	 * the up_clk domain and survive this reset.
	 */
	writel(1, st->clkin_base + AD4134_CLKIN_RSTN);
	fsleep(10);
	writel(0, st->clkin_base + AD4134_CLKIN_RSTN);
	fsleep(10);
	writel(AD4134_CLKIN_IRQ_STARTUP_DONE |
	       AD4134_CLKIN_IRQ_STOP_AT_ALIGN_DONE |
	       AD4134_CLKIN_IRQ_EDGE_TARGET_HIT,
	       st->clkin_base + AD4134_CLKIN_IRQ_PENDING);
	writel(AD4134_CLKIN_IRQ_STARTUP_DONE |
	       AD4134_CLKIN_IRQ_STOP_AT_ALIGN_DONE |
	       AD4134_CLKIN_IRQ_EDGE_TARGET_HIT,
	       st->clkin_base + AD4134_CLKIN_IRQ_MASK);

	writel(AD4134_CLKIN_CTRL_GATE_FORCE_OFF,
	       st->clkin_base + AD4134_CLKIN_CONTROL);

	/*
	 * §E.2 (§H) — both chips must exit reset with the gate already OFF so
	 * their RC->crystal handover happens on the controlled 36-edge burst,
	 * not on free-running edges. Assert both /RESETN, hold, then release
	 * both simultaneously. The slave was left asserted by its own probe;
	 * re-asserting here makes the simultaneous release unambiguous.
	 */
	gpiod_set_value_cansleep(reset_gpio, 1);
	if (slave_reset)
		gpiod_set_value_cansleep(slave_reset, 1);
	fsleep(AD4134_RESET_TIME_US);
	gpiod_set_value_cansleep(reset_gpio, 0);
	if (slave_reset)
		gpiod_set_value_cansleep(slave_reset, 0);

	/* §E.2 POR wait (chip on RC). */
	fsleep(3000);

	/* §E.3 STARTUP_FIRE — deliver 36 XTAL cycles. */
	reinit_completion(&st->clkin_startup_done);
	writel(AD4134_CLKIN_CTRL_STARTUP_FIRE,
	       st->clkin_base + AD4134_CLKIN_CONTROL);

	timeout = wait_for_completion_timeout(&st->clkin_startup_done,
					      msecs_to_jiffies(100));
	if (!timeout) {
		dev_err(dev,
			"ad4134: §E startup_done TIMEOUT (STATUS=0x%08x IRQ_PEND=0x%08x DIV32=0x%08x EDGE=0x%08x)\n",
			readl(st->clkin_base + AD4134_CLKIN_STATUS),
			readl(st->clkin_base + AD4134_CLKIN_IRQ_PENDING),
			readl(st->clkin_base + AD4134_CLKIN_DIV32_PHASE),
			readl(st->clkin_base + AD4134_CLKIN_EDGE_COUNT));
		return -ETIMEDOUT;
	}

	/* §E.4 handshake settle. */
	fsleep(20);

	/* §E.6 RESUME — XTAL runs continuously. */
	writel(AD4134_CLKIN_CTRL_RESUME,
	       st->clkin_base + AD4134_CLKIN_CONTROL);

	return 0;
}

/*
 * Sequence.txt §F — deterministic power-mode change.
 *
 * Wraps a single 1-byte SPI write with the alignment-preserving stop/resume
 * dance. The FSM must be in ST_RUNNING on entry (i.e. §E has completed and
 * the clock is free-running); on success it's back in ST_RUNNING with the
 * edge counter at exactly EDGE_TARGET (137 in the bitstream), which anchors
 * odr_sync so the first ODR lands on CLKin edge 146 — the edge the AD7134
 * datasheet guarantees coincides with the first dig_clk rising edge.
 *
 *   1. ARM_STOP_AT_ALIGN  — FSM RUNNING→ARMED, waits for /32 negedge,
 *                            transitions to STOPPED_LOW. IRQ[1] fires.
 *   2. regmap_write(reg, val) over the independent 100 MHz SPI bus
 *      (still works with XTAL2_CLKIN stopped — that's what §B is for).
 *   3. fsleep §F.4 (10 ms settle).
 *   4. RESUME — clock restarts, edge_cnt resets to 0.
 *   5. wait IRQ[0] — fires when edge_cnt reaches EDGE_TARGET (=137);
 *      the ODR/dig_clk edge itself is ~9.5 cycles later at edge 146.
 *
 * Returns -ETIMEDOUT if either IRQ doesn't arrive within 100 ms; the chip
 * config write itself returns its own errno on SPI failure. On any error
 * we issue a fallback RESUME so the rest of the system doesn't sit with a
 * stopped XTAL forever.
 */
static int ad4134_clkin_change_power_mode(struct ad4134_state *st,
					  unsigned int reg, unsigned int val)
{
	struct device *dev = &st->spi->dev;
	unsigned long timeout;
	int ret;

	/* §F.1 ARM_STOP_AT_ALIGN — wait /32 negedge. */
	reinit_completion(&st->clkin_align_stopped);
	writel(AD4134_CLKIN_CTRL_ARM_STOP_AT_ALIGN,
	       st->clkin_base + AD4134_CLKIN_CONTROL);

	timeout = wait_for_completion_timeout(&st->clkin_align_stopped,
					      msecs_to_jiffies(100));
	if (!timeout) {
		dev_err(dev,
			"ad4134: §F.1 stop_at_align TIMEOUT (STATUS=0x%08x IRQ=0x%08x DIV32=0x%08x)\n",
			readl(st->clkin_base + AD4134_CLKIN_STATUS),
			readl(st->clkin_base + AD4134_CLKIN_IRQ_PENDING),
			readl(st->clkin_base + AD4134_CLKIN_DIV32_PHASE));
		return -ETIMEDOUT;
	}

	/*
	 * §F.3 SPI write over the independent 100 MHz bus (still works with
	 * XTAL2_CLKIN stopped — that's what §B is for).
	 */
	ret = regmap_write(st->regmap, reg, val);
	if (ret) {
		dev_err(dev, "ad4134: §F.3 SPI write FAILED: %d (issuing fallback RESUME)\n", ret);
		writel(AD4134_CLKIN_CTRL_RESUME,
		       st->clkin_base + AD4134_CLKIN_CONTROL);
		return ret;
	}

	/* §H — broadcast the same write to the slave over the shared CS. */
	if (st->ad4134_duo && st->cs_gpio) {
		gpiod_set_value_cansleep(st->cs_gpio, 1);
		ret = regmap_write(st->regmap, reg, val);
		gpiod_set_value_cansleep(st->cs_gpio, 0);
		if (ret)
			dev_warn(dev, "ad4134: §H slave write FAILED: %d\n", ret);
	}

	/* §F.4 settle. */
	fsleep(10000);

	/* §F.6 RESUME — XTAL restarts, edge_cnt=0. */
	reinit_completion(&st->clkin_edge_hit);
	writel(AD4134_CLKIN_CTRL_RESUME,
	       st->clkin_base + AD4134_CLKIN_CONTROL);

	timeout = wait_for_completion_timeout(&st->clkin_edge_hit,
					      msecs_to_jiffies(100));
	if (!timeout) {
		dev_err(dev,
			"ad4134: §F.7 edge_target_reached TIMEOUT (STATUS=0x%08x IRQ=0x%08x EDGE=0x%08x)\n",
			readl(st->clkin_base + AD4134_CLKIN_STATUS),
			readl(st->clkin_base + AD4134_CLKIN_IRQ_PENDING),
			readl(st->clkin_base + AD4134_CLKIN_EDGE_COUNT));
		return -ETIMEDOUT;
	}

	return 0;
}

/*
 * Poll DEVICE_STATUS.STAT_PLL_LOCK until the ASRC PLL locks to the ODR input.
 * Replaces a blind settle delay: the datasheet requires confirming PLL lock
 * before capture, and the lock time varies with ODR. Each chip is read over its
 * own native CS through @regmap — never the broadcast cs_gpio, whose reads draw
 * contention from both chips driving SDO simultaneously.
 */
static int ad4134_wait_pll_lock(struct device *dev, struct regmap *regmap,
				const char *who)
{
	unsigned int status;
	int ret;

	ret = regmap_read_poll_timeout(regmap, AD4134_DEVICE_STATUS_REG,
				       status, status & AD4134_STAT_PLL_LOCK,
				       AD4134_PLL_LOCK_POLL_US,
				       AD4134_PLL_LOCK_TIMEOUT_US);
	if (ret) {
		dev_err(dev,
			"ad4134: %s PLL did not lock (DEVICE_STATUS=0x%02x): %d\n",
			who, status, ret);
		return ret;
	}

	return 0;
}

/* device_for_each_child callback: poll a sibling ADC's PLL via its own regmap. */
static int ad4134_wait_sibling_pll_lock(struct device *dev, void *data)
{
	struct spi_device *self_spi = data;
	struct spi_device *sib_spi;
	struct iio_dev *indio_dev;
	struct ad4134_state *sib;

	if (!dev->driver)
		return 0;

	sib_spi = to_spi_device(dev);
	if (sib_spi == self_spi)
		return 0;

	indio_dev = dev_get_drvdata(dev);
	if (!indio_dev)
		return 0;

	sib = iio_priv(indio_dev);
	if (!sib->regmap)
		return 0;

	return ad4134_wait_pll_lock(dev, sib->regmap, "slave");
}

static int ad4134_setup(struct ad4134_state *st)
{
	struct device *dev = &st->spi->dev;
	struct gpio_desc *reset_gpio;
	struct clk *clk;
	int ret;

	clk = devm_clk_get_enabled(dev, "cnv_ext_clk");
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "Failed to find SYS clock\n");

	st->sys_clk_rate = clk_get_rate(clk);
	if (!st->sys_clk_rate)
		return dev_err_probe(dev, -EINVAL, "Failed to get SYS clock rate\n");
	st->sys_clk_hz = st->sys_clk_rate;
	dev_info(dev, "ad7134: cnv_ext_clk rate = %lu Hz, sys_clk_rate = %lu Hz\n",
		 clk_get_rate(clk), st->sys_clk_rate);

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(st->regulators),
				      st->regulators);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get regulators\n");

	ret = regulator_bulk_enable(ARRAY_SIZE(st->regulators), st->regulators);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable regulators\n");

	ret = regulator_get_voltage(st->regulators[AD4134_REFIN_REGULATOR].consumer);
	if (ret < 0)
		return ret;

	st->refin_mv = ret / 1000;

	ret = devm_add_action_or_reset(dev, ad4134_disable_regulators, st);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to add regulators disable action\n");

	reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(reset_gpio))
		return dev_err_probe(dev, PTR_ERR(reset_gpio),
				     "Failed to find reset GPIO\n");

	/* Published so the master can reach the slave's reset for §E (§H). */
	st->reset_gpio = reset_gpio;

	st->cs_gpio = devm_gpiod_get_optional(dev, "cs", GPIOD_OUT_LOW);
	if (IS_ERR(st->cs_gpio))
		return dev_err_probe(dev, PTR_ERR(st->cs_gpio),
				     "Failed to find cs-gpio\n");

	/*
	 * Sequence.txt §E/§F are one master responsibility on a dual-chip
	 * board sharing XTAL2_CLKIN. The slave keeps /RESETN asserted and lets
	 * the master drive the controlled §E for both chips simultaneously
	 * (gate OFF, both released together, both see the same 36 startup
	 * edges) and broadcast §F/config afterwards. Without this the slave
	 * would release its own reset against a free-running gate — an
	 * uncontrolled RC->crystal handover that defeats §E.
	 */
	if (!st->clkin_present && st->ad4134_duo && ad4134_master_has_clkin(st))
		return 0;

	/*
	 * Sequence.txt §E — release /RESETN with controlled XTAL handover.
	 * §E.1-6 are owned by ad4134_clkin_startup().
	 */
	if (st->clkin_present) {
		ret = ad4134_clkin_startup(st, reset_gpio);
		if (ret)
			return ret;
	}

	/*
	 * Sequence.txt §F.1-7 — power-mode change to HIGH_PERF.  Stops the
	 * XTAL at the next /32 negedge, does the SPI write while the clock
	 * is stopped (SPI runs on its independent 100 MHz clock per §B),
	 * waits 10 ms, resumes, and blocks until the edge_target IRQ fires
	 * at edge 137 — which anchors the first ODR to CLKin edge 146, the
	 * AD7134's guaranteed first dig_clk rising edge.
	 *
	 * Value 0x01 = AD4134_POWER_MODE_HIGH_PERF. All other DEVICE_CONFIG
	 * bits default to 0 after reset.
	 */
	if (st->clkin_present) {
		ret = ad4134_clkin_change_power_mode(st,
						     AD4134_DEVICE_CONFIG_REG,
						     AD4134_POWER_MODE_HIGH_PERF);
		if (ret)
			return ret;
	}

	ret = regmap_update_bits(st->regmap, AD4134_DATA_PACKET_CONFIG_REG,
				 AD4134_DATA_PACKET_CONFIG_FRAME_MASK,
				 FIELD_PREP(AD4134_DATA_PACKET_CONFIG_FRAME_MASK,
					    st->output_frame));
	if (ret)
		return ret;

	ret = regmap_update_bits(st->regmap, AD4134_DIG_IF_CFG_REG,
				 AD4134_DIF_IF_CFG_FORMAT_MASK,
				 FIELD_PREP(AD4134_DIF_IF_CFG_FORMAT_MASK,
					    AD4134_DATA_FORMAT_QUAD_CH_PARALLEL));
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, AD4134_POWER_CONFIG_REG,
			   AD4134_POWER_CONFIG_LDO_PD);
	if (ret)
		return ret;

	ret = regmap_update_bits(st->regmap, AD4134_CHAN_DIG_FILTER_SEL_REG,
				 AD4134_CHAN_DIG_FILTER_SEL_MASK,
				 FIELD_PREP(AD4134_CHAN_DIG_FILTER_SEL_MASK,
					    AD4134_SINC6_FILTER));
	if (ret)
		return ret;

	/*
	 * Sequence.txt §F.10 — start ODR after dig_clk is established and
	 * config registers are programmed. ODR is the conversion clock the
	 * chip's PLL locks to, so it must be running stably for §F.14.
	 */
	if (device_property_present(&st->spi->dev, "pwms")) {
		ret = ad4134_pwm_get(st);
		if (ret)
			return dev_err_probe(dev, ret, "failed to get PWM\n");

		st->odr_hz = 100000;
		ret = ad4134_setup_odr(st, st->odr_hz);
		if (ret)
			return dev_err_probe(dev, ret, "failed to set ODR freq\n");

		ret = ad4134_wait_pll_lock(dev, st->regmap, "master");
		if (ret)
			return ret;
	}

	/*
	 * Dual-chip: re-apply the remaining configuration to the slave.
	 * The DEVICE_CONFIG (LP→HP) write was already sent to the slave
	 * inside ad4134_clkin_change_power_mode() during the XTAL-stopped
	 * window (§H), so both chips received RESUME simultaneously and
	 * both dig_clk outputs rose at edge 146.  The broadcast writes
	 * below cover the non-timing-critical registers (0x11, 0x12, 0x13,
	 * 0x1E); DIG_IF_RESET is not issued here (it is done later from the
	 * ad4134_sync sysfs attribute).  The 0x02 write is included as
	 * a fallback: if the §H slave write failed (SPI error), the slave
	 * is still in LP mode here and needs an HP write.  On a successful
	 * §H this becomes an HP→HP no-op.
	 */
	if (st->ad4134_duo && st->cs_gpio) {
		gpiod_set_value_cansleep(st->cs_gpio, 1);

		regmap_write(st->regmap, AD4134_DATA_PACKET_CONFIG_REG,
			     FIELD_PREP(AD4134_DATA_PACKET_CONFIG_FRAME_MASK,
					st->output_frame));
		regmap_write(st->regmap, AD4134_DIG_IF_CFG_REG,
			     FIELD_PREP(AD4134_DIF_IF_CFG_FORMAT_MASK,
					AD4134_DATA_FORMAT_QUAD_CH_PARALLEL));
		regmap_write(st->regmap, AD4134_DEVICE_CONFIG_REG,
			     AD4134_POWER_MODE_HIGH_PERF);
		regmap_write(st->regmap, AD4134_POWER_CONFIG_REG,
			     AD4134_POWER_CONFIG_LDO_PD);
		regmap_write(st->regmap, AD4134_CHAN_DIG_FILTER_SEL_REG,
			     FIELD_PREP(AD4134_CHAN_DIG_FILTER_SEL_MASK,
					AD4134_SINC6_FILTER));

		gpiod_set_value_cansleep(st->cs_gpio, 0);

		ret = device_for_each_child(&st->spi->controller->dev, st->spi,
					    ad4134_wait_sibling_pll_lock);
		if (ret)
			return ret;
	}

	return 0;
}

static bool ad4134_regmap_log(struct device *dev, unsigned int reg,
			      const char *op)
{
	dev_info(dev, "ad7134: SPI %s reg 0x%02x%s\n", op, reg,
		 atomic_read(&ad4134_capture_active) ?
		 "   <<<<< DURING CAPTURE" : "");

	return true;
}

static bool ad4134_regmap_writeable(struct device *dev, unsigned int reg)
{
	return ad4134_regmap_log(dev, reg, "WRITE");
}

static bool ad4134_regmap_readable(struct device *dev, unsigned int reg)
{
	return ad4134_regmap_log(dev, reg, "read ");
}

static const struct regmap_config ad4134_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.writeable_reg = ad4134_regmap_writeable,
	.readable_reg = ad4134_regmap_readable,
};

static inline int ad4134_spi_engine_compare_fwnode(struct device *dev, void *data)
{
	struct fwnode_handle *fwnode = data;

	return device_match_fwnode(dev, fwnode);
}

static inline void ad4134_spi_engine_release_fwnode(struct device *dev, void *data)
{
	struct fwnode_handle *fwnode = data;

	fwnode_handle_put(fwnode);
}

static int ad4134_bind(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ad4134_state *st = iio_priv(indio_dev);
	int ret;

	ret = component_bind_all(dev, st);
	if (ret)
		return ret;

	ret = ad4134_offload_buffer_setup(indio_dev, st->spi);
	if (ret)
		return ret;

	ret = ad4134_update_conversion_rate(st, st->odr_hz);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set sampling freq\n");

	return iio_device_register(indio_dev);
}

static void ad4134_unbind(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);

	iio_device_unregister(indio_dev);

	component_unbind_all(dev, NULL);
}

static const struct component_master_ops ad4134_comp_ops = {
	.bind = ad4134_bind,
	.unbind = ad4134_unbind,
};

static int ad4134_probe(struct spi_device *spi)
{
	struct component_match *match = NULL;
	struct device *dev = &spi->dev;
	struct fwnode_handle *fwnode = dev_fwnode(dev);
	struct iio_dev *indio_dev;
	struct ad4134_state *st;
	bool ad4134_duo;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	mutex_init(&st->lock);
	st->spi = spi;

	dev_set_drvdata(dev, indio_dev);

	st->regulators[AD4134_AVDD5_REGULATOR].supply = "avdd5";
	st->regulators[AD4134_AVDD1V8_REGULATOR].supply = "avdd1v8";
	st->regulators[AD4134_IOVDD_REGULATOR].supply = "iovdd";
	st->regulators[AD4134_REFIN_REGULATOR].supply = "refin";

	ad4134_duo = ad4134_get_ADC_count(st) == 2;
	st->ad4134_duo = ad4134_duo;

	/*
	 * Dual-chip master (the node carrying the clkin_aligner phandle) drives
	 * the shared §E for both chips, so it needs the slave's reset GPIO.
	 * Defer until the slave has probed far enough to publish it — this makes
	 * the ordering deterministic instead of relying on natural probe order.
	 */
	if (ad4134_duo && device_property_present(dev, "adi,clkin-aligner") &&
	    !ad4134_get_slave_reset(st))
		return dev_err_probe(dev, -EPROBE_DEFER,
				     "waiting for sibling ADC to publish reset GPIO\n");

	st->output_frame = AD4134_DATA_PACKET_24BIT_FRAME;
	ret = device_property_match_property_string(dev, "adi,adc-frame",
						    ad4134_frame_config,
						    ARRAY_SIZE(ad4134_frame_config));
	if (ret < 0)
		dev_warn(dev, "Failed to get adi,adc-frame property: %d\n", ret);
	else
		st->output_frame = ret;

	switch (st->output_frame) {
	case AD4134_DATA_PACKET_16BIT_FRAME:
		if (ad4134_duo) {
			indio_dev->channels = ad4134_16_duo_chan_set;
			indio_dev->num_channels = ARRAY_SIZE(ad4134_16_duo_chan_set);
			indio_dev->available_scan_masks = ad4134_duo_channel_masks;
		} else {
			indio_dev->channels = ad4134_16_chan_set;
			indio_dev->num_channels = ARRAY_SIZE(ad4134_16_chan_set);
			indio_dev->available_scan_masks = ad4134_channel_masks;
		}
		break;
	case AD4134_DATA_PACKET_16BIT_CRC6_FRAME:
		if (ad4134_duo) {
			indio_dev->channels = ad4134_16CRC_duo_chan_set;
			indio_dev->num_channels = ARRAY_SIZE(ad4134_16CRC_duo_chan_set);
			indio_dev->available_scan_masks = ad4134_duo_channel_masks;
		} else {
			indio_dev->channels = ad4134_16CRC_chan_set;
			indio_dev->num_channels = ARRAY_SIZE(ad4134_16CRC_chan_set);
			indio_dev->available_scan_masks = ad4134_channel_masks;
		}
		break;
	case AD4134_DATA_PACKET_24BIT_FRAME:
		if (ad4134_duo) {
			indio_dev->channels = ad4134_24_duo_chan_set;
			indio_dev->num_channels = ARRAY_SIZE(ad4134_24_duo_chan_set);
			indio_dev->available_scan_masks = ad4134_duo_channel_masks;
		} else {
			indio_dev->channels = ad4134_24_chan_set;
			indio_dev->num_channels = ARRAY_SIZE(ad4134_24_chan_set);
			indio_dev->available_scan_masks = ad4134_channel_masks;
		}
		break;
	case AD4134_DATA_PACKET_24BIT_CRC6_FRAME:
		if (ad4134_duo) {
			indio_dev->channels = ad4134_24CRC_duo_chan_set;
			indio_dev->num_channels = ARRAY_SIZE(ad4134_24CRC_duo_chan_set);
			indio_dev->available_scan_masks = ad4134_duo_channel_masks;
		} else {
			indio_dev->channels = ad4134_24CRC_chan_set;
			indio_dev->num_channels = ARRAY_SIZE(ad4134_24CRC_chan_set);
			indio_dev->available_scan_masks = ad4134_channel_masks;
		}
		break;
	default:
		return dev_err_probe(dev, -EINVAL,
				     "Failed to config ADC frame\n");
	}

	st->regmap = devm_regmap_init_spi(spi, &ad4134_regmap_config);
	if (IS_ERR(st->regmap))
		return PTR_ERR(st->regmap);

	/* The HDL is hardconded/configured to read from all 4 DOUT lines. */
	st->num_dout_lines = 4;

	/*
	 * Map the clkin_aligner side-band IP (if present in DT/bitstream).
	 * Must run before ad4134_setup() so §E/§F helpers can be used there.
	 * Returns 0 with clkin_present=false when the phandle is absent —
	 * the legacy code path in ad4134_setup() then runs unchanged.
	 */
	ret = ad4134_clkin_init(st);
	if (ret)
		return ret;

	ret = ad4134_setup(st);
	if (ret)
		return ret;

	if (device_property_present(&st->spi->dev, "gpio-controller")) {
		ret = ad4134_gpio_setup(st);
		if (ret < 0)
			return dev_err_probe(&spi->dev, ret,
					     "Failed to setup GPIOs\n");
	}

	indio_dev->name = spi->dev.of_node->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ad4134_info;

	if (!device_property_present(&st->spi->dev, "adi,spi-engine")) {
		indio_dev->channels = 0;
		indio_dev->num_channels = 0;
		indio_dev->available_scan_masks = 0;
		return devm_iio_device_register(dev, indio_dev);
	}

	indio_dev->setup_ops = &ad4134_offload_buffer_setup_ops;

	st->spi_engine_fwnode = fwnode_find_reference(fwnode, "adi,spi-engine", 0);
	if (IS_ERR(st->spi_engine_fwnode))
		return dev_err_probe(dev, PTR_ERR(st->spi_engine_fwnode),
				     "Failed to find SPI engine node\n");

	component_match_add_release(dev, &match, ad4134_spi_engine_release_fwnode,
				    ad4134_spi_engine_compare_fwnode,
				    st->spi_engine_fwnode);

	return component_master_add_with_match(dev, &ad4134_comp_ops, match);
}

static void ad4134_remove(struct spi_device *spi)
{
	component_master_del(&spi->dev, &ad4134_comp_ops);
}

static const struct spi_device_id ad4134_id[] = {
	{ "ad4134", 0 },
	{ },
};
MODULE_DEVICE_TABLE(spi, ad4134_id);

static const struct of_device_id ad4134_of_match[] = {
	{
		.compatible = "adi,ad4134",
	},
	{ }
};
MODULE_DEVICE_TABLE(of, ad4134_of_match);

static struct spi_driver ad4134_driver = {
	.driver = {
		.name = AD4134_NAME,
		.of_match_table = ad4134_of_match,
	},
	.probe = ad4134_probe,
	.remove = ad4134_remove,
	.id_table = ad4134_id,
};

static int ad4134_spi_engine_bind(struct device *dev, struct device *master,
				  void *data)
{
	struct ad4134_state *st = data;

	st->spi_engine = to_spi_device(dev);

	return 0;
}

static const struct component_ops ad4134_spi_engine_ops = {
	.bind   = ad4134_spi_engine_bind,
};

static int ad4134_spi_engine_probe(struct spi_device *spi)
{
	return component_add(&spi->dev, &ad4134_spi_engine_ops);
}

static void ad4134_spi_engine_remove(struct spi_device *spi)
{
	component_del(&spi->dev, &ad4134_spi_engine_ops);
}

static const struct spi_device_id ad4134_spi_engine_id[] = {
	{ "ad4134-spi-engine", 0 },
	{ },
};
MODULE_DEVICE_TABLE(spi, ad4134_spi_engine_id);

static const struct of_device_id ad4134_spi_engine_of_match[] = {
	{
		.compatible = "adi,ad4134-spi-engine",
	},
	{ }
};
MODULE_DEVICE_TABLE(of, ad4134_spi_engine_of_match);

static struct spi_driver ad4134_spi_engine_driver = {
	.driver = {
		.name = "ad4134-spi-engine",
		.of_match_table = ad4134_spi_engine_of_match,
	},
	.probe = ad4134_spi_engine_probe,
	.remove = ad4134_spi_engine_remove,
	.id_table = ad4134_spi_engine_id,
};

static int __init ad4134_init(void)
{
	int ret;

	ret = spi_register_driver(&ad4134_driver);
	if (ret)
		return ret;

	ret = spi_register_driver(&ad4134_spi_engine_driver);
	if (ret) {
		spi_unregister_driver(&ad4134_driver);
		return ret;
	}

	return 0;
}
module_init(ad4134_init);

static void __exit ad4134_exit(void)
{
	spi_unregister_driver(&ad4134_spi_engine_driver);
	spi_unregister_driver(&ad4134_driver);
}
module_exit(ad4134_exit);

MODULE_AUTHOR("Cosmin Tanislav <cosmin.tanislav@analog.com>");
MODULE_AUTHOR("Marcelo Schmitt <marcelo.schmitt@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD4134 SPI driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(IIO_DMAENGINE_BUFFER);
