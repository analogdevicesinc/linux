// SPDX-License-Identifier: GPL-2.0-only
/**
 * ADE9000 driver
 *
 * Copyright 2025 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/events.h>
#include <linux/iio/sysfs.h>
#include <linux/interrupt.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>

/* Address of ADE9000 registers */
#define ADE9000_REG_AIGAIN		0x000
#define ADE9000_REG_AVGAIN		0x00B
#define ADE9000_REG_AIRMSOS		0x00C
#define ADE9000_REG_AVRMSOS		0x00D
#define ADE9000_REG_APGAIN		0x00E
#define ADE9000_REG_AWATTOS		0x00F
#define ADE9000_REG_AVAROS		0x010
#define ADE9000_REG_AFVAROS		0x012
#define ADE9000_REG_CONFIG0		0x060
#define ADE9000_REG_DICOEFF		0x072
#define ADE9000_REG_AI_PCF		0x20A
#define ADE9000_REG_AV_PCF		0x20B
#define ADE9000_REG_AIRMS		0x20C
#define ADE9000_REG_AVRMS		0x20D
#define ADE9000_REG_AWATT		0x210
#define ADE9000_REG_AVAR		0x211
#define ADE9000_REG_AVA			0x212
#define ADE9000_REG_AFVAR		0x214
#define ADE9000_REG_APF			0x216
#define ADE9000_REG_BI_PCF		0x22A
#define ADE9000_REG_BV_PCF		0x22B
#define ADE9000_REG_BIRMS		0x22C
#define ADE9000_REG_BVRMS		0x22D
#define ADE9000_REG_CI_PCF		0x24A
#define ADE9000_REG_CV_PCF		0x24B
#define ADE9000_REG_CIRMS		0x24C
#define ADE9000_REG_CVRMS		0x24D
#define ADE9000_REG_AWATT_ACC		0x2E5
#define ADE9000_REG_AWATTHR_LO		0x2E6
#define ADE9000_REG_AVAHR_LO		0x2FA
#define ADE9000_REG_AFVARHR_LO		0x30E
#define ADE9000_REG_BWATTHR_LO		0x322
#define ADE9000_REG_BVAHR_LO		0x336
#define ADE9000_REG_BFVARHR_LO		0x34A
#define ADE9000_REG_CWATTHR_LO		0x35E
#define ADE9000_REG_CVAHR_LO		0x372
#define ADE9000_REG_CFVARHR_LO		0x386
#define ADE9000_REG_STATUS0		0x402
#define ADE9000_REG_STATUS1		0x403
#define ADE9000_REG_MASK0		0x405
#define ADE9000_REG_MASK1		0x406
#define ADE9000_REG_EVENT_MASK		0x407
#define ADE9000_REG_VLEVEL		0x40F
#define ADE9000_REG_DIP_LVL		0x410
#define ADE9000_REG_DIPA		0x411
#define ADE9000_REG_DIPB		0x412
#define ADE9000_REG_DIPC		0x413
#define ADE9000_REG_SWELL_LVL		0x414
#define ADE9000_REG_SWELLA		0x415
#define ADE9000_REG_SWELLB		0x416
#define ADE9000_REG_SWELLC		0x417
#define ADE9000_REG_APERIOD		0x418
#define ADE9000_REG_BPERIOD		0x419
#define ADE9000_REG_CPERIOD		0x41A
#define ADE9000_REG_RUN			0x480
#define ADE9000_REG_CONFIG1		0x481
#define ADE9000_REG_ACCMODE		0x492
#define ADE9000_REG_CONFIG3		0x493
#define ADE9000_REG_ZXTOUT		0x498
#define ADE9000_REG_ZX_LP_SEL		0x49A
#define ADE9000_REG_WFB_CFG		0x4A0
#define ADE9000_REG_WFB_PG_IRQEN	0x4A1
#define ADE9000_REG_WFB_TRG_CFG		0x4A2
#define ADE9000_REG_WFB_TRG_STAT	0x4A3
#define ADE9000_REG_CONFIG2		0x4AF
#define ADE9000_REG_EP_CFG		0x4B0
#define ADE9000_REG_EGY_TIME		0x4B2
#define ADE9000_REG_PGA_GAIN		0x4B9
#define ADE9000_REG_VERSION		0x4FE
#define ADE9000_REG_WF_BUFF		0x800
#define ADE9000_REG_WF_HALF_BUFF	0xC00

#define ADE9000_REG_ADDR_MASK		GENMASK(15, 4)
#define ADE9000_REG_READ_BIT_MASK	BIT(3)
#define ADE9000_RX_DEPTH		6
#define ADE9000_TX_DEPTH		10

#define ADE9000_WF_CAP_EN_MASK		BIT(4)
#define ADE9000_WF_CAP_SEL_MASK		BIT(5)
#define ADE9000_WF_MODE_MASK		GENMASK(7, 6)
#define ADE9000_WF_SRC_MASK		GENMASK(9, 8)
#define ADE9000_WF_IN_EN_MASK		BIT(12)

/* External reference selection bit in CONFIG1 */
#define ADE9000_EXT_REF_MASK		BIT(15)

/*
 * Configuration registers
 */
#define ADE9000_PGA_GAIN		0x0000

/* Default configuration */

#define ADE9000_CONFIG0			0x00000000

/* CF3/ZX pin outputs Zero crossing, CF4 = DREADY */
#define ADE9000_CONFIG1			0x000E

/* Default High pass corner frequency of 1.25Hz */
#define ADE9000_CONFIG2			0x0A00

/* Peak and overcurrent detection disabled */
#define ADE9000_CONFIG3			0x0000

/*
 * 50Hz operation, 3P4W Wye configuration, signed accumulation
 * Clear bit 8 i.e. ACCMODE=0x00xx for 50Hz operation
 * ACCMODE=0x0x9x for 3Wire delta when phase B is used as reference
 */
#define ADE9000_ACCMODE			0x0000
#define ADE9000_ACCMODE_60HZ		0x0100

/*Line period and zero crossing obtained from VA */
#define ADE9000_ZX_LP_SEL		0x0000

/* Interrupt mask values for initialization */
#define ADE9000_MASK0_EGYRDY_INT_EN	BIT(0)
#define ADE9000_MASK1_ALL_INT_DIS	0x00000000

/* Events disabled */
#define ADE9000_EVENT_DISABLE		0x00000000

/*
 * Assuming Vnom=1/2 of full scale.
 * Refer to Technical reference manual for detailed calculations.
 */
#define ADE9000_VLEVEL			0x0022EA28

/* Set DICOEFF= 0xFFFFE000 when integrator is enabled */
#define ADE9000_DICOEFF			0x00000000

/* DSP ON */
#define ADE9000_RUN_ON			0xFFFFFFFF

/*
 * Energy Accumulation Settings
 * Enable energy accumulation, accumulate samples at 8ksps
 * latch energy accumulation after EGYRDY
 * If accumulation is changed to half line cycle mode, change EGY_TIME
 */
#define ADE9000_EP_CFG			0x0011

/* Accumulate 4000 samples */
#define ADE9000_EGY_TIME		7999

/*
 * Constant Definitions
 * ADE9000 FDSP: 8000sps, ADE9000 FDSP: 4000sps
 */
#define ADE9000_FDSP			4000
#define ADE9000_DEFAULT_CLK_FREQ_HZ	24576000
#define ADE9000_WFB_CFG			0x03E9
#define ADE9000_WFB_PAGE_SIZE		128
#define ADE9000_WFB_NR_OF_PAGES		16
#define ADE9000_WFB_MAX_CHANNELS	8
#define ADE9000_WFB_BYTES_IN_SAMPLE	4
#define ADE9000_WFB_SAMPLES_IN_PAGE	\
	(ADE9000_WFB_PAGE_SIZE / ADE9000_WFB_MAX_CHANNELS)
#define ADE9000_WFB_MAX_SAMPLES_CHAN	\
	(ADE9000_WFB_SAMPLES_IN_PAGE * ADE9000_WFB_NR_OF_PAGES)
#define ADE9000_WFB_FULL_BUFF_NR_SAMPLES \
	(ADE9000_WFB_PAGE_SIZE * ADE9000_WFB_NR_OF_PAGES)
#define ADE9000_WFB_FULL_BUFF_SIZE	\
	(ADE9000_WFB_FULL_BUFF_NR_SAMPLES * ADE9000_WFB_BYTES_IN_SAMPLE)

#define ADE9000_SWRST_BIT		BIT(0)

/* Status and Mask register bits*/
#define ADE9000_ST0_WFB_TRIG_BIT	BIT(16)
#define ADE9000_ST0_PAGE_FULL_BIT	BIT(17)
#define ADE9000_ST0_EGYRDY		BIT(0)

#define ADE9000_ST1_ZXTOVA_BIT		BIT(6)
#define ADE9000_ST1_ZXTOVB_BIT		BIT(7)
#define ADE9000_ST1_ZXTOVC_BIT		BIT(8)
#define ADE9000_ST1_ZXVA_BIT		BIT(9)
#define ADE9000_ST1_ZXVB_BIT		BIT(10)
#define ADE9000_ST1_ZXVC_BIT		BIT(11)
#define ADE9000_ST1_ZXIA_BIT		BIT(13)
#define ADE9000_ST1_ZXIB_BIT		BIT(14)
#define ADE9000_ST1_ZXIC_BIT		BIT(15)
#define ADE9000_ST1_RSTDONE_BIT		BIT(16)
#define ADE9000_ST1_SEQERR_BIT		BIT(18)
#define ADE9000_ST1_SWELLA_BIT		BIT(20)
#define ADE9000_ST1_SWELLB_BIT		BIT(21)
#define ADE9000_ST1_SWELLC_BIT		BIT(22)
#define ADE9000_ST1_DIPA_BIT		BIT(23)
#define ADE9000_ST1_DIPB_BIT		BIT(24)
#define ADE9000_ST1_DIPC_BIT		BIT(25)
#define ADE9000_ST1_ERROR0_BIT		BIT(28)
#define ADE9000_ST1_ERROR1_BIT		BIT(29)
#define ADE9000_ST1_ERROR2_BIT		BIT(30)
#define ADE9000_ST1_ERROR3_BIT		BIT(31)
#define ADE9000_ST_ERROR \
	(ADE9000_ST1_ERROR0 | ADE9000_ST1_ERROR1 | \
	 ADE9000_ST1_ERROR2 | ADE9000_ST1_ERROR3)
#define ADE9000_ST1_CROSSING_FIRST	6
#define ADE9000_ST1_CROSSING_DEPTH	25

#define ADE9000_WFB_TRG_DIP_BIT		BIT(0)
#define ADE9000_WFB_TRG_SWELL_BIT	BIT(1)
#define ADE9000_WFB_TRG_ZXIA_BIT	BIT(3)
#define ADE9000_WFB_TRG_ZXIB_BIT	BIT(4)
#define ADE9000_WFB_TRG_ZXIC_BIT	BIT(5)
#define ADE9000_WFB_TRG_ZXVA_BIT	BIT(6)
#define ADE9000_WFB_TRG_ZXVB_BIT	BIT(7)
#define ADE9000_WFB_TRG_ZXVC_BIT	BIT(8)

/* Stop when waveform buffer is full */
#define ADE9000_WFB_FULL_MODE		0x0
/* Continuous fill—stop only on enabled trigger events */
#define ADE9000_WFB_EN_TRIG_MODE	0x1
/* Continuous filling—center capture around enabled trigger events */
#define ADE9000_WFB_C_EN_TRIG_MODE	0x2
/* Continuous fill—used as streaming mode for continuous data output */
#define ADE9000_WFB_STREAMING_MODE	0x3

#define ADE9000_LAST_PAGE_BIT		BIT(15)
#define ADE9000_MIDDLE_PAGE_BIT		BIT(7)

/*
 * Full scale Codes referred from Datasheet. Respective digital codes are
 * produced when ADC inputs are at full scale.
 */
#define ADE9000_RMS_FULL_SCALE_CODES	52866837
#define ADE9000_WATT_FULL_SCALE_CODES	20694066
#define ADE9000_PCF_FULL_SCALE_CODES	74770000

/* Phase and channel definitions */
#define ADE9000_PHASE_A_NR		0
#define ADE9000_PHASE_B_NR		1
#define ADE9000_PHASE_C_NR		2

#define ADE9000_SCAN_POS_IA		BIT(0)
#define ADE9000_SCAN_POS_VA		BIT(1)
#define ADE9000_SCAN_POS_IB		BIT(2)
#define ADE9000_SCAN_POS_VB		BIT(3)
#define ADE9000_SCAN_POS_IC		BIT(4)
#define ADE9000_SCAN_POS_VC		BIT(5)

/* Waveform buffer configuration values */
enum ade9000_wfb_cfg {
	ADE9000_WFB_CFG_ALL_CHAN = 0x0,
	ADE9000_WFB_CFG_IA_VA = 0x1,
	ADE9000_WFB_CFG_IB_VB = 0x2,
	ADE9000_WFB_CFG_IC_VC = 0x3,
	ADE9000_WFB_CFG_IA = 0x8,
	ADE9000_WFB_CFG_VA = 0x9,
	ADE9000_WFB_CFG_IB = 0xA,
	ADE9000_WFB_CFG_VB = 0xB,
	ADE9000_WFB_CFG_IC = 0xC,
	ADE9000_WFB_CFG_VC = 0xD,
};

#define ADE9000_PHASE_B_POS_BIT		BIT(5)
#define ADE9000_PHASE_C_POS_BIT		BIT(6)

#define ADE9000_MAX_PHASE_NR		3
#define AD9000_CHANNELS_PER_PHASE	10

#define ADE9000_ADDR_ADJUST(addr, chan)					\
	(((chan) == 0 ? 0 : (chan) == 1 ? 2 : 4) << 4 | (addr))

struct ade9000_state {
	struct completion reset_completion;
	struct mutex lock; /* Protects SPI transactions */
	u8 wf_src;
	u32 wfb_trg;
	u8 wfb_nr_activ_chan;
	u32 wfb_nr_samples;
	struct spi_device *spi;
	struct clk *clkin;
	struct clk_hw clkout_hw;
	struct spi_transfer xfer[2];
	struct spi_message spi_msg;
	struct regmap *regmap;
	union{
		u8 byte[ADE9000_WFB_FULL_BUFF_SIZE];
		__be32 word[ADE9000_WFB_FULL_BUFF_NR_SAMPLES];
	} rx_buff __aligned(IIO_DMA_MINALIGN);
	u8 tx_buff[2] __aligned(IIO_DMA_MINALIGN);
	u8 tx[ADE9000_TX_DEPTH] __aligned(IIO_DMA_MINALIGN);
	u8 rx[ADE9000_RX_DEPTH];
	unsigned int bulk_read_buf[2];
};

static unsigned long ade9000_clkout_recalc_rate(struct clk_hw *hw,
						unsigned long parent_rate)
{
	/* CLKOUT provides the same frequency as the crystal/external clock */
	return parent_rate ? parent_rate : ADE9000_DEFAULT_CLK_FREQ_HZ;
}

static const struct clk_ops ade9000_clkout_ops = {
	.recalc_rate = ade9000_clkout_recalc_rate,
};

static const struct iio_event_spec ade9000_events[] = {
	{
		/* Energy ready event - datasheet: EGYRDY interrupt */
		.type = IIO_EV_TYPE_MAG,
		.dir = IIO_EV_DIR_NONE, /* Non-directional event */
		.mask_shared_by_all = BIT(IIO_EV_INFO_ENABLE),
	},
	{
		/* Sequence error event - datasheet: SEQERR interrupt */
		.type = IIO_EV_TYPE_CHANGE,
		.dir = IIO_EV_DIR_NONE, /* Non-directional event */
		.mask_shared_by_all = BIT(IIO_EV_INFO_ENABLE),
	},
	{
		/* Threshold events - datasheet: zero crossing timeout, sag/swell */
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_NONE, /* Timeout events (ZXTOUT register) */
		.mask_separate = BIT(IIO_EV_INFO_ENABLE), /* Per-channel enable */
		.mask_shared_by_all = BIT(IIO_EV_INFO_VALUE), /* Shared threshold value */
	},
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_EITHER,
		.mask_separate = BIT(IIO_EV_INFO_ENABLE),
	},
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING, /* for swell */
		.mask_separate = BIT(IIO_EV_INFO_ENABLE),
		.mask_shared_by_all = BIT(IIO_EV_INFO_VALUE),
	},
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING, /* for dip */
		.mask_separate = BIT(IIO_EV_INFO_ENABLE),
		.mask_shared_by_all = BIT(IIO_EV_INFO_VALUE),
	},
};

static const char * const ade9000_filter_type_items[] = {
	"sinc4", "sinc4+lp",
};

static const int ade9000_filter_type_values[] = {
	0, 2,
};

static int ade9000_filter_type_get(struct iio_dev *indio_dev,
				   const struct iio_chan_spec *chan)
{
	struct ade9000_state *st = iio_priv(indio_dev);
	u32 val;
	int ret, i;

	ret = regmap_read(st->regmap, ADE9000_REG_WFB_CFG, &val);
	if (ret)
		return ret;

	val = FIELD_GET(ADE9000_WF_SRC_MASK, val);

	for (i = 0; i < ARRAY_SIZE(ade9000_filter_type_values); i++) {
		if (ade9000_filter_type_values[i] == val)
			return i;
	}

	return -EINVAL;
}

static int ade9000_filter_type_set(struct iio_dev *indio_dev,
				   const struct iio_chan_spec *chan,
				   unsigned int index)
{
	struct ade9000_state *st = iio_priv(indio_dev);
	int ret, val;

	if (index >= ARRAY_SIZE(ade9000_filter_type_values))
		return -EINVAL;

	val = ade9000_filter_type_values[index];

	/* Update the WFB_CFG register with the new filter type */
	ret = regmap_update_bits(st->regmap, ADE9000_REG_WFB_CFG,
				 ADE9000_WF_SRC_MASK,
				 FIELD_PREP(ADE9000_WF_SRC_MASK, val));
	if (ret)
		return ret;

	/* Update cached value */
	st->wf_src = val;

	return 0;
}

static const struct iio_enum ade9000_filter_type_enum = {
	.items = ade9000_filter_type_items,
	.num_items = ARRAY_SIZE(ade9000_filter_type_items),
	.get = ade9000_filter_type_get,
	.set = ade9000_filter_type_set,
};

static const struct iio_chan_spec_ext_info ade9000_ext_info[] = {
	IIO_ENUM("filter_type", IIO_SHARED_BY_ALL, &ade9000_filter_type_enum),
	IIO_ENUM_AVAILABLE("filter_type", IIO_SHARED_BY_ALL, &ade9000_filter_type_enum),
	{}
};

#define ADE9000_CURRENT_CHANNEL(num) {					\
	.type = IIO_CURRENT,						\
	.channel = num,							\
	.address = ADE9000_ADDR_ADJUST(ADE9000_REG_AI_PCF, num),	\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_SCALE) |		\
			      BIT(IIO_CHAN_INFO_CALIBSCALE),		\
	.event_spec = &ade9000_events[0],				\
	.num_event_specs = 1,						\
	.scan_index = num,						\
	.indexed = 1,							\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = 32,						\
		.storagebits = 32,					\
		.endianness = IIO_BE,					\
	},								\
}

#define ADE9000_VOLTAGE_CHANNEL(num) {					\
	.type = IIO_VOLTAGE,						\
	.channel = num,							\
	.address = ADE9000_ADDR_ADJUST(ADE9000_REG_AV_PCF, num),	\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_SCALE) |		\
			      BIT(IIO_CHAN_INFO_CALIBSCALE) |		\
			      BIT(IIO_CHAN_INFO_CALIBSCALE) |		\
			      BIT(IIO_CHAN_INFO_FREQUENCY),		\
	.event_spec = ade9000_events,					\
	.num_event_specs = ARRAY_SIZE(ade9000_events),			\
	.scan_index = num + 1,	/* interleave with current channels */	\
	.indexed = 1,							\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = 32,						\
		.storagebits = 32,					\
		.endianness = IIO_BE,					\
	},								\
	.ext_info = ade9000_ext_info,					\
}

#define ADE9000_ALTCURRENT_RMS_CHANNEL(num) {				\
	.type = IIO_ALTCURRENT,						\
	.channel = num,							\
	.address = ADE9000_ADDR_ADJUST(ADE9000_REG_AIRMS, num),		\
	.channel2 = IIO_MOD_RMS,					\
	.modified = 1,							\
	.indexed = 1,							\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
	.scan_index = -1						\
}

#define ADE9000_ALTVOLTAGE_RMS_CHANNEL(num) {				\
	.type = IIO_ALTVOLTAGE,						\
	.channel = num,							\
	.address = ADE9000_ADDR_ADJUST(ADE9000_REG_AVRMS, num),		\
	.channel2 = IIO_MOD_RMS,					\
	.modified = 1,							\
	.indexed = 1,							\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_SCALE) |		\
			      BIT(IIO_CHAN_INFO_OFFSET),		\
	.scan_index = -1						\
}

#define ADE9000_POWER_ACTIVE_CHANNEL(num) {				\
	.type = IIO_POWER,						\
	.channel = num,							\
	.address = ADE9000_ADDR_ADJUST(ADE9000_REG_AWATT, num),		\
	.channel2 = IIO_MOD_ACTIVE,					\
	.modified = 1,							\
	.indexed = 1,							\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_SCALE) |		\
			      BIT(IIO_CHAN_INFO_CALIBBIAS) |		\
			      BIT(IIO_CHAN_INFO_CALIBSCALE) |		\
			      BIT(IIO_CHAN_INFO_POWERFACTOR),		\
	.scan_index = -1						\
}

#define ADE9000_POWER_REACTIVE_CHANNEL(num) {				\
	.type = IIO_POWER,						\
	.channel = num,							\
	.address = ADE9000_ADDR_ADJUST(ADE9000_REG_AVAR, num),		\
	.channel2 = IIO_MOD_REACTIVE,					\
	.modified = 1,							\
	.indexed = 1,							\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_SCALE) |		\
			      BIT(IIO_CHAN_INFO_CALIBBIAS),		\
	.scan_index = -1						\
}

#define ADE9000_POWER_APPARENT_CHANNEL(num) {				\
	.type = IIO_POWER,						\
	.channel = num,							\
	.address = ADE9000_ADDR_ADJUST(ADE9000_REG_AVA, num),		\
	.channel2 = IIO_MOD_APPARENT,					\
	.modified = 1,							\
	.indexed = 1,							\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_SCALE),			\
	.scan_index = -1						\
}

 #define ADE9000_ENERGY_ACTIVE_CHANNEL(num, addr) {			\
	.type = IIO_ENERGY,						\
	.channel = num,							\
	.address = addr,						\
	.channel2 = IIO_MOD_ACTIVE,					\
	.modified = 1,							\
	.indexed = 1,							\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
	.scan_index = -1						\
}

#define ADE9000_ENERGY_APPARENT_CHANNEL(num, addr) {			\
	.type = IIO_ENERGY,						\
	.channel = num,							\
	.address = addr,						\
	.channel2 = IIO_MOD_APPARENT,					\
	.modified = 1,							\
	.indexed = 1,							\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
	.scan_index = -1						\
}

#define ADE9000_ENERGY_REACTIVE_CHANNEL(num, addr) {			\
	.type = IIO_ENERGY,						\
	.channel = num,							\
	.address = addr,						\
	.channel2 = IIO_MOD_REACTIVE,					\
	.modified = 1,							\
	.indexed = 1,							\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
	.scan_index = -1						\
}

static const struct iio_chan_spec ade9000_channels[] = {
	/* Phase A channels */
	ADE9000_CURRENT_CHANNEL(ADE9000_PHASE_A_NR),
	ADE9000_VOLTAGE_CHANNEL(ADE9000_PHASE_A_NR),
	ADE9000_ALTCURRENT_RMS_CHANNEL(ADE9000_PHASE_A_NR),
	ADE9000_ALTVOLTAGE_RMS_CHANNEL(ADE9000_PHASE_A_NR),
	ADE9000_POWER_ACTIVE_CHANNEL(ADE9000_PHASE_A_NR),
	ADE9000_POWER_REACTIVE_CHANNEL(ADE9000_PHASE_A_NR),
	ADE9000_POWER_APPARENT_CHANNEL(ADE9000_PHASE_A_NR),
	ADE9000_ENERGY_ACTIVE_CHANNEL(ADE9000_PHASE_A_NR, ADE9000_REG_AWATTHR_LO),
	ADE9000_ENERGY_APPARENT_CHANNEL(ADE9000_PHASE_A_NR, ADE9000_REG_AVAHR_LO),
	ADE9000_ENERGY_REACTIVE_CHANNEL(ADE9000_PHASE_A_NR, ADE9000_REG_AFVARHR_LO),
	/* Phase B channels */
	ADE9000_CURRENT_CHANNEL(ADE9000_PHASE_B_NR),
	ADE9000_VOLTAGE_CHANNEL(ADE9000_PHASE_B_NR),
	ADE9000_ALTCURRENT_RMS_CHANNEL(ADE9000_PHASE_B_NR),
	ADE9000_ALTVOLTAGE_RMS_CHANNEL(ADE9000_PHASE_B_NR),
	ADE9000_POWER_ACTIVE_CHANNEL(ADE9000_PHASE_B_NR),
	ADE9000_POWER_REACTIVE_CHANNEL(ADE9000_PHASE_B_NR),
	ADE9000_POWER_APPARENT_CHANNEL(ADE9000_PHASE_B_NR),
	ADE9000_ENERGY_ACTIVE_CHANNEL(ADE9000_PHASE_B_NR, ADE9000_REG_BWATTHR_LO),
	ADE9000_ENERGY_APPARENT_CHANNEL(ADE9000_PHASE_B_NR, ADE9000_REG_BVAHR_LO),
	ADE9000_ENERGY_REACTIVE_CHANNEL(ADE9000_PHASE_B_NR, ADE9000_REG_BFVARHR_LO),
	/* Phase C channels */
	ADE9000_CURRENT_CHANNEL(ADE9000_PHASE_C_NR),
	ADE9000_VOLTAGE_CHANNEL(ADE9000_PHASE_C_NR),
	ADE9000_ALTCURRENT_RMS_CHANNEL(ADE9000_PHASE_C_NR),
	ADE9000_ALTVOLTAGE_RMS_CHANNEL(ADE9000_PHASE_C_NR),
	ADE9000_POWER_ACTIVE_CHANNEL(ADE9000_PHASE_C_NR),
	ADE9000_POWER_REACTIVE_CHANNEL(ADE9000_PHASE_C_NR),
	ADE9000_POWER_APPARENT_CHANNEL(ADE9000_PHASE_C_NR),
	ADE9000_ENERGY_ACTIVE_CHANNEL(ADE9000_PHASE_C_NR, ADE9000_REG_CWATTHR_LO),
	ADE9000_ENERGY_APPARENT_CHANNEL(ADE9000_PHASE_C_NR, ADE9000_REG_CVAHR_LO),
	ADE9000_ENERGY_REACTIVE_CHANNEL(ADE9000_PHASE_C_NR, ADE9000_REG_CFVARHR_LO),
	/* System frequency (50Hz/60Hz) and PGA gain configuration channel */
	{
		.type = IIO_ALTVOLTAGE,
		.channel = 0,
		.indexed = 1,
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_FREQUENCY) |
					   BIT(IIO_CHAN_INFO_SCALE),
		.scan_index = -1
	},
};

static const struct reg_sequence ade9000_reg_sequence[] = {
	{ ADE9000_REG_PGA_GAIN, ADE9000_PGA_GAIN },
	{ ADE9000_REG_CONFIG0, ADE9000_CONFIG0 },
	{ ADE9000_REG_CONFIG1, ADE9000_CONFIG1 },
	{ ADE9000_REG_CONFIG2, ADE9000_CONFIG2 },
	{ ADE9000_REG_CONFIG3, ADE9000_CONFIG3 },
	{ ADE9000_REG_ACCMODE, ADE9000_ACCMODE },
	{ ADE9000_REG_ZX_LP_SEL, ADE9000_ZX_LP_SEL },
	{ ADE9000_REG_MASK0, ADE9000_MASK0_EGYRDY_INT_EN },
	{ ADE9000_REG_MASK1, ADE9000_MASK1_ALL_INT_DIS },
	{ ADE9000_REG_EVENT_MASK, ADE9000_EVENT_DISABLE },
	{ ADE9000_REG_WFB_CFG, ADE9000_WFB_CFG },
	{ ADE9000_REG_VLEVEL, ADE9000_VLEVEL },
	{ ADE9000_REG_DICOEFF, ADE9000_DICOEFF },
	{ ADE9000_REG_EGY_TIME, ADE9000_EGY_TIME },
	{ ADE9000_REG_EP_CFG, ADE9000_EP_CFG },
	{ ADE9000_REG_RUN, ADE9000_RUN_ON }
};

static int ade9000_spi_write_reg(void *context, unsigned int reg,
				 unsigned int val)
{
	struct ade9000_state *st = context;
	u16 addr;
	int ret, len;

	guard(mutex)(&st->lock);

	addr = FIELD_PREP(ADE9000_REG_ADDR_MASK, reg);
	put_unaligned_be16(addr, st->tx);

	if (reg > ADE9000_REG_RUN && reg < ADE9000_REG_VERSION) {
		put_unaligned_be16(val, &st->tx[2]);
		len = 4;
	} else {
		put_unaligned_be32(val, &st->tx[2]);
		len = 6;
	}

	ret = spi_write(st->spi, st->tx, len);
	if (ret)
		dev_err(&st->spi->dev, "problem when writing register 0x%x\n", reg);

	return ret;
}

static int ade9000_spi_read_reg(void *context, unsigned int reg,
				unsigned int *val)
{
	struct ade9000_state *st = context;
	u16 addr;
	int ret, rx_len;

	guard(mutex)(&st->lock);

	addr = FIELD_PREP(ADE9000_REG_ADDR_MASK, reg) |
	       ADE9000_REG_READ_BIT_MASK;

	put_unaligned_be16(addr, st->tx);

	/* Skip CRC bytes - only read actual data */
	if (reg > ADE9000_REG_RUN && reg < ADE9000_REG_VERSION)
		rx_len = 2;
	else
		rx_len = 4;

	ret = spi_write_then_read(st->spi, st->tx, 2, st->rx, rx_len);
	if (ret) {
		dev_err(&st->spi->dev, "error reading register 0x%x\n", reg);
		return ret;
	}

	if (reg > ADE9000_REG_RUN && reg < ADE9000_REG_VERSION)
		*val = get_unaligned_be16(st->rx);
	else
		*val = get_unaligned_be32(st->rx);

	return 0;
}

static bool ade9000_is_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case ADE9000_REG_STATUS0:
	case ADE9000_REG_STATUS1:
	case ADE9000_REG_MASK0:
	case ADE9000_REG_MASK1:
	case ADE9000_REG_WFB_PG_IRQEN:
		return false;
	default:
		return true;
	}
}

static int ade9000_configure_scan(struct iio_dev *indio_dev, u32 wfb_addr)
{
	struct ade9000_state *st = iio_priv(indio_dev);
	u16 addr;

	addr = FIELD_PREP(ADE9000_REG_ADDR_MASK, wfb_addr) |
	       ADE9000_REG_READ_BIT_MASK;

	put_unaligned_be16(addr, st->tx_buff);

	st->xfer[0].tx_buf = &st->tx_buff[0];
	st->xfer[0].len = 2;

	st->xfer[1].rx_buf = st->rx_buff.byte;

	/* Always use streaming mode */
	st->xfer[1].len = (st->wfb_nr_samples / 2) * 4;

	spi_message_init_with_transfers(&st->spi_msg, st->xfer, ARRAY_SIZE(st->xfer));
	return 0;
}

static int ade9000_iio_push_streaming(struct iio_dev *indio_dev)
{
	struct ade9000_state *st = iio_priv(indio_dev);
	struct device *dev = &st->spi->dev;
	u32 current_page, i;
	int ret;

	guard(mutex)(&st->lock);

	ret = spi_sync(st->spi, &st->spi_msg);
	if (ret) {
		dev_err(dev, "SPI fail in trigger handler\n");
		return ret;
	}

	/* In streaming mode, only half the buffer is filled per interrupt */
	for (i = 0; i < st->wfb_nr_samples / 2; i += st->wfb_nr_activ_chan)
		iio_push_to_buffers(indio_dev, &st->rx_buff.word[i]);

	ret = regmap_read(st->regmap, ADE9000_REG_WFB_PG_IRQEN, &current_page);
	if (ret) {
		dev_err(dev, "IRQ0 WFB read fail\n");
		return ret;
	}

	if (current_page & ADE9000_MIDDLE_PAGE_BIT) {
		ret = regmap_write(st->regmap, ADE9000_REG_WFB_PG_IRQEN,
				   ADE9000_LAST_PAGE_BIT);
		if (ret) {
			dev_err(dev, "IRQ0 WFB write fail\n");
			return ret;
		}

		ret = ade9000_configure_scan(indio_dev,
					     ADE9000_REG_WF_HALF_BUFF);
		if (ret)
			return ret;
	} else {
		ret = regmap_write(st->regmap, ADE9000_REG_WFB_PG_IRQEN,
				   ADE9000_MIDDLE_PAGE_BIT);
		if (ret) {
			dev_err(dev, "IRQ0 WFB write fail");
			return IRQ_HANDLED;
		}

		ret = ade9000_configure_scan(indio_dev,
					     ADE9000_REG_WF_BUFF);
		if (ret)
			return ret;
	}

	return 0;
}

static int ade9000_iio_push_buffer(struct iio_dev *indio_dev)
{
	struct ade9000_state *st = iio_priv(indio_dev);
	int ret;
	u32 i;

	guard(mutex)(&st->lock);

	ret = spi_sync(st->spi, &st->spi_msg);
	if (ret) {
		dev_err(&st->spi->dev, "SPI fail in trigger handler\n");
		return ret;
	}

	for (i = 0; i < st->wfb_nr_samples; i += st->wfb_nr_activ_chan)
		iio_push_to_buffers(indio_dev, &st->rx_buff.word[i]);

	return 0;
}

static irqreturn_t ade9000_irq0_thread(int irq, void *data)
{
	struct iio_dev *indio_dev = data;
	struct ade9000_state *st = iio_priv(indio_dev);
	struct device *dev = &st->spi->dev;
	s64 timestamp = iio_get_time_ns(indio_dev);
	u32 handled_irq = 0;
	u32 interrupts, status;
	int ret;

	ret = regmap_read(st->regmap, ADE9000_REG_STATUS0, &status);
	if (ret) {
		dev_err(dev, "IRQ0 read status fail\n");
		return IRQ_HANDLED;
	}

	ret = regmap_read(st->regmap, ADE9000_REG_MASK0, &interrupts);
	if (ret) {
		dev_err(dev, "IRQ0 read mask fail\n");
		return IRQ_HANDLED;
	}

	if ((status & ADE9000_ST0_PAGE_FULL_BIT) &&
	    (interrupts & ADE9000_ST0_PAGE_FULL_BIT)) {
		/* Always use streaming mode */
		ret = ade9000_iio_push_streaming(indio_dev);
		if (ret) {
			dev_err(dev, "IRQ0 IIO push fail\n");
			return IRQ_HANDLED;
		}

		handled_irq |= ADE9000_ST0_PAGE_FULL_BIT;
	}

	if ((status & ADE9000_ST0_WFB_TRIG_BIT) &&
	    (interrupts & ADE9000_ST0_WFB_TRIG_BIT)) {
		ret = regmap_update_bits(st->regmap, ADE9000_REG_WFB_CFG,
					 ADE9000_WF_CAP_EN_MASK, 0);
		if (ret) {
			dev_err(dev, "IRQ0 WFB fail\n");
			return IRQ_HANDLED;
		}

		if (iio_buffer_enabled(indio_dev)) {
			ret = ade9000_iio_push_buffer(indio_dev);
			if (ret) {
				dev_err(dev, "IRQ0 IIO push fail @ WFB TRIG\n");
				return IRQ_HANDLED;
			}
		}

		handled_irq |= ADE9000_ST0_WFB_TRIG_BIT;
	}

	if ((status & ADE9000_ST0_EGYRDY) &&
	    (interrupts & ADE9000_ST0_EGYRDY)) {
		iio_push_event(indio_dev,
			       IIO_UNMOD_EVENT_CODE(IIO_ENERGY,
						    ADE9000_ST0_EGYRDY,
						    IIO_EV_TYPE_MAG,
						    IIO_EV_DIR_NONE),
			       timestamp);

		handled_irq |= ADE9000_ST0_EGYRDY;
	}

	ret = regmap_write(st->regmap, ADE9000_REG_STATUS0, handled_irq);
	if (ret)
		dev_err(dev, "IRQ0 write status fail\n");

	return IRQ_HANDLED;
}

static irqreturn_t ade9000_irq1_thread(int irq, void *data)
{
	struct iio_dev *indio_dev = data;
	struct ade9000_state *st = iio_priv(indio_dev);
	unsigned int bit = ADE9000_ST1_CROSSING_FIRST;
	s64 timestamp = iio_get_time_ns(indio_dev);
	u32 handled_irq = 0;
	u32 interrupts;
	u32 result;
	u32 status;
	u32 tmp;
	unsigned long interrupt_bits;
	int ret;

	if (!completion_done(&st->reset_completion)) {
		ret = regmap_read(st->regmap, ADE9000_REG_STATUS1, &result);
		if (ret) {
			dev_err(&st->spi->dev, "IRQ1 read status fail\n");
			return ret;
		}

		if (result & ADE9000_ST1_RSTDONE_BIT)
			complete(&st->reset_completion);
		else
			dev_err(&st->spi->dev, "Error testing reset done\n");

		return IRQ_HANDLED;
	}

	ret = regmap_read(st->regmap, ADE9000_REG_STATUS1, &status);
	if (ret) {
		dev_err(&st->spi->dev, "IRQ1 read status fail\n");
		return IRQ_HANDLED;
	}

	ret = regmap_read(st->regmap, ADE9000_REG_MASK1, &interrupts);
	if (ret) {
		dev_err(&st->spi->dev, "IRQ1 read status fail\n");
		return IRQ_HANDLED;
	}

	interrupt_bits = interrupts;
	for_each_set_bit_from(bit, &interrupt_bits,
			      ADE9000_ST1_CROSSING_DEPTH){
		tmp = status & BIT(bit);

		switch (tmp) {
		case ADE9000_ST1_ZXVA_BIT:
			iio_push_event(indio_dev,
				       IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE,
							    ADE9000_ST1_ZXVA_BIT,
							    IIO_EV_TYPE_THRESH,
							    IIO_EV_DIR_EITHER),
				       timestamp);
			handled_irq |= ADE9000_ST1_ZXVA_BIT;
			break;
		case ADE9000_ST1_ZXTOVA_BIT:
			iio_push_event(indio_dev,
				       IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE,
							    ADE9000_ST1_ZXTOVA_BIT,
							    IIO_EV_TYPE_THRESH,
							    IIO_EV_DIR_EITHER),
				       timestamp);
			handled_irq |= ADE9000_ST1_ZXTOVA_BIT;
			break;
		case ADE9000_ST1_ZXIA_BIT:
			iio_push_event(indio_dev,
				       IIO_UNMOD_EVENT_CODE(IIO_CURRENT,
							    ADE9000_ST1_ZXIA_BIT,
							    IIO_EV_TYPE_THRESH,
							    IIO_EV_DIR_EITHER),
				       timestamp);
			handled_irq |= ADE9000_ST1_ZXIA_BIT;
			break;
		case ADE9000_ST1_ZXVB_BIT:
			iio_push_event(indio_dev,
				       IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE,
							    ADE9000_ST1_ZXVB_BIT,
							    IIO_EV_TYPE_THRESH,
							    IIO_EV_DIR_EITHER),
				       timestamp);
			handled_irq |= ADE9000_ST1_ZXVB_BIT;
			break;
		case ADE9000_ST1_ZXTOVB_BIT:
			iio_push_event(indio_dev,
				       IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE,
							    ADE9000_ST1_ZXTOVB_BIT,
							    IIO_EV_TYPE_THRESH,
							    IIO_EV_DIR_EITHER),
				       timestamp);
			handled_irq |= ADE9000_ST1_ZXTOVB_BIT;
			break;
		case ADE9000_ST1_ZXIB_BIT:
			iio_push_event(indio_dev,
				       IIO_UNMOD_EVENT_CODE(IIO_CURRENT,
							    ADE9000_ST1_ZXIB_BIT,
							    IIO_EV_TYPE_THRESH,
							    IIO_EV_DIR_EITHER),
				       timestamp);
			handled_irq |= ADE9000_ST1_ZXIB_BIT;
			break;
		case ADE9000_ST1_ZXVC_BIT:
			iio_push_event(indio_dev,
				       IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE,
							    ADE9000_ST1_ZXVC_BIT,
							    IIO_EV_TYPE_THRESH,
							    IIO_EV_DIR_EITHER),
				       timestamp);
			handled_irq |= ADE9000_ST1_ZXVC_BIT;
			break;
		case ADE9000_ST1_ZXTOVC_BIT:
			iio_push_event(indio_dev,
				       IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE,
							    ADE9000_ST1_ZXTOVC_BIT,
							    IIO_EV_TYPE_THRESH,
							    IIO_EV_DIR_EITHER),
				       timestamp);
			handled_irq |= ADE9000_ST1_ZXTOVC_BIT;
			break;
		case ADE9000_ST1_ZXIC_BIT:
			iio_push_event(indio_dev,
				       IIO_UNMOD_EVENT_CODE(IIO_CURRENT,
							    ADE9000_ST1_ZXIC_BIT,
							    IIO_EV_TYPE_THRESH,
							    IIO_EV_DIR_EITHER),
				       timestamp);
			handled_irq |= ADE9000_ST1_ZXIC_BIT;
			break;
		case ADE9000_ST1_SWELLA_BIT:
			iio_push_event(indio_dev,
				       IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE,
							    ADE9000_ST1_SWELLA_BIT >> 20,
							    IIO_EV_TYPE_THRESH,
							    IIO_EV_DIR_RISING),
				       timestamp);
			handled_irq |= ADE9000_ST1_SWELLA_BIT;
			break;
		case ADE9000_ST1_SWELLB_BIT:
			iio_push_event(indio_dev,
				       IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE,
							    ADE9000_ST1_SWELLB_BIT >> 20,
							    IIO_EV_TYPE_THRESH,
							    IIO_EV_DIR_RISING),
				       timestamp);
			handled_irq |= ADE9000_ST1_SWELLB_BIT;
			break;
		case ADE9000_ST1_SWELLC_BIT:
			iio_push_event(indio_dev,
				       IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE,
							    ADE9000_ST1_SWELLC_BIT >> 20,
							    IIO_EV_TYPE_THRESH,
							    IIO_EV_DIR_RISING),
				       timestamp);
			handled_irq |= ADE9000_ST1_SWELLC_BIT;
			break;
		case ADE9000_ST1_DIPA_BIT:
			iio_push_event(indio_dev,
				       IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE,
							    ADE9000_ST1_DIPA_BIT >> 20,
							    IIO_EV_TYPE_THRESH,
							    IIO_EV_DIR_FALLING),
				       timestamp);
			handled_irq |= ADE9000_ST1_DIPA_BIT;
			break;
		case ADE9000_ST1_DIPB_BIT:
			iio_push_event(indio_dev,
				       IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE,
							    ADE9000_ST1_DIPB_BIT >> 20,
							    IIO_EV_TYPE_THRESH,
							    IIO_EV_DIR_FALLING),
				       timestamp);
			handled_irq |= ADE9000_ST1_DIPB_BIT;
			break;
		case ADE9000_ST1_DIPC_BIT:
			iio_push_event(indio_dev,
				       IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE,
							    ADE9000_ST1_DIPC_BIT >> 20,
							    IIO_EV_TYPE_THRESH,
							    IIO_EV_DIR_FALLING),
				       timestamp);
			handled_irq |= ADE9000_ST1_DIPC_BIT;
			break;
		case ADE9000_ST1_SEQERR_BIT:
			iio_push_event(indio_dev,
				       IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE,
							    ADE9000_ST1_SEQERR_BIT >> 12,
							    IIO_EV_TYPE_CHANGE,
							    IIO_EV_DIR_NONE),
				       timestamp);
			handled_irq |= ADE9000_ST1_SEQERR_BIT;
			break;
		default:
			return IRQ_HANDLED;
		}
	}

	ret = regmap_write(st->regmap, ADE9000_REG_STATUS1, handled_irq);
	if (ret)
		return ret;

	return IRQ_HANDLED;
}

static irqreturn_t ade9000_dready_thread(int irq, void *data)
{
	struct iio_dev *indio_dev = data;

	/* Handle data ready interrupt from C4/EVENT/DREADY pin */
	if (!iio_device_claim_buffer_mode(indio_dev)) {
		ade9000_iio_push_buffer(indio_dev);
		iio_device_release_buffer_mode(indio_dev);
	}

	return IRQ_HANDLED;
}

static int ade9000_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val,
			    int *val2,
			    long mask)
{
	struct ade9000_state *st = iio_priv(indio_dev);
	unsigned int reg, measured;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_FREQUENCY:
		if (chan->type == IIO_VOLTAGE) {
			int period_reg;
			int period;

			switch (chan->channel) {
			case ADE9000_PHASE_A_NR:
				period_reg = ADE9000_REG_APERIOD;
				break;
			case ADE9000_PHASE_B_NR:
				period_reg = ADE9000_REG_BPERIOD;
				break;
			case ADE9000_PHASE_C_NR:
				period_reg = ADE9000_REG_CPERIOD;
				break;
			default:
				return -EINVAL;
			}
			ret = regmap_read(st->regmap, period_reg, &period);
			if (ret)
				return ret;
			*val = 4000 * 65536;
			*val2 = period + 1;
			return IIO_VAL_FRACTIONAL;
		}

		ret = regmap_read(st->regmap, ADE9000_REG_ACCMODE, &reg);
		if (ret)
			return ret;
		*val = (reg & ADE9000_ACCMODE_60HZ) ? 60 : 50;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_RAW:
		if (chan->type == IIO_ENERGY) {
			u16 lo_reg = chan->address;

			ret = regmap_bulk_read(st->regmap, lo_reg,
					       st->bulk_read_buf, 2);
			if (ret)
				return ret;

			*val = st->bulk_read_buf[0];  /* Lower 32 bits */
			*val2 = st->bulk_read_buf[1]; /* Upper 32 bits */
			return IIO_VAL_INT_64;
		}

		ret = iio_device_claim_direct(indio_dev);
		if (ret)
			return ret;

		ret = regmap_read(st->regmap, chan->address, &measured);
		iio_device_release_direct(indio_dev);
		if (ret)
			return ret;

		*val = measured;

		return IIO_VAL_INT;

	case IIO_CHAN_INFO_POWERFACTOR:
		ret = iio_device_claim_direct(indio_dev);
		if (ret)
			return ret;

		/* Map power channel to corresponding power factor register */
		reg = ADE9000_ADDR_ADJUST(ADE9000_REG_APF, chan->channel);
		ret = regmap_read(st->regmap, reg, &measured);
		iio_device_release_direct(indio_dev);
		if (ret)
			return ret;

		*val = measured;

		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		if (chan->info_mask_shared_by_all) {
			/* Shared PGA gain read - only for channel with shared frequency */
			ret = regmap_read(st->regmap, ADE9000_REG_PGA_GAIN, &reg);
			if (ret)
				return ret;
			*val = min(1 << ((reg >> (8 + chan->channel)) & GENMASK(1, 0)), 4);
			return IIO_VAL_INT;
		} else if (chan->type == IIO_CURRENT || chan->type == IIO_VOLTAGE ||
			   chan->type == IIO_ALTVOLTAGE) {
			switch (chan->address) {
			case ADE9000_REG_AI_PCF:
			case ADE9000_REG_AV_PCF:
			case ADE9000_REG_BI_PCF:
			case ADE9000_REG_BV_PCF:
			case ADE9000_REG_CI_PCF:
			case ADE9000_REG_CV_PCF:
				*val = 1;
				*val2 = ADE9000_PCF_FULL_SCALE_CODES;
				return IIO_VAL_FRACTIONAL;
			case ADE9000_REG_AIRMS:
			case ADE9000_REG_AVRMS:
			case ADE9000_REG_BIRMS:
			case ADE9000_REG_BVRMS:
			case ADE9000_REG_CIRMS:
			case ADE9000_REG_CVRMS:
				*val = 1;
				*val2 = ADE9000_RMS_FULL_SCALE_CODES;
				return IIO_VAL_FRACTIONAL;
			default:
				return -EINVAL;
			}
		} else if (chan->type == IIO_POWER) {
			*val = 1;
			*val2 = ADE9000_WATT_FULL_SCALE_CODES;
			return IIO_VAL_FRACTIONAL;
		} else {
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}
}

static int ade9000_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val,
			     int val2,
			     long mask)
{
	struct ade9000_state *st = iio_priv(indio_dev);
	u32 addr;
	u32 tmp;

	switch (mask) {
	case IIO_CHAN_INFO_FREQUENCY:
		switch (val) {
		case 50:
			return regmap_write(st->regmap, ADE9000_REG_ACCMODE,
					    ADE9000_ACCMODE);
		case 60:
			return regmap_write(st->regmap, ADE9000_REG_ACCMODE,
					    ADE9000_ACCMODE_60HZ);
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_OFFSET:
		switch (chan->type) {
		case IIO_CURRENT:
			return regmap_write(st->regmap,
					    ADE9000_ADDR_ADJUST(ADE9000_REG_AIRMSOS,
								chan->channel), val);
		case IIO_VOLTAGE:
		case IIO_ALTVOLTAGE:
			return regmap_write(st->regmap,
					    ADE9000_ADDR_ADJUST(ADE9000_REG_AVRMSOS,
								chan->channel), val);
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_CALIBBIAS:
		switch (chan->type) {
		case IIO_POWER:
			tmp = chan->address;
			tmp &= ~ADE9000_PHASE_B_POS_BIT;
			tmp &= ~ADE9000_PHASE_C_POS_BIT;

			switch (tmp) {
			case ADE9000_REG_AWATTOS:
				return regmap_write(st->regmap,
						    ADE9000_ADDR_ADJUST(ADE9000_REG_AWATTOS,
									chan->channel), val);
			case ADE9000_REG_AVAR:
				return regmap_write(st->regmap,
						    ADE9000_ADDR_ADJUST(ADE9000_REG_AVAROS,
									chan->channel), val);
			case ADE9000_REG_AFVAR:
				return regmap_write(st->regmap,
						    ADE9000_ADDR_ADJUST(ADE9000_REG_AFVAROS,
									chan->channel), val);
			default:
				return -EINVAL;
			}
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_CALIBSCALE:
		/*
		 * Calibration gain registers for fine-tuning measurements.
		 * These are separate from PGA gain and applied in the digital domain.
		 */
		switch (chan->type) {
		case IIO_CURRENT:
			return regmap_write(st->regmap,
					    ADE9000_ADDR_ADJUST(ADE9000_REG_AIGAIN,
								chan->channel), val);
		case IIO_VOLTAGE:
			return regmap_write(st->regmap,
					    ADE9000_ADDR_ADJUST(ADE9000_REG_AVGAIN,
								chan->channel), val);
		case IIO_POWER:
			return regmap_write(st->regmap,
					    ADE9000_ADDR_ADJUST(ADE9000_REG_APGAIN,
								chan->channel), val);
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_SCALE:
		/* Only shared PGA scale is writable, per-channel scales are read-only */
		if (!(chan->info_mask_shared_by_all))
			return -EINVAL;

		/*
		 * PGA (Programmable Gain Amplifier) settings affect the analog
		 * input stage scaling, shared by all channels. This is different
		 * from the per-channel calibration gains above.
		 */
		if (val > 4 || val < 1 || val == 3)
			return -EINVAL;
		addr = ADE9000_REG_PGA_GAIN;
		/*
		 * PGA gain settings: 1x, 2x, 4x (3x not supported)
		 * Each channel uses 2 bits in PGA_GAIN register:
		 * - Channel 0: bits [9:8]
		 * - Channel 1: bits [11:10]
		 * - Channel 2: bits [13:12]
		 * Convert gain (1,2,4) to register value (0,1,2) using ilog2()
		 */
		val = ilog2(val) << (chan->channel * 2 + 8);
		tmp = GENMASK(1, 0) << (chan->channel * 2 + 8);
		return regmap_update_bits(st->regmap, addr, tmp, val);
	default:
		return -EINVAL;
	}
}

static int ade9000_reg_access(struct iio_dev *indio_dev,
			      unsigned int reg,
			      unsigned int tx_val,
			      unsigned int *rx_val)
{
	struct ade9000_state *st = iio_priv(indio_dev);

	if (rx_val)
		return regmap_read(st->regmap, reg, rx_val);

	return regmap_write(st->regmap, reg, tx_val);
}

static int ade9000_read_event_config(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     enum iio_event_type type,
				     enum iio_event_direction dir)
{
	struct ade9000_state *st = iio_priv(indio_dev);
	u32 interrupts0, interrupts1, number;
	int ret;

	ret = regmap_read(st->regmap, ADE9000_REG_MASK0, &interrupts0);
	if (ret)
		return ret;

	ret = regmap_read(st->regmap, ADE9000_REG_MASK1, &interrupts1);
	if (ret)
		return ret;

	if (type == IIO_EV_TYPE_MAG)
		return interrupts0 & ADE9000_ST0_EGYRDY;

	if (type == IIO_EV_TYPE_CHANGE)
		return interrupts1 & ADE9000_ST1_SEQERR_BIT;

	number = chan->channel;

	switch (number) {
	case ADE9000_PHASE_A_NR:
		if (chan->type == IIO_VOLTAGE) {
			if (dir == IIO_EV_DIR_EITHER)
				return interrupts1 & ADE9000_ST1_ZXVA_BIT;
			if (dir == IIO_EV_DIR_NONE)
				return interrupts1 & ADE9000_ST1_ZXTOVA_BIT;
			if (dir == IIO_EV_DIR_RISING)
				return interrupts1 & ADE9000_ST1_SWELLA_BIT;
			if (dir == IIO_EV_DIR_FALLING)
				return interrupts1 & ADE9000_ST1_DIPA_BIT;
		} else if (chan->type == IIO_CURRENT) {
			return interrupts1 & ADE9000_ST1_ZXIA_BIT;
		}
		break;
	case ADE9000_PHASE_B_NR:
		if (chan->type == IIO_VOLTAGE) {
			if (dir == IIO_EV_DIR_EITHER)
				return interrupts1 & ADE9000_ST1_ZXVB_BIT;
			if (dir == IIO_EV_DIR_NONE)
				return interrupts1 & ADE9000_ST1_ZXTOVB_BIT;
			if (dir == IIO_EV_DIR_RISING)
				return interrupts1 & ADE9000_ST1_SWELLB_BIT;
			if (dir == IIO_EV_DIR_FALLING)
				return interrupts1 & ADE9000_ST1_DIPB_BIT;
		} else if (chan->type == IIO_CURRENT) {
			return interrupts1 & ADE9000_ST1_ZXIB_BIT;
		}
		break;
	case ADE9000_PHASE_C_NR:
		if (chan->type == IIO_VOLTAGE) {
			if (dir == IIO_EV_DIR_EITHER)
				return interrupts1 & ADE9000_ST1_ZXVC_BIT;
			if (dir == IIO_EV_DIR_NONE)
				return interrupts1 & ADE9000_ST1_ZXTOVC_BIT;
			if (dir == IIO_EV_DIR_RISING)
				return interrupts1 & ADE9000_ST1_SWELLC_BIT;
			if (dir == IIO_EV_DIR_FALLING)
				return interrupts1 & ADE9000_ST1_DIPC_BIT;
		} else if (chan->type == IIO_CURRENT) {
			return interrupts1 & ADE9000_ST1_ZXIC_BIT;
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ade9000_write_event_config(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan,
				      enum iio_event_type type,
				      enum iio_event_direction dir,
				      bool state)
{
	struct ade9000_state *st = iio_priv(indio_dev);
	u32 interrupts, tmp;
	int ret;

	/* Clear all pending events in STATUS1 register (write 1 to clear) */
	ret = regmap_write(st->regmap, ADE9000_REG_STATUS1, GENMASK(31, 0));
	if (ret)
		return ret;

	if (type == IIO_EV_TYPE_MAG) {
		ret = regmap_update_bits(st->regmap, ADE9000_REG_STATUS0,
					 ADE9000_ST0_EGYRDY, ADE9000_ST0_EGYRDY);
		if (ret)
			return ret;
		return regmap_update_bits(st->regmap, ADE9000_REG_MASK0,
					 ADE9000_ST0_EGYRDY,
					 state ? ADE9000_ST1_SEQERR_BIT : 0);
	}

	if (type == IIO_EV_TYPE_CHANGE)
		return regmap_update_bits(st->regmap, ADE9000_REG_MASK1,
					 ADE9000_ST1_SEQERR_BIT,
					 state ? ADE9000_ST1_SEQERR_BIT : 0);

	if (dir == IIO_EV_DIR_EITHER) {
		static const struct {
			u32 irq;
			u32 wfb_trg;
		} trig_arr[6] = {
			{
				.irq = ADE9000_ST1_ZXVA_BIT,
				.wfb_trg = ADE9000_WFB_TRG_ZXVA_BIT
			}, {
				.irq = ADE9000_ST1_ZXIA_BIT,
				.wfb_trg = ADE9000_WFB_TRG_ZXIA_BIT
			}, {
				.irq = ADE9000_ST1_ZXVB_BIT,
				.wfb_trg = ADE9000_WFB_TRG_ZXVB_BIT
			}, {
				.irq = ADE9000_ST1_ZXIB_BIT,
				.wfb_trg = ADE9000_WFB_TRG_ZXIB_BIT
			}, {
				.irq = ADE9000_ST1_ZXVC_BIT,
				.wfb_trg = ADE9000_WFB_TRG_ZXVC_BIT
			}, {
				.irq = ADE9000_ST1_ZXIC_BIT,
				.wfb_trg = ADE9000_WFB_TRG_ZXIC_BIT
			},
		};
		if (state) {
			interrupts |= trig_arr[chan->channel * 2 + chan->type].irq;
			st->wfb_trg |= trig_arr[chan->channel * 2 + chan->type].wfb_trg;
		} else {
			interrupts &= ~trig_arr[chan->channel * 2 + chan->type].irq;
			st->wfb_trg &= ~trig_arr[chan->channel * 2 + chan->type].wfb_trg;
		}
	}

	if (dir == IIO_EV_DIR_NONE) {
		switch (chan->channel) {
		case ADE9000_PHASE_A_NR:
			tmp |= ADE9000_ST1_ZXTOVA_BIT;
			break;
		case ADE9000_PHASE_B_NR:
			tmp |= ADE9000_ST1_ZXTOVB_BIT;
			break;
		case ADE9000_PHASE_C_NR:
			tmp |= ADE9000_ST1_ZXTOVC_BIT;
			break;
		default:
			break;
		}

		if (state)
			interrupts |= tmp;
		else
			interrupts &= ~tmp;
	} else if (dir == IIO_EV_DIR_RISING) {
		switch (chan->channel) {
		case ADE9000_PHASE_A_NR:
			tmp |= ADE9000_ST1_SWELLA_BIT;
			break;
		case ADE9000_PHASE_B_NR:
			tmp |= ADE9000_ST1_SWELLB_BIT;
			break;
		case ADE9000_PHASE_C_NR:
			tmp |= ADE9000_ST1_SWELLC_BIT;
			break;
		default:
			break;
		}

		if (state) {
			interrupts |= tmp;
			st->wfb_trg |= ADE9000_WFB_TRG_SWELL_BIT;
		} else {
			interrupts &= ~tmp;
			st->wfb_trg &= ~ADE9000_WFB_TRG_SWELL_BIT;
		}
	} else if (dir == IIO_EV_DIR_FALLING) {
		switch (chan->channel) {
		case ADE9000_PHASE_A_NR:
			tmp |= ADE9000_ST1_DIPA_BIT;
			break;
		case ADE9000_PHASE_B_NR:
			tmp |= ADE9000_ST1_DIPB_BIT;
			break;
		case ADE9000_PHASE_C_NR:
			tmp |= ADE9000_ST1_DIPC_BIT;
			break;
		default:
			break;
		}

		if (state) {
			interrupts |= tmp;
			st->wfb_trg |= ADE9000_WFB_TRG_DIP_BIT;
		} else {
			interrupts &= ~tmp;
			st->wfb_trg &= ~ADE9000_WFB_TRG_DIP_BIT;
		}
	}

	return regmap_update_bits(st->regmap, ADE9000_REG_MASK1, interrupts,
				  interrupts);
}

static int ade9000_write_event_value(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     enum iio_event_type type,
				     enum iio_event_direction dir,
				     enum iio_event_info info,
				     int val, int val2)
{
	struct ade9000_state *st = iio_priv(indio_dev);

	switch (info) {
	case IIO_EV_INFO_VALUE:
		switch (dir) {
		case IIO_EV_DIR_FALLING:
			return regmap_write(st->regmap, ADE9000_REG_DIP_LVL, val);
		case IIO_EV_DIR_RISING:
			return regmap_write(st->regmap, ADE9000_REG_SWELL_LVL, val);
		case IIO_EV_DIR_NONE:
			return regmap_write(st->regmap, ADE9000_REG_ZXTOUT, val);
		default:
			return -EINVAL;
		}
	default:
		return -EINVAL;
	}
}

static int ade9000_read_event_value(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    enum iio_event_type type,
				    enum iio_event_direction dir,
				    enum iio_event_info info,
				    int *val, int *val2)
{
	struct ade9000_state *st = iio_priv(indio_dev);
	unsigned int data;
	int ret;

	switch (info) {
	case IIO_EV_INFO_VALUE:
		switch (dir) {
		case IIO_EV_DIR_FALLING:
			ret = regmap_read(st->regmap, ADE9000_REG_DIP_LVL, &data);
			if (ret)
				return ret;
			*val = data;
			return IIO_VAL_INT;
		case IIO_EV_DIR_RISING:
			ret = regmap_read(st->regmap, ADE9000_REG_SWELL_LVL, &data);
			if (ret)
				return ret;
			*val = data;
			return IIO_VAL_INT;
		case IIO_EV_DIR_NONE:
			ret = regmap_read(st->regmap, ADE9000_REG_ZXTOUT, &data);
			if (ret)
				return ret;
			*val = data;
			return IIO_VAL_INT;
		default:
			return -EINVAL;
		}
	default:
		return -EINVAL;
	}
}

static int ade9000_waveform_buffer_config(struct iio_dev *indio_dev)
{
	struct ade9000_state *st = iio_priv(indio_dev);
	u32 wfb_cfg_val = 0;
	u32 active_scans;

	bitmap_to_arr32(&active_scans, indio_dev->active_scan_mask,
			indio_dev->masklength);

	switch (active_scans) {
	case ADE9000_SCAN_POS_IA | ADE9000_SCAN_POS_VA:
		wfb_cfg_val = ADE9000_WFB_CFG_IA_VA;
		st->wfb_nr_activ_chan = 2;
		break;
	case ADE9000_SCAN_POS_IB | ADE9000_SCAN_POS_VB:
		wfb_cfg_val = ADE9000_WFB_CFG_IB_VB;
		st->wfb_nr_activ_chan = 2;
		break;
	case ADE9000_SCAN_POS_IC | ADE9000_SCAN_POS_VC:
		wfb_cfg_val = ADE9000_WFB_CFG_IC_VC;
		st->wfb_nr_activ_chan = 2;
		break;
	case ADE9000_SCAN_POS_IA:
		wfb_cfg_val = ADE9000_WFB_CFG_IA;
		st->wfb_nr_activ_chan = 1;
		break;
	case ADE9000_SCAN_POS_VA:
		wfb_cfg_val = ADE9000_WFB_CFG_VA;
		st->wfb_nr_activ_chan = 1;
		break;
	case ADE9000_SCAN_POS_IB:
		wfb_cfg_val = ADE9000_WFB_CFG_IB;
		st->wfb_nr_activ_chan = 1;
		break;
	case ADE9000_SCAN_POS_VB:
		wfb_cfg_val = ADE9000_WFB_CFG_VB;
		st->wfb_nr_activ_chan = 1;
		break;
	case ADE9000_SCAN_POS_IC:
		wfb_cfg_val = ADE9000_WFB_CFG_IC;
		st->wfb_nr_activ_chan = 1;
		break;
	case ADE9000_SCAN_POS_VC:
		wfb_cfg_val = ADE9000_WFB_CFG_VC;
		st->wfb_nr_activ_chan = 1;
		break;
	case (ADE9000_SCAN_POS_IA | ADE9000_SCAN_POS_VA | ADE9000_SCAN_POS_IB |
	      ADE9000_SCAN_POS_VB | ADE9000_SCAN_POS_IC | ADE9000_SCAN_POS_VC):
		wfb_cfg_val = ADE9000_WFB_CFG_ALL_CHAN;
		st->wfb_nr_activ_chan = 6;
		break;
	default:
		dev_err(&st->spi->dev, "Unsupported combination of scans\n");
		return -EINVAL;
	}

	wfb_cfg_val |= FIELD_PREP(ADE9000_WF_SRC_MASK, st->wf_src);

	return regmap_write(st->regmap, ADE9000_REG_WFB_CFG, wfb_cfg_val);
}

static int ade9000_waveform_buffer_interrupt_setup(struct ade9000_state *st)
{
	int ret;

	ret = regmap_write(st->regmap, ADE9000_REG_WFB_TRG_CFG, 0x0);
	if (ret)
		return ret;

	/* Always use streaming mode setup */
	ret = regmap_write(st->regmap, ADE9000_REG_WFB_PG_IRQEN,
			   ADE9000_MIDDLE_PAGE_BIT);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, ADE9000_REG_STATUS0, GENMASK(31, 0));
	if (ret)
		return ret;

	return regmap_set_bits(st->regmap, ADE9000_REG_MASK0,
			       ADE9000_ST0_PAGE_FULL_BIT);
}

static int ade9000_buffer_preenable(struct iio_dev *indio_dev)
{
	struct ade9000_state *st = iio_priv(indio_dev);
	int ret;

	ret = ade9000_waveform_buffer_config(indio_dev);
	if (ret)
		return ret;

	st->wfb_nr_samples = ADE9000_WFB_MAX_SAMPLES_CHAN * st->wfb_nr_activ_chan;

	ret = ade9000_configure_scan(indio_dev, ADE9000_REG_WF_BUFF);
	if (ret)
		return ret;

	ret = ade9000_waveform_buffer_interrupt_setup(st);
	if (ret)
		return ret;

	ret = regmap_update_bits(st->regmap, ADE9000_REG_WFB_CFG,
				 ADE9000_WF_CAP_EN_MASK,
				 ADE9000_WF_CAP_EN_MASK);
	if (ret) {
		dev_err(&st->spi->dev, "Post-enable waveform buffer enable fail\n");
		return ret;
	}

	return 0;
}

static int ade9000_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct ade9000_state *st = iio_priv(indio_dev);
	struct device *dev = &st->spi->dev;
	u32 interrupts;
	int ret;

	ret = regmap_update_bits(st->regmap, ADE9000_REG_WFB_CFG,
				 ADE9000_WF_CAP_EN_MASK, 0);
	if (ret) {
		dev_err(dev, "Post-disable waveform buffer disable fail\n");
		return ret;
	}

	ret = regmap_write(st->regmap, ADE9000_REG_WFB_TRG_CFG, 0x0);
	if (ret)
		return ret;

	interrupts = ADE9000_ST0_WFB_TRIG_BIT | ADE9000_ST0_PAGE_FULL_BIT;

	ret = regmap_update_bits(st->regmap, ADE9000_REG_MASK0, interrupts, 0);
	if (ret) {
		dev_err(dev, "Post-disable update maks0 fail\n");
		return ret;
	}

	return regmap_write(st->regmap, ADE9000_REG_STATUS0, GENMASK(31, 0));
}

static int ade9000_reset(struct ade9000_state *st)
{
	struct device *dev = &st->spi->dev;
	struct gpio_desc *gpio_reset;
	int ret;

	gpio_reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(gpio_reset))
		return PTR_ERR(gpio_reset);

	if (gpio_reset) {
		fsleep(10);
		gpiod_set_value_cansleep(gpio_reset, 0);
		fsleep(50000);
	} else {
		ret = regmap_update_bits(st->regmap, ADE9000_REG_CONFIG1,
					 ADE9000_SWRST_BIT, ADE9000_SWRST_BIT);
		if (ret)
			return ret;
		fsleep(90);
		return 0;
	}

	if (!wait_for_completion_timeout(&st->reset_completion,
					 msecs_to_jiffies(1000))) {
		dev_err(dev, "Reset timeout after 1s\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static int ade9000_setup(struct ade9000_state *st)
{
	struct device *dev = &st->spi->dev;
	int ret;

	ret = regmap_multi_reg_write(st->regmap, ade9000_reg_sequence,
				     ARRAY_SIZE(ade9000_reg_sequence));
	if (ret)
		return dev_err_probe(dev, ret, "Failed to write register sequence");

	fsleep(2000);

	/* Clear all pending status bits by writing 1s */
	ret = regmap_write(st->regmap, ADE9000_REG_STATUS0, GENMASK(31, 0));
	if (ret)
		return dev_err_probe(dev, ret, "Failed to clear STATUS0");

	ret = regmap_write(st->regmap, ADE9000_REG_STATUS1, GENMASK(31, 0));
	if (ret)
		return dev_err_probe(dev, ret, "Failed to clear STATUS1");

	return 0;
}

static const struct iio_buffer_setup_ops ade9000_buffer_ops = {
	.preenable = &ade9000_buffer_preenable,
	.postdisable = &ade9000_buffer_postdisable,
};

static const struct iio_info ade9000_info = {
	.read_raw = ade9000_read_raw,
	.write_raw = ade9000_write_raw,
	.debugfs_reg_access = ade9000_reg_access,
	.write_event_config = ade9000_write_event_config,
	.read_event_config = ade9000_read_event_config,
	.write_event_value = ade9000_write_event_value,
	.read_event_value = ade9000_read_event_value,
};

static const struct regmap_config ade9000_regmap_config = {
	.reg_bits = 16,
	.val_bits = 32,
	.zero_flag_mask = true,
	.cache_type = REGCACHE_RBTREE,
	.reg_read = ade9000_spi_read_reg,
	.reg_write = ade9000_spi_write_reg,
	.volatile_reg = ade9000_is_volatile_reg,
};

static int ade9000_setup_clkout(struct device *dev, struct ade9000_state *st)
{
	struct clk_init_data clk_init = {};
	struct clk *clkout;
	int ret;

	/*
	 * Only provide clock output when using external CMOS clock.
	 * When using crystal, CLKOUT is connected to crystal and shouldn't
	 * be used as clock provider for other devices.
	 */
	if (!device_property_present(dev, "#clock-cells") || !st->clkin)
		return 0;

	clk_init.name = "clkout";
	clk_init.ops = &ade9000_clkout_ops;
	clk_init.flags = CLK_GET_RATE_NOCACHE;
	clk_init.num_parents = 0;

	st->clkout_hw.init = &clk_init;

	clkout = devm_clk_register(dev, &st->clkout_hw);
	if (IS_ERR(clkout))
		return dev_err_probe(dev, PTR_ERR(clkout), "Failed to register clkout");

	ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_simple_get, &st->clkout_hw);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to add clock provider");

	return 0;
}

static int ade9000_request_irq(struct device *dev, const char *name,
			       irq_handler_t handler, void *dev_id)
{
	int irq, ret;

	irq = fwnode_irq_get_byname(dev_fwnode(dev), name);
	if (irq < 0)
		return 0;

	ret = devm_request_threaded_irq(dev, irq, NULL, handler,
					IRQF_ONESHOT, KBUILD_MODNAME, dev_id);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to request %s irq", name);

	return 0;
}

static int ade9000_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct iio_dev *indio_dev;
	struct ade9000_state *st;
	struct regmap *regmap;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return dev_err_probe(dev, -ENOMEM, "Unable to allocate ADE9000 IIO");

	st = iio_priv(indio_dev);

	regmap = devm_regmap_init(dev, NULL, st, &ade9000_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap), "Unable to allocate ADE9000 regmap");

	st->regmap = regmap;
	st->spi = spi;

	init_completion(&st->reset_completion);

	ret = ade9000_request_irq(dev, "irq0", ade9000_irq0_thread, indio_dev);
	if (ret)
		return ret;

	ret = ade9000_request_irq(dev, "irq1", ade9000_irq1_thread, indio_dev);
	if (ret)
		return ret;

	ret = ade9000_request_irq(dev, "dready", ade9000_dready_thread, indio_dev);
	if (ret)
		return ret;

	ret = devm_mutex_init(dev, &st->lock);
	if (ret)
		return ret;

	/* External CMOS clock input (optional - crystal can be used instead) */
	st->clkin = devm_clk_get_optional_enabled(dev, "clkin");
	if (IS_ERR(st->clkin))
		return dev_err_probe(dev, PTR_ERR(st->clkin), "Failed to get and enable clkin");

	ret = ade9000_setup_clkout(dev, st);
	if (ret)
		return ret;

	indio_dev->name = "ade9000";
	indio_dev->info = &ade9000_info;
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_SOFTWARE;
	indio_dev->setup_ops = &ade9000_buffer_ops;

	ret = devm_regulator_get_enable(&spi->dev, "vdd");
	if (ret)
		return dev_err_probe(&spi->dev, ret,
				     "Failed to get and enable vdd regulator\n");

	ret = devm_regulator_get_enable_optional(dev, "vref");
	if (ret < 0 && ret != -ENODEV)
		return dev_err_probe(dev, ret,
				     "Failed to get and enable vref regulator\n");

	/* Configure reference selection based on vref regulator availability */
	if (ret != -ENODEV) {
		ret = regmap_update_bits(st->regmap, ADE9000_REG_CONFIG1,
					 ADE9000_EXT_REF_MASK,
					 ADE9000_EXT_REF_MASK);
		if (ret)
			return ret;
	}

	indio_dev->channels = ade9000_channels;
	indio_dev->num_channels = ARRAY_SIZE(ade9000_channels);

	ret = devm_iio_kfifo_buffer_setup(dev, indio_dev,
					  &ade9000_buffer_ops);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to setup IIO buffer");

	ret = ade9000_reset(st);
	if (ret)
		return ret;

	ret = ade9000_setup(st);
	if (ret)
		return ret;

	return devm_iio_device_register(dev, indio_dev);
};

static const struct spi_device_id ade9000_id[] = {
	{ "ade9000", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, ade9000_id);

static const struct of_device_id ade9000_of_match[] = {
	{ .compatible = "adi,ade9000" },
	{ }
};
MODULE_DEVICE_TABLE(of, ade9000_of_match);

static struct spi_driver ade9000_driver = {
	.driver = {
		.name = "ade9000",
		.of_match_table = ade9000_of_match,
	},
	.probe = ade9000_probe,
	.id_table = ade9000_id,
};
module_spi_driver(ade9000_driver);

MODULE_AUTHOR("Antoniu Miclaus <antoniu.miclaus@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADE9000");
MODULE_LICENSE("GPL");
