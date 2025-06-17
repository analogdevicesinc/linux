// SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause
/**
 * ADE9078 driver
 *
 * Copyright 2021 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/events.h>
#include <linux/iio/sysfs.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>
#include <asm/unaligned.h>

/* Address of ADE90XX registers */
#define	ADE9078_REG_AIGAIN		0x000
#define	ADE9078_REG_AVGAIN		0x00B
#define	ADE9078_REG_AIRMSOS		0x00C
#define	ADE9078_REG_AVRMSOS		0x00D
#define	ADE9078_REG_APGAIN		0x00E
#define	ADE9078_REG_AWATTOS		0x00F
#define	ADE9078_REG_AVAROS		0x010
#define	ADE9078_REG_AFVAROS		0x012
#define	ADE9078_REG_CONFIG0		0x060
#define	ADE9078_REG_DICOEFF		0x072
#define	ADE9078_REG_AI_PCF		0x20A
#define	ADE9078_REG_AV_PCF		0x20B
#define	ADE9078_REG_AIRMS		0x20C
#define	ADE9078_REG_AVRMS		0x20D
#define	ADE9078_REG_AWATT		0x210
#define	ADE9078_REG_AVAR		0x211
#define	ADE9078_REG_AVA			0x212
#define ADE9078_REG_AFVAR		0x214
#define	ADE9078_REG_APF			0x216
#define	ADE9078_REG_BI_PCF		0x22A
#define	ADE9078_REG_BV_PCF		0x22B
#define	ADE9078_REG_BIRMS		0x22C
#define	ADE9078_REG_BVRMS		0x22D
#define	ADE9078_REG_CI_PCF		0x24A
#define	ADE9078_REG_CV_PCF		0x24B
#define	ADE9078_REG_CIRMS		0x24C
#define	ADE9078_REG_CVRMS		0x24D
#define	ADE9078_REG_AWATT_ACC		0x2E5
#define ADE9078_REG_AWATTHR_LO		0x2E6
#define ADE9078_REG_AVAHR_LO		0x2FA
#define ADE9078_REG_AFVARHR_LO		0x30E
#define ADE9078_REG_BWATTHR_LO		0x322
#define ADE9078_REG_BVAHR_LO		0x336
#define ADE9078_REG_BFVARHR_LO		0x34A
#define ADE9078_REG_CWATTHR_LO		0x35E
#define ADE9078_REG_CVAHR_LO		0x372
#define ADE9078_REG_CFVARHR_LO		0x386
#define	ADE9078_REG_STATUS0		0x402
#define	ADE9078_REG_STATUS1		0x403
#define	ADE9078_REG_MASK0		0x405
#define	ADE9078_REG_MASK1		0x406
#define	ADE9078_REG_EVENT_MASK		0x407
#define	ADE9078_REG_VLEVEL		0x40F
#define	ADE9078_REG_DIP_LVL		0x410
#define	ADE9078_REG_DIPA		0x411
#define	ADE9078_REG_DIPB		0x412
#define	ADE9078_REG_DIPC		0x413
#define	ADE9078_REG_SWELL_LVL		0x414
#define	ADE9078_REG_SWELLA		0x415
#define	ADE9078_REG_SWELLB		0x416
#define	ADE9078_REG_SWELLC		0x417
#define ADE9078_REG_APERIOD		0x418
#define ADE9078_REG_BPERIOD		0x419
#define ADE9078_REG_CPERIOD		0x41A
#define	ADE9078_REG_RUN			0x480
#define ADE9078_REG_CONFIG1		0x481
#define	ADE9078_REG_ACCMODE		0x492
#define	ADE9078_REG_CONFIG3		0x493
#define ADE9078_REG_ZXTOUT		0x498
#define	ADE9078_REG_ZX_LP_SEL		0x49A
#define	ADE9078_REG_WFB_CFG		0x4A0
#define	ADE9078_REG_WFB_PG_IRQEN	0x4A1
#define	ADE9078_REG_WFB_TRG_CFG		0x4A2
#define	ADE9078_REG_WFB_TRG_STAT	0x4A3
#define	ADE9078_REG_CONFIG2		0x4AF
#define	ADE9078_REG_EP_CFG		0x4B0
#define	ADE9078_REG_EGY_TIME		0x4B2
#define	ADE9078_REG_PGA_GAIN		0x4B9
#define	ADE9078_REG_VERSION		0x4FE
#define ADE9078_REG_WF_BUFF		0x800
#define ADE9078_REG_WF_HALF_BUFF	0xC00

#define ADE9078_REG_ADDR_MASK		GENMASK(15, 4)
#define ADE9078_REG_READ_BIT_MASK	BIT(3)
#define ADE9078_RX_DEPTH		6
#define ADE9078_TX_DEPTH		10

#define ADE9078_WF_CAP_EN_MASK		BIT(4)
#define ADE9078_WF_CAP_SEL_MASK		BIT(5)
#define ADE9078_WF_MODE_MASK		GENMASK(7, 6)
#define ADE9078_WF_SRC_MASK		GENMASK(9, 8)
#define ADE9078_WF_IN_EN_MASK		BIT(12)

/*
 * Configuration registers
 * PGA@0x0000. Gain of all channels=1
 */
#define ADE9078_PGA_GAIN		0x0000

/* Default configuration */
#define ADE9078_CONFIG0			0x00000000

/* CF3/ZX pin outputs Zero crossing */
#define ADE9078_CONFIG1			0x0002

//TODO: This looks like it could be better expressed in terms
//of defines for the fields contained in this register.
//Same for many of the ones that follow.

/* Default High pass corner frequency of 1.25Hz */
#define ADE9078_CONFIG2			0x0A00

/* Peak and overcurrent detection disabled */
#define ADE9078_CONFIG3			0x0000

/*
 * 50Hz operation, 3P4W Wye configuration, signed accumulation
 * Clear bit 8 i.e. ACCMODE=0x00xx for 50Hz operation
 * ACCMODE=0x0x9x for 3Wire delta when phase B is used as reference
 */
#define ADE9078_ACCMODE			0x0000
#define ADE9078_ACCMODE_60HZ		0x0100

/*Line period and zero crossing obtained from VA */
#define ADE9078_ZX_LP_SEL		0x0000

/* Disable all interrupts */
#define ADE9078_MASK0			0x00000001

/* Disable all interrupts */
#define ADE9078_MASK1			0x00000000

/* Events disabled */
#define ADE9078_EVENT_MASK		0x00000000

/*
 * Assuming Vnom=1/2 of full scale.
 * Refer to Technical reference manual for detailed calculations.
 */
#define ADE9078_VLEVEL			0x0022EA28

/* Set DICOEFF= 0xFFFFE000 when integrator is enabled */
#define ADE9078_DICOEFF			0x00000000

/* DSP ON */
#define ADE9078_RUN_ON			0xFFFFFFFF

/*
 * Energy Accumulation Settings
 * Enable energy accumulation, accumulate samples at 8ksps
 * latch energy accumulation after EGYRDY
 * If accumulation is changed to half line cycle mode, change EGY_TIME
 */
#define ADE9078_EP_CFG			0x0011

/* Accumulate 4000 samples */
#define ADE9078_EGY_TIME		7999

/*
 * Constant Definitions
 * ADE9000 FDSP: 8000sps, ADE9078 FDSP: 4000sps
 */
#define ADE9078_FDSP			4000
#define ADE9078_WFB_CFG			0x0329
#define ADE9078_WFB_PAGE_SIZE		128
#define ADE9078_WFB_NR_OF_PAGES		16
#define ADE9078_WFB_MAX_CHANNELS	8
#define ADE9078_WFB_BYTES_IN_SAMPLE	4
#define ADE9078_WFB_SAMPLES_IN_PAGE	\
	(ADE9078_WFB_PAGE_SIZE / ADE9078_WFB_MAX_CHANNELS)
#define ADE9078_WFB_MAX_SAMPLES_CHAN	\
	(ADE9078_WFB_SAMPLES_IN_PAGE * ADE9078_WFB_NR_OF_PAGES)
#define ADE9078_WFB_FULL_BUFF_NR_SAMPLES \
	(ADE9078_WFB_PAGE_SIZE * ADE9078_WFB_NR_OF_PAGES)
#define ADE9078_WFB_FULL_BUFF_SIZE	\
	(ADE9078_WFB_FULL_BUFF_NR_SAMPLES * ADE9078_WFB_BYTES_IN_SAMPLE)

#define ADE9078_SWRST_BIT		BIT(0)

/* Status and Mask register bits*/
#define ADE9078_ST0_WFB_TRIG_BIT	BIT(16)
#define ADE9078_ST0_PAGE_FULL_BIT	BIT(17)
#define ADE9078_ST0_EGYRDY		BIT(0)

#define ADE9078_ST1_ZXTOVA_BIT		BIT(6)
#define ADE9078_ST1_ZXTOVB_BIT		BIT(7)
#define ADE9078_ST1_ZXTOVC_BIT		BIT(8)
#define ADE9078_ST1_ZXVA_BIT		BIT(9)
#define ADE9078_ST1_ZXVB_BIT		BIT(10)
#define ADE9078_ST1_ZXVC_BIT		BIT(11)
#define ADE9078_ST1_ZXIA_BIT		BIT(13)
#define ADE9078_ST1_ZXIB_BIT		BIT(14)
#define ADE9078_ST1_ZXIC_BIT		BIT(15)
#define ADE9078_ST1_RSTDONE_BIT		BIT(16)
#define ADE9078_ST1_SEQERR_BIT    	BIT(18)
#define ADE9078_ST1_SWELLA_BIT    	BIT(20)
#define ADE9078_ST1_SWELLB_BIT    	BIT(21)
#define ADE9078_ST1_SWELLC_BIT    	BIT(22)
#define ADE9078_ST1_DIPA_BIT    	BIT(23)
#define ADE9078_ST1_DIPB_BIT    	BIT(24)
#define ADE9078_ST1_DIPC_BIT    	BIT(25)
#define ADE9078_ST1_ERROR0_BIT		BIT(28)
#define ADE9078_ST1_ERROR1_BIT		BIT(29)
#define ADE9078_ST1_ERROR2_BIT		BIT(30)
#define ADE9078_ST1_ERROR3_BIT		BIT(31)
#define ADE9078_ST_ERROR \
	(ADE9078_ST1_ERROR0 | \
	 ADE9078_ST1_ERROR1 | \
	 ADE9078_ST1_ERROR2 | \
	 ADE9078_ST1_ERROR3)
#define ADE9078_ST1_CROSSING_FIRST	6
#define ADE9078_ST1_CROSSING_DEPTH	25

#define ADE9078_WFB_TRG_DIP_BIT		BIT(0)
#define ADE9078_WFB_TRG_SWELL_BIT	BIT(1)
#define ADE9078_WFB_TRG_ZXIA_BIT	BIT(3)
#define ADE9078_WFB_TRG_ZXIB_BIT	BIT(4)
#define ADE9078_WFB_TRG_ZXIC_BIT	BIT(5)
#define ADE9078_WFB_TRG_ZXVA_BIT	BIT(6)
#define ADE9078_WFB_TRG_ZXVB_BIT	BIT(7)
#define ADE9078_WFB_TRG_ZXVC_BIT	BIT(8)

/* Stop when waveform buffer is full */
#define ADE9078_WFB_FULL_MODE		0x0
/* Continuous fill—stop only on enabled trigger events */
#define ADE9078_WFB_EN_TRIG_MODE	0x1
/* Continuous filling—center capture around enabled trigger events */
#define ADE9078_WFB_C_EN_TRIG_MODE	0x2
/* Continuous fill—used as streaming mode for continuous data output */
#define ADE9078_WFB_STREAMING_MODE	0x3

#define ADE9078_LAST_PAGE_BIT		BIT(15)
#define ADE9078_MIDDLE_PAGE_BIT		BIT(7)

/*
 * Full scale Codes referred from Datasheet.Respective digital codes are
 * produced when ADC inputs are at full scale. Do not Change.
 */
#define ADE9078_RMS_FULL_SCALE_CODES	52866837
#define ADE9000_WATT_FULL_SCALE_CODES	20694066
#define ADE9078_PCF_FULL_SCALE_CODES	74770000

/* Phase and channel definitions */
#define ADE9078_PHASE_A_NR		0
#define ADE9078_PHASE_B_NR		2
#define ADE9078_PHASE_C_NR		4

#define ADE9078_SCAN_POS_IA		BIT(0)
#define ADE9078_SCAN_POS_VA		BIT(1)
#define ADE9078_SCAN_POS_IB		BIT(2)
#define ADE9078_SCAN_POS_VB		BIT(3)
#define ADE9078_SCAN_POS_IC		BIT(4)
#define ADE9078_SCAN_POS_VC		BIT(5)
#define ADE9078_SCAN_POS_ALL \
	(ADE9078_SCAN_POS_IA | \
	 ADE9078_SCAN_POS_VA | \
	 ADE9078_SCAN_POS_IB | \
	 ADE9078_SCAN_POS_VB | \
	 ADE9078_SCAN_POS_IC | \
	 ADE9078_SCAN_POS_VC)

#define ADE9078_PHASE_B_POS_BIT		BIT(5)
#define ADE9078_PHASE_C_POS_BIT		BIT(6)

#define ADE9078_MAX_PHASE_NR		3
#define AD9078_CHANNELS_PER_PHASE	18

#define ADE9078_ADDR_ADJUST(addr, chan)					\
	(((chan) << 4) | (addr))

/**
 * struct ade9078_state - ADE9078 specific data
 * @rst_done:		flag for when reset sequence irq has been received
 * @wf_mode:		wave form buffer mode, read datasheet for more details,
 *			retrieved from DT
 * @wfb_trg:		wave form buffer triger configuration, read datasheet
 *			for more details, retrieved from DT
 * @wfb_nr_activ_chan:	number of active channels in the IIO buffer
 * @wfb_nr_samples:	number of maximum samples in the IC buffer
 * @spi:		spi device associated to the ade9078
 * @tx:			transmit buffer for the spi
 * @rx:			receive buffer for the spi
 * @xfer:		transfer setup used in iio buffer configuration
 * @spi_msg:		message transfer trough spi, used in iio buffer
 *			configuration
 * @regmap:		register map pointer
 * @indio_dev:		the IIO device
 * @trig:		iio trigger pointer, is connected to IRQ0 and IRQ1
 * @rx_buff:		receive buffer for the iio buffer trough spi, will
 *			contain the samples from the IC wave form buffer
 * @tx_buff:		transmit buffer for the iio buffer trough spi, used
 *			in iio	buffer configuration
 */
struct ade9078_state {
	bool rst_done;
	u8 wf_mode;
	u32 wfb_trg;
	u8 wfb_nr_activ_chan;
	u32 wfb_nr_samples;
	struct spi_device *spi;
	u8 *tx;
	u8 *rx;
	u32 egy_active_accum[3];
	u32 egy_apparent_accum[3];
	u32 egy_reactive_accum[3];
	struct spi_transfer xfer[2];
	struct spi_message spi_msg;
	struct regmap *regmap;
	union{
		u8 byte[ADE9078_WFB_FULL_BUFF_SIZE];
		__be32 word[ADE9078_WFB_FULL_BUFF_NR_SAMPLES];
	} rx_buff ____cacheline_aligned;
	u8 tx_buff[2];
};

static const struct iio_event_spec ade9078_events[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_EITHER,
		.mask_separate = BIT(IIO_EV_INFO_ENABLE) |
				 BIT(IIO_EV_INFO_VALUE),
	},
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING, // For swell
		.mask_separate = BIT(IIO_EV_INFO_ENABLE) | BIT(IIO_EV_INFO_VALUE),
    	},
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING, // For dip
		.mask_separate = BIT(IIO_EV_INFO_ENABLE) | BIT(IIO_EV_INFO_VALUE),
	},
};

static ssize_t ade9078_read_zxtout(struct iio_dev *indio_dev,
				uintptr_t private,
				const struct iio_chan_spec *chan, char *buf)
{
	struct ade9078_state *st = iio_priv(indio_dev);
	unsigned int regval;
	int ret;

	ret = regmap_read(st->regmap, ADE9078_REG_ZXTOUT, &regval);

	return ret ? ret : sprintf(buf, "%d\n", regval);
}

static ssize_t ade9078_write_zxtout(struct iio_dev *indio_dev,
				uintptr_t private,
				const struct iio_chan_spec *chan,
				const char *buf, size_t len)
{
	struct ade9078_state *st = iio_priv(indio_dev);
	u32 val;
	int ret;

	ret = kstrtou32(buf, 0, &val);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, ADE9078_REG_ZXTOUT, val);

	return ret ? ret : len;
}

static ssize_t ade9078_read_swell_lvl(struct iio_dev *indio_dev,
				uintptr_t private,
				const struct iio_chan_spec *chan, char *buf)
{
	struct ade9078_state *st = iio_priv(indio_dev);
	unsigned int regval;
	int ret;

	ret = regmap_read(st->regmap, ADE9078_REG_SWELL_LVL, &regval);

	return ret ? ret : sprintf(buf, "%d\n", regval);
}

static ssize_t ade9078_write_swell_lvl(struct iio_dev *indio_dev,
				uintptr_t private,
				const struct iio_chan_spec *chan,
				const char *buf, size_t len)
{
	struct ade9078_state *st = iio_priv(indio_dev);
	u32 val;
	int ret;

	ret = kstrtou32(buf, 0, &val);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, ADE9078_REG_SWELL_LVL, val);

	return ret ? ret : len;
}

static ssize_t ade9078_read_dip_lvl(struct iio_dev *indio_dev,
				uintptr_t private,
				const struct iio_chan_spec *chan, char *buf)
{
	struct ade9078_state *st = iio_priv(indio_dev);
	unsigned int regval;
	int ret;

	ret = regmap_read(st->regmap, ADE9078_REG_DIP_LVL, &regval);

	return ret ? ret : sprintf(buf, "%d\n", regval);
}

static ssize_t ade9078_write_dip_lvl(struct iio_dev *indio_dev,
				uintptr_t private,
				const struct iio_chan_spec *chan,
				const char *buf, size_t len)
{
	struct ade9078_state *st = iio_priv(indio_dev);
	u32 val;
	int ret;

	ret = kstrtou32(buf, 0, &val);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, ADE9078_REG_DIP_LVL, val);

	return ret ? ret : len;
}


static const struct iio_chan_spec_ext_info ade9078_voltage_ext_info[] = {
    {
        .name = "zxtout",
        .read = ade9078_read_zxtout,
        .write = ade9078_write_zxtout,
        .shared = IIO_SHARED_BY_ALL,
    },
    { }
};

static const struct iio_chan_spec_ext_info ade9078_swell_lvl_ext_info[] = {
    {
        .name = "swell_lvl",
        .read = ade9078_read_swell_lvl,
        .write = ade9078_write_swell_lvl,
        .shared = IIO_SHARED_BY_TYPE,
    },
    { }
};

static const struct iio_chan_spec_ext_info ade9078_dip_lvl_ext_info[] = {
    {
        .name = "dip_lvl",
        .read = ade9078_read_dip_lvl,
        .write = ade9078_write_dip_lvl,
        .shared = IIO_SHARED_BY_TYPE,
    },
    { }
};


//TODO extend_name defines new ABI.  Needs documentation in
//Documentation/ABI/testing/sysfs-bus-iio-*
//and a very strong reason why it makes sense to do it this way rather than
//via modifiers or similar.
#define ADE9078_CURRENT_CHANNEL(num, name) {				\
	.type = IIO_CURRENT,						\
	.channel = num,							\
	.extend_name = name,						\
	.address = ADE9078_ADDR_ADJUST(ADE9078_REG_AI_PCF, num),	\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_SCALE) |		\
			      BIT(IIO_CHAN_INFO_HARDWAREGAIN),		\
	.event_spec = &ade9078_events[0],				\
	.num_event_specs = 1,						\
	.scan_index = num,						\
	.indexed = 1,							\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = 32,						\
		.storagebits = 32,					\
		.shift = 0,						\
		.endianness = IIO_BE,					\
	},								\
}

#define ADE9078_VOLTAGE_CHANNEL(num, name) {				\
	.type = IIO_VOLTAGE,						\
	.channel = num,							\
	.extend_name = name,						\
	.address = ADE9078_ADDR_ADJUST(ADE9078_REG_AV_PCF, num),	\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_SCALE) |		\
			      BIT(IIO_CHAN_INFO_CALIBSCALE) |		\
			      BIT(IIO_CHAN_INFO_HARDWAREGAIN),		\
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.event_spec = ade9078_events,					\
	.ext_info = ade9078_voltage_ext_info,				\
	.num_event_specs = ARRAY_SIZE(ade9078_events),			\
	.scan_index = num + 1,						\
	.indexed = 1,							\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = 32,						\
		.storagebits = 32,					\
		.shift = 0,						\
		.endianness = IIO_BE,					\
	},								\
}

#define ADE9078_CURRENT_RMS_CHANNEL(num, name) {			\
	.type = IIO_CURRENT,						\
	.channel = num,							\
	.address = ADE9078_ADDR_ADJUST(ADE9078_REG_AIRMS, num),		\
	.extend_name = name "_rms",					\
	.indexed = 1,							\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_SCALE) |		\
			      BIT(IIO_CHAN_INFO_OFFSET),		\
	.scan_index = -1						\
}

#define ADE9078_VOLTAGE_RMS_CHANNEL(num, name) {			\
	.type = IIO_VOLTAGE,						\
	.channel = num,							\
	.address = ADE9078_ADDR_ADJUST(ADE9078_REG_AVRMS, num),		\
	.extend_name = name "_rms",					\
	.indexed = 1,							\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_SCALE) |		\
			      BIT(IIO_CHAN_INFO_OFFSET),		\
	.scan_index = -1						\
}

#define ADE9078_POWER_ACTIV_CHANNEL(num, name) {			\
	.type = IIO_POWER,						\
	.channel = num,							\
	.address = ADE9078_ADDR_ADJUST(ADE9078_REG_AWATT, num),		\
	.extend_name = name "_active",					\
	.indexed = 1,							\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_SCALE) |		\
			      BIT(IIO_CHAN_INFO_OFFSET) |		\
			      BIT(IIO_CHAN_INFO_HARDWAREGAIN),		\
	.scan_index = -1						\
}

#define ADE9078_POWER_REACTIV_CHANNEL(num, name) {			\
	.type = IIO_POWER,						\
	.channel = num,							\
	.address = ADE9078_ADDR_ADJUST(ADE9078_REG_AVAR, num),		\
	.extend_name = name "_reactive",				\
	.indexed = 1,							\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_SCALE) |		\
			      BIT(IIO_CHAN_INFO_OFFSET),		\
	.scan_index = -1						\
}

#define ADE9078_POWER_APPARENT_CHANNEL(num, name) {			\
	.type = IIO_POWER,						\
	.channel = num,							\
	.address = ADE9078_ADDR_ADJUST(ADE9078_REG_AVA, num),		\
	.extend_name = name "_apparent",				\
	.indexed = 1,							\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_SCALE),			\
	.scan_index = -1						\
}

#define ADE9078_POWER_FUND_REACTIV_CHANNEL(num, name) {			\
	.type = IIO_POWER,						\
	.channel = num,							\
	.address = ADE9078_ADDR_ADJUST(ADE9078_REG_AFVAR, num),		\
	.extend_name = name "_fund_reactive",				\
	.indexed = 1,							\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_SCALE) |		\
			      BIT(IIO_CHAN_INFO_OFFSET),		\
	.scan_index = -1						\
}

#define ADE9078_POWER_FACTOR_CHANNEL(num, name) {			\
	.type = IIO_POWER,						\
	.channel = num,							\
	.address = ADE9078_ADDR_ADJUST(ADE9078_REG_APF, num),		\
	.extend_name = name "_factor",					\
	.indexed = 1,							\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_SCALE),			\
	.scan_index = -1						\
}

 #define ADE9078_ENERGY_ACTIVE_CHANNEL(num, name, addr) {     \
	.type = IIO_ENERGY,                                          \
	.channel = num,                                              \
    .address = addr,                                             \
    .extend_name = name "_active",                            \
	.indexed = 1,                                                \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),          \
	.scan_index = -1                                             \
}

#define ADE9078_ENERGY_APPARENT_CHANNEL(num, name, addr) {     \
	.type = IIO_ENERGY,                                          \
	.channel = num,                                              \
	.address = addr,                                             \
	.extend_name = name "_apparent",                            \
	.indexed = 1,                                                \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),          \
	.scan_index = -1                                             \
}

#define ADE9078_ENERGY_REACTIVE_CHANNEL(num, name, addr) {     \
	.type = IIO_ENERGY,                                          \
	.channel = num,                                              \
	.address = addr,                                             \
	.extend_name = name "_reactive",                            \
	.indexed = 1,                                                \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),          \
	.scan_index = -1                                             \
}

 #define ADE9078_ENERGY_ACTIVE_ACCUM_CHANNEL(num, name, addr) {     \
	.type = IIO_ENERGY,                                          \
	.channel = num,                                              \
    .address = addr,                                             \
    .extend_name = name "_active_accum",                            \
	.indexed = 1,                                                \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),          \
	.scan_index = -1                                             \
}

#define ADE9078_ENERGY_APPARENT_ACCUM_CHANNEL(num, name, addr) {     \
	.type = IIO_ENERGY,                                          \
	.channel = num,                                              \
	.address = addr,                                             \
	.extend_name = name "_apparent_accum",                            \
	.indexed = 1,                                                \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),          \
	.scan_index = -1                                             \
}

#define ADE9078_ENERGY_REACTIVE_ACCUM_CHANNEL(num, name, addr) {     \
	.type = IIO_ENERGY,                                          \
	.channel = num,                                              \
	.address = addr,                                             \
	.extend_name = name "_reactive_accum",                          \
	.indexed = 1,                                                \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),          \
	.scan_index = -1                                             \
}

#define ADE9078_FREQ_CHANNEL(num, name, addr) {			\
	.type = IIO_VOLTAGE,						\
	.channel = num,							\
	.extend_name = name "_freq",					\
	.address = addr,						\
	.indexed = 1,							\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.scan_index = -1						\
}

#define ADE9078_SWELL_CHANNEL(num, name) {				\
	.type = IIO_VOLTAGE,						\
	.channel = num,							\
	.address = (ADE9078_REG_SWELLA + (num / 2)),				\
	.extend_name = name "_swell",					\
	.ext_info = ade9078_swell_lvl_ext_info,				\
	.indexed = 1,							\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
	.scan_index = -1						\
}

#define ADE9078_DIP_CHANNEL(num, name) {				\
	.type = IIO_VOLTAGE,						\
	.channel = num,							\
	.address = (ADE9078_REG_DIPA + (num / 2)),				\
	.extend_name = name "_dip",					\
	.ext_info = ade9078_dip_lvl_ext_info,				\
	.indexed = 1,							\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
	.scan_index = -1						\
}

/* IIO channels of the ade9078 for each phase individually */
static const struct iio_chan_spec ade9078_a_channels[] = {
	ADE9078_CURRENT_CHANNEL(ADE9078_PHASE_A_NR, "A"),
	ADE9078_VOLTAGE_CHANNEL(ADE9078_PHASE_A_NR, "A"),
	ADE9078_CURRENT_RMS_CHANNEL(ADE9078_PHASE_A_NR, "A"),
	ADE9078_VOLTAGE_RMS_CHANNEL(ADE9078_PHASE_A_NR, "A"),
	ADE9078_POWER_ACTIV_CHANNEL(ADE9078_PHASE_A_NR, "A"),
	ADE9078_POWER_REACTIV_CHANNEL(ADE9078_PHASE_A_NR, "A"),
	ADE9078_POWER_APPARENT_CHANNEL(ADE9078_PHASE_A_NR, "A"),
	ADE9078_POWER_FUND_REACTIV_CHANNEL(ADE9078_PHASE_A_NR, "A"),
	ADE9078_POWER_FACTOR_CHANNEL(ADE9078_PHASE_A_NR, "A"),
	ADE9078_SWELL_CHANNEL(ADE9078_PHASE_A_NR, "A"),
	ADE9078_DIP_CHANNEL(ADE9078_PHASE_A_NR, "A"),
	ADE9078_ENERGY_ACTIVE_CHANNEL(ADE9078_PHASE_A_NR, "A", ADE9078_REG_AWATTHR_LO),
	ADE9078_ENERGY_APPARENT_CHANNEL(ADE9078_PHASE_A_NR, "A", ADE9078_REG_AVAHR_LO),
	ADE9078_ENERGY_REACTIVE_CHANNEL(ADE9078_PHASE_A_NR, "A", ADE9078_REG_AFVARHR_LO),
	ADE9078_ENERGY_ACTIVE_ACCUM_CHANNEL(ADE9078_PHASE_A_NR, "A", ADE9078_REG_AWATTHR_LO),
	ADE9078_ENERGY_APPARENT_ACCUM_CHANNEL(ADE9078_PHASE_A_NR, "A", ADE9078_REG_AVAHR_LO),
	ADE9078_ENERGY_REACTIVE_ACCUM_CHANNEL(ADE9078_PHASE_A_NR, "A", ADE9078_REG_AFVARHR_LO),
	ADE9078_FREQ_CHANNEL(ADE9078_PHASE_A_NR, "A", ADE9078_REG_APERIOD),
};

static const struct iio_chan_spec ade9078_b_channels[] = {
	ADE9078_CURRENT_CHANNEL(ADE9078_PHASE_B_NR, "B"),
	ADE9078_VOLTAGE_CHANNEL(ADE9078_PHASE_B_NR, "B"),
	ADE9078_CURRENT_RMS_CHANNEL(ADE9078_PHASE_B_NR, "B"),
	ADE9078_VOLTAGE_RMS_CHANNEL(ADE9078_PHASE_B_NR, "B"),
	ADE9078_POWER_ACTIV_CHANNEL(ADE9078_PHASE_B_NR, "B"),
	ADE9078_POWER_REACTIV_CHANNEL(ADE9078_PHASE_B_NR, "B"),
	ADE9078_POWER_APPARENT_CHANNEL(ADE9078_PHASE_B_NR, "B"),
	ADE9078_POWER_FUND_REACTIV_CHANNEL(ADE9078_PHASE_B_NR, "B"),
	ADE9078_POWER_FACTOR_CHANNEL(ADE9078_PHASE_B_NR, "B"),
	ADE9078_SWELL_CHANNEL(ADE9078_PHASE_B_NR, "B"),
	ADE9078_DIP_CHANNEL(ADE9078_PHASE_B_NR, "B"),
	ADE9078_ENERGY_ACTIVE_CHANNEL(ADE9078_PHASE_B_NR, "B", ADE9078_REG_BWATTHR_LO),
	ADE9078_ENERGY_APPARENT_CHANNEL(ADE9078_PHASE_B_NR, "B", ADE9078_REG_BVAHR_LO),
	ADE9078_ENERGY_REACTIVE_CHANNEL(ADE9078_PHASE_B_NR, "B", ADE9078_REG_BFVARHR_LO),
	ADE9078_ENERGY_ACTIVE_ACCUM_CHANNEL(ADE9078_PHASE_B_NR, "B", ADE9078_REG_BWATTHR_LO),
	ADE9078_ENERGY_APPARENT_ACCUM_CHANNEL(ADE9078_PHASE_B_NR, "B", ADE9078_REG_BVAHR_LO),
	ADE9078_ENERGY_REACTIVE_ACCUM_CHANNEL(ADE9078_PHASE_B_NR, "B", ADE9078_REG_BFVARHR_LO),
	ADE9078_FREQ_CHANNEL(ADE9078_PHASE_B_NR, "B", ADE9078_REG_BPERIOD),
};

static const struct iio_chan_spec ade9078_c_channels[] = {
	ADE9078_CURRENT_CHANNEL(ADE9078_PHASE_C_NR, "C"),
	ADE9078_VOLTAGE_CHANNEL(ADE9078_PHASE_C_NR, "C"),
	ADE9078_CURRENT_RMS_CHANNEL(ADE9078_PHASE_C_NR, "C"),
	ADE9078_VOLTAGE_RMS_CHANNEL(ADE9078_PHASE_C_NR, "C"),
	ADE9078_POWER_ACTIV_CHANNEL(ADE9078_PHASE_C_NR, "C"),
	ADE9078_POWER_REACTIV_CHANNEL(ADE9078_PHASE_C_NR, "C"),
	ADE9078_POWER_APPARENT_CHANNEL(ADE9078_PHASE_C_NR, "C"),
	ADE9078_POWER_FUND_REACTIV_CHANNEL(ADE9078_PHASE_C_NR, "C"),
	ADE9078_POWER_FACTOR_CHANNEL(ADE9078_PHASE_C_NR, "C"),
	ADE9078_SWELL_CHANNEL(ADE9078_PHASE_C_NR, "C"),
	ADE9078_DIP_CHANNEL(ADE9078_PHASE_C_NR, "C"),
	ADE9078_ENERGY_ACTIVE_CHANNEL(ADE9078_PHASE_C_NR, "C", ADE9078_REG_CWATTHR_LO),
	ADE9078_ENERGY_APPARENT_CHANNEL(ADE9078_PHASE_C_NR, "C", ADE9078_REG_CVAHR_LO),
	ADE9078_ENERGY_REACTIVE_CHANNEL(ADE9078_PHASE_C_NR, "C", ADE9078_REG_CFVARHR_LO),
	ADE9078_ENERGY_ACTIVE_ACCUM_CHANNEL(ADE9078_PHASE_C_NR, "C", ADE9078_REG_CWATTHR_LO),
	ADE9078_ENERGY_APPARENT_ACCUM_CHANNEL(ADE9078_PHASE_C_NR, "C", ADE9078_REG_CVAHR_LO),
	ADE9078_ENERGY_REACTIVE_ACCUM_CHANNEL(ADE9078_PHASE_C_NR, "C", ADE9078_REG_CFVARHR_LO),
	ADE9078_FREQ_CHANNEL(ADE9078_PHASE_C_NR, "C", ADE9078_REG_CPERIOD),
};

static struct reg_sequence ade9078_reg_sequence[] = {
	{ ADE9078_REG_PGA_GAIN, ADE9078_PGA_GAIN },
	{ ADE9078_REG_CONFIG0, ADE9078_CONFIG0 },
	{ ADE9078_REG_CONFIG1, ADE9078_CONFIG1 },
	{ ADE9078_REG_CONFIG2, ADE9078_CONFIG2 },
	{ ADE9078_REG_CONFIG3, ADE9078_CONFIG3 },
	{ ADE9078_REG_ACCMODE, ADE9078_ACCMODE },
	{ ADE9078_REG_ZX_LP_SEL, ADE9078_ZX_LP_SEL },
	{ ADE9078_REG_MASK0, ADE9078_MASK0 },
	{ ADE9078_REG_MASK1, ADE9078_MASK1 },
	{ ADE9078_REG_EVENT_MASK, ADE9078_EVENT_MASK },
	{ ADE9078_REG_WFB_CFG, ADE9078_WFB_CFG },
	{ ADE9078_REG_VLEVEL, ADE9078_VLEVEL },
	{ ADE9078_REG_DICOEFF, ADE9078_DICOEFF },
	{ ADE9078_REG_EGY_TIME, ADE9078_EGY_TIME },
	{ ADE9078_REG_EP_CFG, ADE9078_EP_CFG },
	{ ADE9078_REG_RUN, ADE9078_RUN_ON }
};

//TODO As mentioned below, I don't immediately understand why this can't
//be done with appropriate standard regmap.  Perhaps you could give
//more details of what is missing.  I'd like to see that added to
//regmap if possible.

/**
 * ade9078_spi_write_reg() - ADE9078 write register over SPI
 * @context:	void pointer to the SPI device
 * @reg:	address of the of desired register
 * @val:	value to be written to the ade9078
 *
 *
 * The data format for communicating with the ade9078 over SPI
 * is very specific and can access both 32bit and 16bit registers
 */
static int ade9078_spi_write_reg(void *context,
				 unsigned int reg,
				 unsigned int val)
{
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);
	struct ade9078_state *st = spi_get_drvdata(spi);

	u16 addr;
	int ret = 0;
	struct spi_transfer xfer[] = {
		{
			.tx_buf = st->tx,
		},
	};

	addr = FIELD_PREP(ADE9078_REG_ADDR_MASK, reg);

	put_unaligned_be16(addr, st->tx);
	put_unaligned_be32(val, &st->tx[2]);

	if (reg > ADE9078_REG_RUN && reg < ADE9078_REG_VERSION) {
		put_unaligned_be16(val, &st->tx[2]);
		xfer[0].len = 4;
	} else {
		xfer[0].len = 6;
	}

	ret = spi_sync_transfer(st->spi, xfer, ARRAY_SIZE(xfer));
	if (ret) {
		dev_err(&st->spi->dev, "problem when writing register 0x%x",
			reg);
	}

	return ret;
}

/**
 * ade9078_spi_write_reg() - ADE9078 read register over SPI
 * @context:	void pointer to the SPI device
 * @reg:	address of the of desired register
 * @val:	value to be read to the ade9078
 *
 * The data format for communicating with the ade9078 over SPI
 * is very specific and can access both 32bit and 16bit registers
 */
static int ade9078_spi_read_reg(void *context,
				unsigned int reg,
				unsigned int *val)
{
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);
	struct ade9078_state *st = spi_get_drvdata(spi);

	u16 addr;
	int ret = 0;
	struct spi_transfer xfer[] = {
		{
			.tx_buf = st->tx,
			.len = 2,
		},
		{
			.rx_buf = st->rx,
		},
	};

	addr = FIELD_PREP(ADE9078_REG_ADDR_MASK, reg) |
	       ADE9078_REG_READ_BIT_MASK;

	put_unaligned_be16(addr, st->tx);

	if (reg > ADE9078_REG_RUN && reg < ADE9078_REG_VERSION)
		xfer[1].len = 4;
	else
		xfer[1].len = 6;

// TODO This doesn't look like a fixed length register which is expected
//for regmap...  Also that len should just be the rx register which
//you treat below as 16 bit and 32 bit (so 2 and 4, not 4 and 6).
//
//Can the larger registers be treated as bulk reads of pairs of smaller ones?
	ret = spi_sync_transfer(st->spi, xfer, ARRAY_SIZE(xfer));
	if (ret) {
		dev_err(&st->spi->dev, "problem when reading register 0x%x",
			reg);
		goto err_ret;
	}

	//registers which are 16 bits
	if (reg > 0x480 && reg < 0x4FE)
		*val = get_unaligned_be16(st->rx);
	else
		*val = get_unaligned_be32(st->rx);

err_ret:
	return ret;
}

/**
 * ade9078_is_volatile_reg() - List of ade9078 registers which should use
 * caching
 * @dev:	device data
 * @reg:	address of the of desired register
 */
static bool ade9078_is_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case ADE9078_REG_STATUS0:
	case ADE9078_REG_STATUS1:
	case ADE9078_REG_MASK0:
	case ADE9078_REG_MASK1:
	case ADE9078_REG_WFB_PG_IRQEN:
		return false;
	default:
		return true;
	}
}

/**
 * ade9078_configure_scan() - Sets up the transfer parameters
 * as well as the tx and rx buffers
 * @indio_dev:	the IIO device
 * @wfb_addr:	buffer address to read from
 *
 * In case the waveform buffer is set to streaming mode, only half of the buffer
 * will be read out at a time
 */
static int ade9078_configure_scan(struct iio_dev *indio_dev, u32 wfb_addr)
{
	struct ade9078_state *st = iio_priv(indio_dev);
	u16 addr;

	addr = FIELD_PREP(ADE9078_REG_ADDR_MASK, wfb_addr) |
	       ADE9078_REG_READ_BIT_MASK;

	put_unaligned_be16(addr, st->tx_buff);

	st->xfer[0].tx_buf = &st->tx_buff[0];
	st->xfer[0].len = 2;

	st->xfer[1].rx_buf = &st->rx_buff.byte[0];

	if (st->wf_mode == ADE9078_WFB_STREAMING_MODE)
		st->xfer[1].len = (st->wfb_nr_samples / 2) * 4;
	else
		st->xfer[1].len = st->wfb_nr_samples * 4;

	spi_message_init_with_transfers(&st->spi_msg, st->xfer, 2);
	return 0;
}

/**
 * ade9078_iio_push_streaming() - designed for reading the FIFO in continuous
 * mode.
 * @indio_dev:	the IIO device
 *
 * This function implements a continuous streaming mode, of the internal FIFO.
 * To maintain continuity, only half of the FIFO is read out any given time
 * while the other half fills up and is pushed to the IIO buffer. Thus after
 * each iio_push_to_buffer the interrupt is set to either the last page or the
 * middle page and the read address of the FIFO to either the beginning of the
 * first page of to the beginning of page 8 (middle page + 1)
 */
static int ade9078_iio_push_streaming(struct iio_dev *indio_dev)
{
	struct ade9078_state *st = iio_priv(indio_dev);
	u32 current_page;
	int ret;
	u32 i;

	ret = spi_sync(st->spi, &st->spi_msg);
	if (ret) {
		dev_err(&st->spi->dev, "SPI fail in trigger handler");
		return ret;
	}

	for (i = 0; i < st->wfb_nr_samples / 2; i += st->wfb_nr_activ_chan)
		iio_push_to_buffers(indio_dev, &st->rx_buff.word[i]);

	ret = regmap_read(st->regmap, ADE9078_REG_WFB_PG_IRQEN, &current_page);
	if (ret) {
		dev_err(&st->spi->dev, "IRQ0 WFB read fail");
		return ret;
	}

	//switch pages to be read and which interrupt to occur
	if (current_page & ADE9078_MIDDLE_PAGE_BIT) {
		ret = regmap_write(st->regmap, ADE9078_REG_WFB_PG_IRQEN,
				   ADE9078_LAST_PAGE_BIT);
		if (ret) {
			dev_err(&st->spi->dev, "IRQ0 WFB write fail");
			return ret;
		}

		ret = ade9078_configure_scan(indio_dev,
					     ADE9078_REG_WF_HALF_BUFF);
		if (ret)
			return ret;
	} else {
		ret = regmap_write(st->regmap, ADE9078_REG_WFB_PG_IRQEN,
				   ADE9078_MIDDLE_PAGE_BIT);
		if (ret) {
			dev_err(&st->spi->dev,
				"IRQ0 WFB write fail");
			return IRQ_HANDLED;
		}

		ret = ade9078_configure_scan(indio_dev,
					     ADE9078_REG_WF_BUFF);
		if (ret)
			return ret;
	}

	return 0;
}

/**
 * ade9078_iio_push_buffer() - reads out the content of the waveform buffer and
 * pushes it to the IIO buffer.
 * @indio_dev:	the IIO device
 */
static int ade9078_iio_push_buffer(struct iio_dev *indio_dev)
{
	struct ade9078_state *st = iio_priv(indio_dev);
	int ret;
	u32 i;

	ret = spi_sync(st->spi, &st->spi_msg);
	if (ret) {
		dev_err(&st->spi->dev, "SPI fail in trigger handler");
		return ret;
	}

	for (i = 0; i < st->wfb_nr_samples; i += st->wfb_nr_activ_chan)
		iio_push_to_buffers(indio_dev, &st->rx_buff.word[i]);

	return 0;
}

/**
 * ade9078_en_wfb() - enables or disables the WFBuffer in the ADE9078
 * @st:		ade9078 device data
 * @state:	true for enabled; false for disabled
 */
static int ade9078_en_wfb(struct ade9078_state *st, bool state)
{
	return regmap_update_bits(st->regmap, ADE9078_REG_WFB_CFG, BIT(4),
				  state ? BIT(4) : 0);
}

/**
 * ade9078_irq0_thread() - Thread for IRQ0.
 * @irq:	interrupt
 * @data:	private callback data passed trough the interrupt.
 *
 * It reads Status register 0 and checks for the IRQ activation. This is
 * configured to acquire samples in to the IC buffer and dump it in to the
 * iio_buffer according to Stop When Buffer Is Full Mode, Stop Filling on
 * Trigger and Capture Around Trigger from the ADE9078 Datasheet
 */
static irqreturn_t ade9078_irq0_thread(int irq, void *data)
{
	struct iio_dev *indio_dev = data;
	struct ade9078_state *st = iio_priv(indio_dev);
	u32 handled_irq = 0;
	u32 interrupts;
	u32 status;
	int ret;

	ret = regmap_read(st->regmap, ADE9078_REG_STATUS0, &status);
	if (ret) {
		dev_err(&st->spi->dev, "IRQ0 read status fail");
		return IRQ_HANDLED;
	}

	ret = regmap_read(st->regmap, ADE9078_REG_MASK0, &interrupts);
	if (ret) {
		dev_err(&st->spi->dev, "IRQ0 read status fail");
		return IRQ_HANDLED;
	}

	if ((status & ADE9078_ST0_PAGE_FULL_BIT) &&
	    (interrupts & ADE9078_ST0_PAGE_FULL_BIT)) {
		switch (st->wf_mode) {
		case ADE9078_WFB_FULL_MODE:
			ret = ade9078_en_wfb(st, false);
			if (ret) {
				dev_err(&st->spi->dev, "IRQ0 WFB stop fail");
				return IRQ_HANDLED;
			}
			ret = ade9078_iio_push_buffer(indio_dev);
			if (ret) {
				dev_err(&st->spi->dev, "IRQ0 IIO push fail");
				return IRQ_HANDLED;
			}

			interrupts &= ~ADE9078_ST0_PAGE_FULL_BIT;

			ret = regmap_write(st->regmap, ADE9078_REG_MASK0,
					   interrupts);
			if (ret) {
				dev_err(&st->spi->dev, "IRQ0 MAKS0 write fail");
				return IRQ_HANDLED;
			}
			break;

		case ADE9078_WFB_C_EN_TRIG_MODE:
			ret = regmap_write(st->regmap, ADE9078_REG_WFB_PG_IRQEN,
					   ADE9078_LAST_PAGE_BIT);
			if (ret) {
				dev_err(&st->spi->dev, "IRQ0 WFB write fail");
				return IRQ_HANDLED;
			}

			ret = regmap_write(st->regmap, ADE9078_REG_WFB_TRG_CFG,
					   st->wfb_trg);
			if (ret) {
				dev_err(&st->spi->dev, "IRQ0 WFB write fail");
				return IRQ_HANDLED;
			}

			interrupts |= ADE9078_ST0_WFB_TRIG_BIT;
			interrupts &= ~ADE9078_ST0_PAGE_FULL_BIT;

			ret = regmap_write(st->regmap, ADE9078_REG_MASK0,
					   interrupts);
			if (ret) {
				dev_err(&st->spi->dev, "IRQ0 MAKS0 write fail");
				return IRQ_HANDLED;
			}
			break;

		case ADE9078_WFB_EN_TRIG_MODE:
			ret = regmap_write(st->regmap, ADE9078_REG_WFB_TRG_CFG,
					   st->wfb_trg);
			if (ret) {
				dev_err(&st->spi->dev, "IRQ0 WFB write fail");
				return IRQ_HANDLED;
			}

			interrupts |= ADE9078_ST0_WFB_TRIG_BIT;

			interrupts &= ~ADE9078_ST0_PAGE_FULL_BIT;

			ret = regmap_write(st->regmap, ADE9078_REG_MASK0,
					   interrupts);
			if (ret) {
				dev_err(&st->spi->dev, "IRQ0 MAKS0 write fail");
				return IRQ_HANDLED;
			}
			break;

		case ADE9078_WFB_STREAMING_MODE:
			ret = ade9078_iio_push_streaming(indio_dev);
			if (ret) {
				dev_err(&st->spi->dev, "IRQ0 IIO push fail");
				return IRQ_HANDLED;
			}
			break;

		default:
			return IRQ_HANDLED;
		}

		handled_irq |= ADE9078_ST0_PAGE_FULL_BIT;
	}

	if ((status & ADE9078_ST0_WFB_TRIG_BIT) &&
	    (interrupts & ADE9078_ST0_WFB_TRIG_BIT)) {
		//Stop Filling on Trigger and Center Capture Around Trigger
		ret = ade9078_en_wfb(st, false);
		if (ret) {
			dev_err(&st->spi->dev, "IRQ0 WFB fail");
			return IRQ_HANDLED;
		}

		ret = ade9078_iio_push_buffer(indio_dev);
		if (ret) {
			dev_err(&st->spi->dev, "IRQ0 IIO push fail @ WFB TRIG");
			return IRQ_HANDLED;
		}

		handled_irq |= ADE9078_ST0_WFB_TRIG_BIT;
	}

	if ((status & ADE9078_ST0_EGYRDY) &&
	    (interrupts & ADE9078_ST0_EGYRDY)) {
		u32 data;
		/* As per datasheet, reading the _HI registers is enough. */

		ret = regmap_read(st->regmap, (ADE9078_REG_AWATTHR_LO + 1), &data);
		if (ret)
			return ret;

		st->egy_active_accum[0] = data;

		ret = regmap_read(st->regmap, (ADE9078_REG_BWATTHR_LO + 1), &data);
		if (ret)
			return ret;

		st->egy_active_accum[1] = data;

		ret = regmap_read(st->regmap, (ADE9078_REG_CWATTHR_LO + 1), &data);
		if (ret)
			return ret;

		st->egy_active_accum[2] = data;

		ret = regmap_read(st->regmap, (ADE9078_REG_AVAHR_LO + 1), &data);
		if (ret)
			return ret;

		st->egy_apparent_accum[0] = data;

		ret = regmap_read(st->regmap, (ADE9078_REG_BVAHR_LO + 1), &data);
		if (ret)
			return ret;

		st->egy_apparent_accum[1] = data;

		ret = regmap_read(st->regmap, (ADE9078_REG_CVAHR_LO + 1), &data);
		if (ret)
			return ret;

		st->egy_apparent_accum[2] = data;

		ret = regmap_read(st->regmap, (ADE9078_REG_AFVARHR_LO + 1), &data);
		if (ret)
			return ret;

		st->egy_reactive_accum[0] = data;

		ret = regmap_read(st->regmap, (ADE9078_REG_BFVARHR_LO + 1), &data);
		if (ret)
			return ret;

		st->egy_reactive_accum[1] = data;

		ret = regmap_read(st->regmap, (ADE9078_REG_CFVARHR_LO + 1), &data);
		if (ret)
			return ret;

		st->egy_reactive_accum[2] = data;

		handled_irq |= ADE9078_ST0_EGYRDY;
	}

	ret = regmap_write(st->regmap, ADE9078_REG_STATUS0, handled_irq);
	if (ret)
		dev_err(&st->spi->dev, "IRQ0 write status fail");

	return IRQ_HANDLED;
}

/**
 * ade9078_irq1_thread() - Thread for IRQ1.
 * @irq:	interrupt
 * @data:	private callback data passed trough the interrupt.
 *
 * It reads Status register 1 and checks for the IRQ activation. This thread
 * handles the reset condition and the zero-crossing conditions for all 3 phases
 * on Voltage and Current
 */
static irqreturn_t ade9078_irq1_thread(int irq, void *data)
{
	struct iio_dev *indio_dev = data;
	struct ade9078_state *st = iio_priv(indio_dev);
	struct iio_chan_spec const *chan = indio_dev->channels;
	unsigned int bit = ADE9078_ST1_CROSSING_FIRST;
	s64 timestamp = iio_get_time_ns(indio_dev);
	u32 interrupts;
	u32 result;
	u32 status;
	u32 tmp;
	int ret;

	if (!st->rst_done) {
		ret = regmap_read(st->regmap, ADE9078_REG_STATUS1, &result);
		if (ret)
			return ret;

		if (result & ADE9078_ST1_RSTDONE_BIT)
			st->rst_done = true;
		else
			dev_err(&st->spi->dev, "Error testing reset done");

		return IRQ_HANDLED;
	}

	ret = regmap_read(st->regmap, ADE9078_REG_STATUS1, &status);
	if (ret)
		return IRQ_HANDLED;

	ret = regmap_read(st->regmap, ADE9078_REG_MASK1, &interrupts);
	if (ret) {
		dev_err(&st->spi->dev, "IRQ1 read status fail");
		return IRQ_HANDLED;
	}

	// if ((status & ADE9078_ST1_SEQERR_BIT) && (interrupts & ADE9078_ST1_SEQERR_BIT)) {
	// 	       iio_push_event(indio_dev,
	// 	       IIO_UNMOD_EVENT_CODE(chan->type,
	// 				    0,
	// 				    IIO_EV_TYPE_THRESH,
	// 				    IIO_EV_DIR_EITHER),
	// 	       timestamp);
	// }

	for_each_set_bit_from(bit, (unsigned long *)&interrupts,
			      ADE9078_ST1_CROSSING_DEPTH){
		tmp = status & BIT(bit);

		switch (tmp) {
		case ADE9078_ST1_ZXVA_BIT:
		case ADE9078_ST1_ZXTOVA_BIT:
		case ADE9078_ST1_ZXIA_BIT:
			iio_push_event(indio_dev,
				       IIO_UNMOD_EVENT_CODE(chan->type,
							    ADE9078_PHASE_A_NR,
							    IIO_EV_TYPE_THRESH,
							    IIO_EV_DIR_EITHER),
				       timestamp);
			break;
		case ADE9078_ST1_ZXVB_BIT:
		case ADE9078_ST1_ZXTOVB_BIT:
		case ADE9078_ST1_ZXIB_BIT:
			iio_push_event(indio_dev,
				       IIO_UNMOD_EVENT_CODE(chan->type,
							    ADE9078_PHASE_B_NR,
							    IIO_EV_TYPE_THRESH,
							    IIO_EV_DIR_EITHER),
				       timestamp);
			break;
		case ADE9078_ST1_ZXVC_BIT:
		case ADE9078_ST1_ZXTOVC_BIT:
		case ADE9078_ST1_ZXIC_BIT:
			iio_push_event(indio_dev,
				       IIO_UNMOD_EVENT_CODE(chan->type,
							    ADE9078_PHASE_C_NR,
							    IIO_EV_TYPE_THRESH,
							    IIO_EV_DIR_EITHER),
				       timestamp);
			break;
		case ADE9078_ST1_SWELLA_BIT:
			iio_push_event(indio_dev,
					IIO_UNMOD_EVENT_CODE(chan->type,
							ADE9078_PHASE_A_NR,
							IIO_EV_TYPE_THRESH,
							IIO_EV_DIR_RISING),
					timestamp);
			break;
		case ADE9078_ST1_SWELLB_BIT:
			iio_push_event(indio_dev,
					IIO_UNMOD_EVENT_CODE(chan->type,
							ADE9078_PHASE_B_NR,
							IIO_EV_TYPE_THRESH,
							IIO_EV_DIR_RISING),
					timestamp);
			break;
		case ADE9078_ST1_SWELLC_BIT:
			iio_push_event(indio_dev,
					IIO_UNMOD_EVENT_CODE(chan->type,
							ADE9078_PHASE_C_NR,
							IIO_EV_TYPE_THRESH,
							IIO_EV_DIR_RISING),
					timestamp);
			break;
		case ADE9078_ST1_DIPA_BIT:
			iio_push_event(indio_dev,
					IIO_UNMOD_EVENT_CODE(chan->type,
							ADE9078_PHASE_A_NR,
							IIO_EV_TYPE_THRESH,
							IIO_EV_DIR_FALLING),
					timestamp);
			break;
		case ADE9078_ST1_DIPB_BIT:
			iio_push_event(indio_dev,
					IIO_UNMOD_EVENT_CODE(chan->type,
							ADE9078_PHASE_B_NR,
							IIO_EV_TYPE_THRESH,
							IIO_EV_DIR_FALLING),
					timestamp);
			break;
		case ADE9078_ST1_DIPC_BIT:
			iio_push_event(indio_dev,
					IIO_UNMOD_EVENT_CODE(chan->type,
							ADE9078_PHASE_C_NR,
							IIO_EV_TYPE_THRESH,
							IIO_EV_DIR_FALLING),
					timestamp);
			break;
		case ADE9078_ST1_SEQERR_BIT:
			iio_push_event(indio_dev,
					IIO_UNMOD_EVENT_CODE(chan->type,
							0,
							IIO_EV_TYPE_THRESH,
							IIO_EV_DIR_EITHER),
					timestamp);
			break;
		default:
			return IRQ_HANDLED;
		}
	}

	return IRQ_HANDLED;
}

/**
 * ade9078_read_raw() - IIO read function
 * @indio_dev:	the IIO device
 * @chan:	channel specs of the ade9078
 * @val:	first half of the read value
 * @val2:	second half of the read value
 * @mask:	info mask of the channel
 */
static int ade9078_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val,
			    int *val2,
			    long mask)
{
	struct ade9078_state *st = iio_priv(indio_dev);
	unsigned int reg;
	int measured;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = regmap_read(st->regmap, ADE9078_REG_ACCMODE, &reg);
		if (ret)
			return ret;
		*val = (reg & BIT(8)) ? 60 : 50;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_RAW:
     		dev_info(&st->spi->dev, "read_raw: type=%d, extend_name=%s\n", chan->type,
       			 chan->extend_name ? chan->extend_name : "NULL");     
        	// Check for frequency channel hack (using VOLTAGE type and "_freq" suffix)
        	if (chan->type == IIO_VOLTAGE && chan->extend_name &&
			strstr(chan->extend_name, "_freq")) {
			int period;
			ret = regmap_read(st->regmap, chan->address, &period);
			if (ret)
				return ret;
			*val = 4000 * 65536;  // Numerator
			*val2 = period + 1;   // Denominator
            		return IIO_VAL_FRACTIONAL;
        	}
		// Check for ENERGY_ACCUM channel (mapped to _raw attribute)
		if (chan->type == IIO_ENERGY && chan->extend_name &&
			strstr(chan->extend_name, "_active_accum")) {
			*val = st->egy_active_accum[chan->channel / 2];
			return IIO_VAL_INT;
		}

		if (chan->type == IIO_ENERGY && chan->extend_name &&
			strstr(chan->extend_name, "_apparent_accum")) {
			*val = st->egy_apparent_accum[chan->channel / 2];
			return IIO_VAL_INT;
		}

		if (chan->type == IIO_ENERGY && chan->extend_name &&
			strstr(chan->extend_name, "_reactive_accum")) {
			*val = st->egy_reactive_accum[chan->channel / 2];
			return IIO_VAL_INT;
		}

		// Check for ENERGY channel (mapped to _raw attribute)
		if (chan->type == IIO_ENERGY) {
			s64 val64;
			u32 data[2];
			u16 lo_reg = chan->address;
			ret = regmap_bulk_read(st->regmap, lo_reg, data, 2);
			if (ret)
				return ret;
		
			val64 = ((u64)data[1] << 32) | data[0];
			*(s64 *)val = val64;
			return IIO_VAL_INT;
		}
	
		// Default: claim and read simple 32-bit register     
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
		return ret;
	
		ret = regmap_read(st->regmap, chan->address, &measured);
		if (ret)
		return ret;
	
		iio_device_release_direct_mode(indio_dev);
		*val = measured;
	
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		if (chan->type == IIO_CURRENT || chan->type == IIO_VOLTAGE) {
			switch (chan->address) {
			case ADE9078_REG_AI_PCF:
			case ADE9078_REG_AV_PCF:
			case ADE9078_REG_BI_PCF:
			case ADE9078_REG_BV_PCF:
			case ADE9078_REG_CI_PCF:
			case ADE9078_REG_CV_PCF:
				*val = 1;
				*val2 = ADE9078_PCF_FULL_SCALE_CODES;
				return IIO_VAL_FRACTIONAL;
			case ADE9078_REG_AIRMS:
			case ADE9078_REG_AVRMS:
			case ADE9078_REG_BIRMS:
			case ADE9078_REG_BVRMS:
			case ADE9078_REG_CIRMS:
			case ADE9078_REG_CVRMS:
				*val = 1;
				*val2 = ADE9078_RMS_FULL_SCALE_CODES;
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
	case IIO_CHAN_INFO_CALIBSCALE:
		ret = regmap_read(st->regmap, ADE9078_REG_PGA_GAIN, &reg);
		if (ret)
			return ret;
		*val = 1 << ((reg >> (8 + chan->channel)) & 0x3);
		if (*val > 4)
			*val = 4;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

/**
 * ade9078_write_raw() - IIO write function
 * @indio_dev:	the IIO device
 * @chan:	channel specs of the ade9078
 * @val:	first half of the read value
 * @val2:	second half of the read value
 * @mask:	info mask of the channel
 */
static int ade9078_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val,
			     int val2,
			     long mask)
{
	struct ade9078_state *st = iio_priv(indio_dev);
	u32 addr;
	u32 tmp;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (val != 50 && val != 60)
			return -EINVAL;
		return regmap_write(st->regmap, ADE9078_REG_ACCMODE,
				    (val == 60) ? ADE9078_ACCMODE_60HZ : ADE9078_ACCMODE);
	case IIO_CHAN_INFO_OFFSET:
		switch (chan->type) {
		case IIO_CURRENT:
			addr = ADE9078_ADDR_ADJUST(ADE9078_REG_AIRMSOS,
						   chan->channel);
			break;
		case IIO_VOLTAGE:
			addr = ADE9078_ADDR_ADJUST(ADE9078_REG_AVRMSOS,
						   chan->channel);
			break;
		case IIO_POWER:
			tmp = chan->address;
			tmp &= ~ADE9078_PHASE_B_POS_BIT;
			tmp &= ~ADE9078_PHASE_C_POS_BIT;

			switch (tmp) {
			case ADE9078_REG_AWATTOS:
				addr = ADE9078_ADDR_ADJUST(ADE9078_REG_AWATTOS,
							   chan->channel);
				break;
			case ADE9078_REG_AVAR:
				addr = ADE9078_ADDR_ADJUST(ADE9078_REG_AVAROS,
							   chan->channel);
				break;
			case ADE9078_REG_AFVAR:
				addr = ADE9078_ADDR_ADJUST(ADE9078_REG_AFVAROS,
							   chan->channel);
				break;
			default:
				return -EINVAL;
			}

			break;
		default:
			return -EINVAL;
		}
		break;
	case IIO_CHAN_INFO_HARDWAREGAIN:
		switch (chan->type) {
		case IIO_CURRENT:
			addr = ADE9078_ADDR_ADJUST(ADE9078_REG_AIGAIN,
						   chan->channel);
			break;
		case IIO_VOLTAGE:
			addr = ADE9078_ADDR_ADJUST(ADE9078_REG_AVGAIN,
						   chan->channel);
			break;
		case IIO_POWER:
			addr = ADE9078_ADDR_ADJUST(ADE9078_REG_APGAIN,
						   chan->channel);
			break;
		default:
			return -EINVAL;
		}
		break;
	case IIO_CHAN_INFO_CALIBSCALE:
		if (val > 4 || val < 1 || val == 3)
			return -EINVAL;
		addr = ADE9078_REG_PGA_GAIN;
		val = ilog2(val) << (chan->channel + 8);
		tmp = 0x3 << (chan->channel + 8);
		return regmap_update_bits(st->regmap, addr, tmp, val);
	default:
		return -EINVAL;
	}

	return regmap_write(st->regmap, addr, val);
}

/**
 * ade9078_reg_access() - IIO debug register access
 * @indio_dev:	the IIO device
 * @reg:	register to be accessed
 * @tx_val:	value to be transmitted
 * @rx_val:	value to be received
 */
static int ade9078_reg_access(struct iio_dev *indio_dev,
			      unsigned int reg,
			      unsigned int tx_val,
			      unsigned int *rx_val)
{
	struct ade9078_state *st = iio_priv(indio_dev);

	if (rx_val)
		return regmap_read(st->regmap, reg, rx_val);

	return regmap_write(st->regmap, reg, tx_val);
}

/**
 * ade9078_write_event_config() - IIO event configure to enable zero-crossing.
 * @indio_dev: the device instance data
 * @chan: channel for the event whose state is being set
 * @type: type of the event whose state is being set
 * @dir: direction of the vent whose state is being set
 * @state: whether to enable or disable the device.
 *
 * This will enable zero-crossing and zero-crossing timeout on voltage and
 * current for each phases. These events will also influence the trigger
 * conditions for the buffer capture.
 */
static int ade9078_write_event_config(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan,
				      enum iio_event_type type,
				      enum iio_event_direction dir,
				      int state)
{
	struct ade9078_state *st = iio_priv(indio_dev);
	u32 interrupts, tmp;
	int ret;
	struct irq_wfb_trig {
		u32 irq;
		u32 wfb_trg;
	};

	ret = regmap_write(st->regmap, ADE9078_REG_STATUS1, GENMASK(31, 0));
	if (ret)
		return ret;

	if (chan->type == IIO_COUNT)
		return regmap_update_bits(st->regmap, ADE9078_REG_MASK1,
                        		 ADE9078_ST1_SEQERR_BIT,
                        		 state ? ADE9078_ST1_SEQERR_BIT : 0);

	struct irq_wfb_trig trig_arr[6] = {
		{.irq = ADE9078_ST1_ZXVA_BIT | ADE9078_ST1_ZXTOVA_BIT,
		 .wfb_trg = ADE9078_WFB_TRG_ZXVA_BIT
		},
		{.irq = ADE9078_ST1_ZXIA_BIT,
		 .wfb_trg = ADE9078_WFB_TRG_ZXIA_BIT
		},
		{.irq = ADE9078_ST1_ZXVB_BIT | ADE9078_ST1_ZXTOVB_BIT,
		 .wfb_trg = ADE9078_WFB_TRG_ZXVB_BIT
		},
		{.irq = ADE9078_ST1_ZXIB_BIT,
		 .wfb_trg = ADE9078_WFB_TRG_ZXIB_BIT
		},
		{.irq = ADE9078_ST1_ZXVC_BIT | ADE9078_ST1_ZXTOVC_BIT,
		 .wfb_trg = ADE9078_WFB_TRG_ZXVC_BIT
		},
		{.irq = ADE9078_ST1_ZXIC_BIT,
		 .wfb_trg = ADE9078_WFB_TRG_ZXIC_BIT
		},
	};

	if (dir == IIO_EV_DIR_EITHER) {
		if (state) {
			interrupts |= trig_arr[chan->channel + chan->type].irq;
			st->wfb_trg |= trig_arr[chan->channel + chan->type].wfb_trg;
		} else {
			interrupts &= ~trig_arr[chan->channel + chan->type].irq;
			st->wfb_trg &= ~trig_arr[chan->channel + chan->type].wfb_trg;
		}
	} else if (dir == IIO_EV_DIR_RISING) {
		switch (chan->channel){
		case ADE9078_PHASE_A_NR: 
			tmp |= ADE9078_ST1_SWELLA_BIT;
			break;
		case ADE9078_PHASE_B_NR: 
			tmp |= ADE9078_ST1_SWELLB_BIT;
			break;
		case ADE9078_PHASE_C_NR: 
			tmp |= ADE9078_ST1_SWELLC_BIT;
			break;
		default:
			break;
		}

		if (state) {
			interrupts |= tmp;
			st->wfb_trg |= ADE9078_WFB_TRG_SWELL_BIT;
		} else {
			interrupts &= ~tmp;
			st->wfb_trg &= ~ADE9078_WFB_TRG_SWELL_BIT;
		}

	} else if (dir == IIO_EV_DIR_FALLING) {
		switch (chan->channel){
		case ADE9078_PHASE_A_NR: 
			interrupts |= ADE9078_ST1_DIPA_BIT;
			break;
		case ADE9078_PHASE_B_NR: 
			interrupts |= ADE9078_ST1_DIPB_BIT;
			break;
		case ADE9078_PHASE_C_NR: 
			interrupts |= ADE9078_ST1_DIPC_BIT;
			break;
		default:
			break;
		}

		if (state) {
			interrupts |= tmp;
			st->wfb_trg |= ADE9078_WFB_TRG_DIP_BIT;
		} else {
			interrupts &= ~tmp;
			st->wfb_trg &= ~ADE9078_WFB_TRG_DIP_BIT;
		}
	}
	return regmap_update_bits(st->regmap, ADE9078_REG_MASK1, interrupts,
				  interrupts);
}

/**
 * ade9078_read_event_vlaue() - Outputs the result of the zero-crossing for
 * voltage and current for each phase.
 * @indio_dev: device instance specific data
 * @chan: channel for the event whose value is being read
 * @type: type of the event whose value is being read
 * @dir: direction of the vent whose value is being read
 * @info: info type of the event whose value is being read
 * @val: value for the event code.
 * @val2: unused
 *
 * The conventional meaning of this type of function would be to display the
 * threshold value rather then the sate of it. This however would be counter
 * intuitive because ADE9078 can only detect zero-crossings. Thus the return
 * results are different from the expected type.
 * Return:
 *	0 - if crossing event not set
 *	1 - if crossing event occurred
 *	-1 - if crossing timeout (only for Voltages)
 */
static int ade9078_read_event_vlaue(struct iio_dev *indio_dev,
//TODO value?  If so, this should be reading the thresholds, not anything
//about current status of events.
				    const struct iio_chan_spec *chan,
				    enum iio_event_type type,
				    enum iio_event_direction dir,
				    enum iio_event_info info,
				    int *val, int *val2)
{
	struct ade9078_state *st = iio_priv(indio_dev);
	u32 handled_irq = 0;
	u32 interrupts;
	u32 number;
	u32 status;
	int ret;

	ret = regmap_read(st->regmap, ADE9078_REG_STATUS1, &status);
	if (ret)
		return ret;

	ret = regmap_read(st->regmap, ADE9078_REG_MASK1, &interrupts);
	if (ret)
		return ret;

	*val = 0;
	number = chan->channel;
	switch (number) {
	case ADE9078_PHASE_A_NR:
		if (chan->type == IIO_VOLTAGE) {
			if (dir == IIO_EV_DIR_EITHER) {
				if (status & ADE9078_ST1_ZXVA_BIT) {
					*val = 1;
					handled_irq |= ADE9078_ST1_ZXVA_BIT;
				} else if (status & ADE9078_ST1_ZXTOVA_BIT) {
					*val = -1;
					handled_irq |= ADE9078_ST1_ZXTOVA_BIT;
				}
				if (!(interrupts & ADE9078_ST1_ZXTOVA_BIT))
					*val = 0;
			}
			if (dir == IIO_EV_DIR_RISING) {
				if (status & ADE9078_ST1_SWELLA_BIT) {
					*val = 1;
					handled_irq |= ADE9078_ST1_SWELLA_BIT;
				}
				if (!(interrupts & ADE9078_ST1_SWELLA_BIT))
					*val = 0;
			}
			if (dir == IIO_EV_DIR_FALLING) {
				if (status & ADE9078_ST1_DIPA_BIT) {
					*val = 1;
					handled_irq |= ADE9078_ST1_DIPA_BIT;
				}
				if (!(interrupts & ADE9078_ST1_DIPA_BIT))
					*val = 0;
			}
		} else if (chan->type == IIO_CURRENT) {
			if (status & ADE9078_ST1_ZXIA_BIT) {
				*val = 1;
				handled_irq |= ADE9078_ST1_ZXIA_BIT;
			}
		} else if (chan->type == IIO_COUNT) {
			*val = (status & ADE9078_ST1_SEQERR_BIT) ? 1 : 0;
			handled_irq |= ADE9078_ST1_SEQERR_BIT;
			if (!(interrupts & ADE9078_ST1_SEQERR_BIT))
				*val = 0;
		}
		break;
	case ADE9078_PHASE_B_NR:
		if (chan->type == IIO_VOLTAGE) {
			if (dir == IIO_EV_DIR_EITHER) {
				if (status & ADE9078_ST1_ZXVB_BIT) {
					*val = 1;
					handled_irq |= ADE9078_ST1_ZXVB_BIT;
				} else if (status & ADE9078_ST1_ZXTOVB_BIT) {
					*val = -1;
					handled_irq |= ADE9078_ST1_ZXTOVB_BIT;
				}
				if (!(interrupts & ADE9078_ST1_ZXTOVB_BIT))
					*val = 0;
			}
			if (dir == IIO_EV_DIR_RISING) {
				if (status & ADE9078_ST1_SWELLB_BIT) {
					*val = 1;
					handled_irq |= ADE9078_ST1_SWELLB_BIT;
				}
				if (!(interrupts & ADE9078_ST1_SWELLB_BIT))
					*val = 0;
			}
			if (dir == IIO_EV_DIR_FALLING) {
				if (status & ADE9078_ST1_DIPB_BIT) {
					*val = 1;
					handled_irq |= ADE9078_ST1_DIPB_BIT;
				}
				if (!(interrupts & ADE9078_ST1_DIPB_BIT))
					*val = 0;
			}
		} else if (chan->type == IIO_CURRENT) {
			if (status & ADE9078_ST1_ZXIB_BIT) {
				*val = 1;
				handled_irq |= ADE9078_ST1_ZXIB_BIT;
			}
		}
		break;
	case ADE9078_PHASE_C_NR:
		if (chan->type == IIO_VOLTAGE) {
			if (dir == IIO_EV_DIR_EITHER) {
				if (status & ADE9078_ST1_ZXVC_BIT) {
					*val = 1;
					handled_irq |= ADE9078_ST1_ZXVC_BIT;
				} else if (status & ADE9078_ST1_ZXTOVC_BIT) {
					*val = -1;
					handled_irq |= ADE9078_ST1_ZXTOVC_BIT;
				}
				if (!(interrupts & ADE9078_ST1_ZXTOVC_BIT))
					*val = 0;
			}
			if (dir == IIO_EV_DIR_RISING) {
				if (status & ADE9078_ST1_SWELLC_BIT) {
					*val = 1;
					handled_irq |= ADE9078_ST1_SWELLC_BIT;
				}
				if (!(interrupts & ADE9078_ST1_SWELLC_BIT))
					*val = 0;
			}
			if (dir == IIO_EV_DIR_FALLING) {
				if (status & ADE9078_ST1_DIPC_BIT) {
					*val = 1;
					handled_irq |= ADE9078_ST1_DIPC_BIT;
				}
				if (!(interrupts & ADE9078_ST1_DIPC_BIT))
					*val = 0;
			}
		} else if (chan->type == IIO_CURRENT) {
			if (status & ADE9078_ST1_ZXIC_BIT) {
				*val = 1;
				handled_irq |= ADE9078_ST1_ZXIC_BIT;
			}
		}
		break;
	default:
		return -EINVAL;
	}

	ret = regmap_write(st->regmap, ADE9078_REG_STATUS1, handled_irq);
//TODO Are there other bits, or is this always going to write all bits that
//might be set? If it is all such bits then
	if (ret)
		return ret;

	return IIO_VAL_INT;
}

/**
 * ade9078_config_wfb() - Reads the ade9078 node and configures the wave form
 * buffer based on the options set.
 * @indio_dev:	the IIO device
 *
 * Besides parsing the device tree for the configuration of the waveform buffer,
 * additionally it reads the active scan mask in order to set the input data of
 * the buffer. There are only a few available input configurations permitted by
 * the IC, any unpermitted configuration will result in all channels being
 * active.
 */
static int ade9078_config_wfb(struct iio_dev *indio_dev)
{
	struct ade9078_state *st = iio_priv(indio_dev);
	u32 wfg_cfg_val = 0;
	u32 active_scans;
	u32 tmp;
	int ret;

	bitmap_to_arr32(&active_scans, indio_dev->active_scan_mask,
			indio_dev->masklength);

	switch (active_scans) {
	case ADE9078_SCAN_POS_IA | ADE9078_SCAN_POS_VA:
		wfg_cfg_val = 0x1;
		st->wfb_nr_activ_chan = 2;
		break;
	case ADE9078_SCAN_POS_IB | ADE9078_SCAN_POS_VB:
		wfg_cfg_val = 0x2;
		st->wfb_nr_activ_chan = 2;
		break;
	case ADE9078_SCAN_POS_IC | ADE9078_SCAN_POS_VC:
		wfg_cfg_val = 0x3;
		st->wfb_nr_activ_chan = 2;
		break;
	case ADE9078_SCAN_POS_IA:
		wfg_cfg_val = 0x8;
		st->wfb_nr_activ_chan = 1;
		break;
	case ADE9078_SCAN_POS_VA:
		wfg_cfg_val = 0x9;
		st->wfb_nr_activ_chan = 1;
		break;
	case ADE9078_SCAN_POS_IB:
		wfg_cfg_val = 0xA;
		st->wfb_nr_activ_chan = 1;
		break;
	case ADE9078_SCAN_POS_VB:
		wfg_cfg_val = 0xB;
		st->wfb_nr_activ_chan = 1;
		break;
	case ADE9078_SCAN_POS_IC:
		wfg_cfg_val = 0xC;
		st->wfb_nr_activ_chan = 1;
		break;
	case ADE9078_SCAN_POS_VC:
		wfg_cfg_val = 0xD;
		st->wfb_nr_activ_chan = 1;
		break;
	case ADE9078_SCAN_POS_ALL:
		wfg_cfg_val = 0x0;
		st->wfb_nr_activ_chan = 6;
		break;
	default:
		dev_err(&st->spi->dev, "Unsupported combination of scans");
		return -EINVAL;
	}

	ret = device_property_read_u32(&st->spi->dev, "adi,wf-cap-sel", &tmp);
	if (ret) {
		dev_err(&st->spi->dev, "Failed to get wf-cap-sel: %d\n", ret);
		return ret;
	}
	wfg_cfg_val |= FIELD_PREP(ADE9078_WF_CAP_SEL_MASK, tmp);

	ret = device_property_read_u32(&st->spi->dev, "adi,wf-mode", &tmp);
	if (ret) {
		dev_err(&st->spi->dev, "Failed to get wf-mode: %d\n", ret);
		return ret;
	}
	wfg_cfg_val |= FIELD_PREP(ADE9078_WF_MODE_MASK, tmp);
	st->wf_mode = tmp;

	ret = device_property_read_u32(&st->spi->dev, "adi,wf-src", &tmp);
	if (ret) {
		dev_err(&st->spi->dev,
			"Failed to get wf-src: %d\n",
			ret);
		return ret;
	}
	wfg_cfg_val |= FIELD_PREP(ADE9078_WF_SRC_MASK, tmp);

	ret = device_property_read_u32(&st->spi->dev, "adi,wf-in-en", &tmp);
	if (ret) {
		dev_err(&st->spi->dev, "Failed to get wf-in-en: %d\n", ret);
		return ret;
	}
	wfg_cfg_val |= FIELD_PREP(ADE9078_WF_IN_EN_MASK, tmp);

	return regmap_write(st->regmap, ADE9078_REG_WFB_CFG, wfg_cfg_val);
}

/**
 * ade9078_wfb_interrupt_setup() - Configures the wave form buffer interrupt
 * according to modes
 * @st:		ade9078 device data
 * @mode:	modes according to datasheet; values [0-2]
 *
 * This sets the interrupt register and other registers related to the
 * interrupts according to mode [0-2] from the datasheet
 */
static int ade9078_wfb_interrupt_setup(struct ade9078_state *st, u8 mode)
{
	int ret;

	ret = regmap_write(st->regmap, ADE9078_REG_WFB_TRG_CFG, 0x0);
	if (ret)
		return ret;

	switch (mode) {
	case ADE9078_WFB_FULL_MODE:
	case ADE9078_WFB_EN_TRIG_MODE:
		ret = regmap_write(st->regmap, ADE9078_REG_WFB_PG_IRQEN,
				   ADE9078_LAST_PAGE_BIT);
		if (ret)
			return ret;
		break;
	case ADE9078_WFB_C_EN_TRIG_MODE:
		ret = regmap_write(st->regmap, ADE9078_REG_WFB_PG_IRQEN,
				   ADE9078_MIDDLE_PAGE_BIT);
		if (ret)
			return ret;
		break;
	case ADE9078_WFB_STREAMING_MODE:
		ret = regmap_write(st->regmap, ADE9078_REG_WFB_PG_IRQEN,
				   ADE9078_MIDDLE_PAGE_BIT);
		if (ret)
			return ret;
		break;
	default:
		return -EINVAL;
	}

	ret = regmap_write(st->regmap, ADE9078_REG_STATUS0, GENMASK(31, 0));
	if (ret)
		return ret;

	return regmap_update_bits(st->regmap, ADE9078_REG_MASK0,
				  ADE9078_ST0_PAGE_FULL_BIT,
				  ADE9078_ST0_PAGE_FULL_BIT);
}

/**
 * ade9078_buffer_preenable() - Configures the waveform buffer, sets the
 * interrupts and enables the buffer
 * @indio_dev:	the IIO device
 */
static int ade9078_buffer_preenable(struct iio_dev *indio_dev)
{
	struct ade9078_state *st = iio_priv(indio_dev);
	int ret;

	ret = ade9078_config_wfb(indio_dev);
	if (ret)
		return ret;

	st->wfb_nr_samples = ADE9078_WFB_MAX_SAMPLES_CHAN *
			     st->wfb_nr_activ_chan;

	ret = ade9078_configure_scan(indio_dev, ADE9078_REG_WF_BUFF);
	if (ret)
		return ret;

	ret = ade9078_wfb_interrupt_setup(st, st->wf_mode);
	if (ret)
		return ret;

	ret = ade9078_en_wfb(st, true);
	if (ret) {
		dev_err(&st->spi->dev, "Post-enable wfb enable fail");
		return ret;
	}

	return 0;
}

/**
 * ade9078_buffer_postdisable() - After the iio is disable
 * this will disable the ade9078 internal buffer for acquisition
 * @indio_dev:	the IIO device
 */
static int ade9078_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct ade9078_state *st = iio_priv(indio_dev);
	u32 interrupts = 0;
	int ret;

	ret = ade9078_en_wfb(st, false);
	if (ret) {
		dev_err(&st->spi->dev, "Post-disable wfb disable fail");
		return ret;
	}

	ret = regmap_write(st->regmap, ADE9078_REG_WFB_TRG_CFG, 0x0);
	if (ret)
		return ret;

	interrupts |= ADE9078_ST0_WFB_TRIG_BIT;
	interrupts |= ADE9078_ST0_PAGE_FULL_BIT;

	return regmap_update_bits(st->regmap, ADE9078_REG_MASK0, interrupts, 0);
	if (ret) {
		dev_err(&st->spi->dev, "Post-disable update maks0 fail");
		return ret;
	}

	return regmap_write(st->regmap, ADE9078_REG_STATUS0, GENMASK(31, 0));
}

/**
 * ade9078_setup_iio_channels() - parses the phase nodes of the device-tree and
 * creates the iio channels based on the active phases in the DT.
 * @indio_dev:	the IIO device
 */
static int ade9078_setup_iio_channels(struct iio_dev *indio_dev)
{
	struct ade9078_state *st = iio_priv(indio_dev);
	struct device *dev = &st->spi->dev;
	struct fwnode_handle *phase_node = NULL;
	struct iio_chan_spec *chan;
	u32 phase_nr;
	int ret;

	chan = devm_kcalloc(dev,
			    (ADE9078_MAX_PHASE_NR *
			     ARRAY_SIZE(ade9078_a_channels)) + 1,
			    sizeof(*chan), GFP_KERNEL);
	if (!chan) {
		dev_err(dev, "Unable to allocate ADE9078 channels");
		return -ENOMEM;
	}
	indio_dev->num_channels = 0;
	indio_dev->channels = chan;

	struct iio_chan_spec *chan_ptr = chan;	

	fwnode_for_each_available_child_node(dev_fwnode(dev), phase_node) {
		ret = fwnode_property_read_u32(phase_node, "reg", &phase_nr);
		if (ret) {
			dev_err(dev, "Could not read channel reg : %d\n", ret);
			return ret;
		}

		switch (phase_nr) {
		case ADE9078_PHASE_A_NR:
			memcpy(chan_ptr, ade9078_a_channels,
			       sizeof(ade9078_a_channels));
			break;
		case ADE9078_PHASE_B_NR:
			memcpy(chan_ptr, ade9078_b_channels,
			       sizeof(ade9078_b_channels));
			break;
		case ADE9078_PHASE_C_NR:
			memcpy(chan_ptr, ade9078_c_channels,
			       sizeof(ade9078_c_channels));
			break;
		default:
			return -EINVAL;
		}

		chan_ptr += AD9078_CHANNELS_PER_PHASE;
		indio_dev->num_channels += AD9078_CHANNELS_PER_PHASE;
	}

	chan[indio_dev->num_channels] = (struct iio_chan_spec) {
		.type = IIO_COUNT,
		.channel = 0,
		.address = ADE9078_REG_STATUS1,
		.extend_name = "seqerr",
		.event_spec = &ade9078_events[0],
		.num_event_specs = 1,
		.scan_index = -1, // not used in buffer
	};
	indio_dev->num_channels++;

	return 0;
}

/**
 * ade9078_reset() - Reset sequence for the ADE9078.
 * @st:		ade9078 device data
 *
 * The hardware reset is optional in the DT. When no hardware reset has been
 * declared a software reset is executed.
 */
static int ade9078_reset(struct ade9078_state *st)
{
	struct gpio_desc *gpio_reset;
	int ret;

	st->rst_done = false;

	gpio_reset = devm_gpiod_get_optional(&st->spi->dev, "reset",
					     GPIOD_OUT_HIGH);
	if (IS_ERR(gpio_reset))
		return PTR_ERR(gpio_reset);

	if (gpio_reset) {
		gpiod_set_value_cansleep(gpio_reset, 1);
		usleep_range(1, 100);
		gpiod_set_value_cansleep(gpio_reset, 0);
		msleep_interruptible(50);
	} else {
		ret = regmap_update_bits(st->regmap, ADE9078_REG_CONFIG1,
					 ADE9078_SWRST_BIT, ADE9078_SWRST_BIT);
		if (ret)
			return ret;
		usleep_range(80, 100);
	}

	if (!st->rst_done)
		return -EIO;

	return 0;
}

/**
 * ade9078_setup() - initial register setup of the ade9078
 * @st:		ade9078 device data
 */
static int ade9078_setup(struct ade9078_state *st)
{
	int ret;
	u32 tmp;

	ret = device_property_read_u32(&st->spi->dev, "adi,egy-time", &tmp);
	if (!ret)
		ade9078_reg_sequence[13].def = tmp;

	ret = regmap_multi_reg_write(st->regmap, ade9078_reg_sequence,
				     ARRAY_SIZE(ade9078_reg_sequence));
	if (ret)
		return ret;

	msleep_interruptible(2);

	ret = regmap_write(st->regmap, ADE9078_REG_STATUS0, GENMASK(31, 0));
	if (ret)
		return ret;

	return regmap_write(st->regmap, ADE9078_REG_STATUS1, GENMASK(31, 0));
}

static const struct iio_buffer_setup_ops ade9078_buffer_ops = {
	.preenable = &ade9078_buffer_preenable,
	.postdisable = &ade9078_buffer_postdisable,
};

static const struct iio_info ade9078_info = {
	.read_raw = &ade9078_read_raw,
	.write_raw = &ade9078_write_raw,
	.debugfs_reg_access = &ade9078_reg_access,
	.write_event_config = &ade9078_write_event_config,
	.read_event_value = &ade9078_read_event_vlaue,
};

//TODO How big would the changes needed to support this in the regmap
//core be?   Superficially I can't immediately see why it won't work.
//regmap appears to support setting flags in any of the address bytes
//as it uses regmap_set_work_buf_flag_mask() internally and
//that will set bits in any of the reg_bits/8 bytes.
/*
 * Regmap configuration
 * The register access of the ade9078 requires a 16 bit address
 * with the read flag on bit 3. This is not supported by default
 * regmap functionality, thus reg_read and reg_write have been
 * replaced with custom functions
 */
static const struct regmap_config ade9078_regmap_config = {
	.reg_bits = 16,
	.val_bits = 32,
	.zero_flag_mask = true,
	.cache_type = REGCACHE_RBTREE,
	.reg_read = ade9078_spi_read_reg,
	.reg_write = ade9078_spi_write_reg,
	.volatile_reg = ade9078_is_volatile_reg,
};

static int ade9078_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct ade9078_state *st;
#if 0
	struct iio_buffer *buffer;
#endif
	struct regmap *regmap;
	int irq;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev) {
		dev_err(&spi->dev, "Unable to allocate ADE9078 IIO");
		return -ENOMEM;
	}
	st = iio_priv(indio_dev);
//TODO Avoid having multiple small allocations like this by making them part of
//the iio_priv() structure.
	st->rx = devm_kcalloc(&spi->dev, ADE9078_RX_DEPTH, sizeof(*st->rx),
			      GFP_KERNEL);
	if (!st->rx)
		return -ENOMEM;

	st->tx = devm_kcalloc(&spi->dev, ADE9078_TX_DEPTH, sizeof(*st->tx),
			      GFP_KERNEL);
	if (!st->tx)
		return -ENOMEM;
//TODO Probably better to pass the iio_device structure and have a pointer to
//spi in there.
	regmap = devm_regmap_init(&spi->dev, NULL, spi, &ade9078_regmap_config);
//TODO This shouldn't be needed after you've changed the passed context to provide
//the iio_dev structure and you can then call iio_priv() on that.
	if (IS_ERR(regmap))	{
		dev_err(&spi->dev, "Unable to allocate ADE9078 regmap");
		return PTR_ERR(regmap);
	}
	spi_set_drvdata(spi, st);

//TODO A generic firmware properties version of irq_get_by_name is queued in
//the i2c tree.  Please use that here and mention it in the
//patch description / cover letter.  I'll probably have pulled it into
//IIO shortly anyway.
//https://git.kernel.org/pub/scm/linux/kernel/git/wsa/linux.git/commit/?h=i2c/alert-for-acpi&id=ca0acb511c21738b32386ce0f85c284b351d919e

	irq = of_irq_get_byname((&spi->dev)->of_node, "irq0");
	if (irq < 0) {
		dev_err(&spi->dev, "Unable to find irq0");
		return -EINVAL;
	}

	ret = devm_request_threaded_irq(&spi->dev, irq, NULL,
					ade9078_irq0_thread,
					IRQF_ONESHOT,
					KBUILD_MODNAME, indio_dev);
	if (ret) {
		dev_err(&spi->dev, "Failed to request threaded irq: %d\n", ret);
		return ret;
	}

	irq = of_irq_get_byname((&spi->dev)->of_node, "irq1");
	if (irq < 0) {
		dev_err(&spi->dev, "Unable to find irq1");
		return -EINVAL;
	}

	ret = devm_request_threaded_irq(&spi->dev, irq, NULL,
					ade9078_irq1_thread,
					IRQF_ONESHOT,
					KBUILD_MODNAME, indio_dev);
	if (ret) {
		dev_err(&spi->dev, "Failed to request threaded irq: %d\n", ret);
		return ret;
	}

	st->spi = spi;

	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->dev.parent = &st->spi->dev;
	indio_dev->info = &ade9078_info;
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_SOFTWARE;
	indio_dev->setup_ops = &ade9078_buffer_ops;

	st->regmap = regmap;

	ret = ade9078_setup_iio_channels(indio_dev);
	if (ret) {
		dev_err(&spi->dev, "Failed to set up IIO channels");
		return ret;
	}
#if 0
	buffer = devm_iio_kfifo_allocate(&spi->dev);
	if (!buffer)
		return -ENOMEM;

	iio_device_attach_buffer(indio_dev, buffer);
#endif
	ret = devm_iio_kfifo_buffer_setup(&spi->dev, indio_dev,
		&ade9078_buffer_ops);
	if (ret)
		return ret;

	ret = ade9078_reset(st);
	if (ret) {
		dev_err(&spi->dev, "ADE9078 reset failed");
		return ret;
	}

	ret = ade9078_setup(st);
	if (ret) {
		dev_err(&spi->dev, "Unable to setup ADE9078");
		return ret;
	}

	ret = devm_iio_device_register(&spi->dev, indio_dev);
	if (ret) {
		dev_err(&spi->dev, "Unable to register IIO device");
		return ret;
	}

	return ret;
};

static const struct spi_device_id ade9078_id[] = {
		{"ade9078", 0},
		{"ade9039", 1},
		{}
};
MODULE_DEVICE_TABLE(spi, ade9078_id);

static const struct of_device_id ade9078_of_match[] = {
	{ .compatible = "adi,ade9078" },
	{ .compatible = "adi,ade9039" },
	{}
};
MODULE_DEVICE_TABLE(of, ade9078_of_match);

static struct spi_driver ade9078_driver = {
		.driver = {
			.name = "ade9078",
		},
		.probe = ade9078_probe,
		.id_table = ade9078_id,
};
module_spi_driver(ade9078_driver);

MODULE_AUTHOR("Ciprian Hegbeli <ciprian.hegbeli@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADE9078 Polyphase Energy Metering IC Driver");
MODULE_LICENSE("Dual BSD/GPL");
