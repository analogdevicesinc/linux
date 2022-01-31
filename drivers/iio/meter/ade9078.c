// SPDX-License-Identifier: GPL-2.0-only
/*
 * ADE9078 driver
 *
 * Copyright 2021 Analog Devices Inc.
 */

#include <asm/unaligned.h>
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
#include <linux/spi/spi.h>
#include <linux/regmap.h>

/*address of ADE90XX registers*/
#define	ADDR_AIGAIN			0x000
#define	ADDR_AVGAIN			0x00B
#define	ADDR_AIRMSOS			0x00C
#define	ADDR_AVRMSOS			0x00D
#define	ADDR_APGAIN			0x00E
#define	ADDR_AWATTOS			0x00F
#define	ADDR_AVAROS			0x010
#define	ADDR_AFVAROS			0x012
#define	ADDR_CONFIG0			0x060
#define	ADDR_DICOEFF			0x072
#define	ADDR_AI_PCF			0x20A
#define	ADDR_AV_PCF			0x20B
#define	ADDR_AIRMS			0x20C
#define	ADDR_AVRMS			0x20D
#define	ADDR_AWATT			0x210
#define	ADDR_AVAR			0x211
#define	ADDR_AVA			0x212
#define ADDR_AFVAR			0x214
#define	ADDR_APF			0x216
#define	ADDR_CI_PCF			0x24A
#define	ADDR_CV_PCF			0x24B
#define	ADDR_CIRMS			0x24C
#define	ADDR_CVRMS			0x24D
#define	ADDR_AWATT_ACC			0x2E5
#define	ADDR_STATUS0			0x402
#define	ADDR_STATUS1			0x403
#define	ADDR_MASK0			0x405
#define	ADDR_MASK1			0x406
#define	ADDR_EVENT_MASK			0x407
#define	ADDR_VLEVEL			0x40F
#define	ADDR_RUN			0x480
#define	ADDR_CONFIG1			0x481
#define	ADDR_ACCMODE			0x492
#define	ADDR_CONFIG3			0x493
#define	ADDR_ZX_LP_SEL			0x49A
#define	ADDR_WFB_CFG			0x4A0
#define	ADDR_WFB_PG_IRQEN		0x4A1
#define	ADDR_WFB_TRG_CFG		0x4A2
#define	ADDR_WFB_TRG_STAT		0x4A3
#define	ADDR_CONFIG2			0x4AF
#define	ADDR_EP_CFG			0x4B0
#define	ADDR_EGY_TIME			0x4B2
#define	ADDR_PGA_GAIN			0x4B9
#define	ADDR_VERSION			0x4FE
#define ADDR_WF_BUFF			0x800

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

/*Default configuration*/
#define ADE9078_CONFIG0			0x00000000

/*CF3/ZX pin outputs Zero crossing*/
#define ADE9078_CONFIG1			0x0002

/*Default High pass corner frequency of 1.25Hz*/
#define ADE9078_CONFIG2			0x0A00

/*Peak and overcurrent detection disabled*/
#define ADE9078_CONFIG3			0x0000

/*
 * 50Hz operation, 3P4W Wye configuration, signed accumulation
 * Clear bit 8 i.e. ACCMODE=0x00xx for 50Hz operation
 * ACCMODE=0x0x9x for 3Wire delta when phase B is used as reference
 */
#define ADE9078_ACCMODE			0x0000

/*Line period and zero crossing obtained from VA*/
#define ADE9078_ZX_LP_SEL		0x0000

/*Disable all interrupts*/
#define ADE9078_MASK0			0x00000000

/*Disable all interrupts*/
#define ADE9078_MASK1			0x00000000

/*Events disabled*/
#define ADE9078_EVENT_MASK		0x00000000

/*
 * Assuming Vnom=1/2 of full scale.
 * Refer Technical reference manual for detailed calculations.
 */
#define ADE9078_VLEVEL			0x0022EA28

/*Set DICOEFF= 0xFFFFE000 when integrator is enabled*/
#define ADE9078_DICOEFF			0x00000000

/*DSP ON*/
#define ADE9078_RUN_ON			0xFFFFFFFF

/*
 * Energy Accumulation Settings
 * Enable energy accumulation, accumulate samples at 8ksps
 * latch energy accumulation after EGYRDY
 * If accumulation is changed to half line cycle mode, change EGY_TIME
 */
#define ADE9078_EP_CFG			0x0011

/*Accumulate 4000 samples*/
#define ADE9078_EGY_TIME		0x0FA0

/*
 * Constant Definitions
 * ADE9000 FDSP: 8000sps, ADE9078 FDSP: 4000sps
 */
#define ADE9078_FDSP			4000
#define ADE9078_WFB_CFG			0x0329
#define ADE9078_WFB_PAGE_SIZE		128
#define ADE9078_WFB_BYTES_IN_PAGE	4
#define ADE9078_WFB_PAGE_ARRAY_SIZE	\
	(ADE9078_WFB_PAGE_SIZE * ADE9078_WFB_BYTES_IN_PAGE)
#define ADE9078_WFB_FULL_BUFF_SIZE	\
	(ADE9078_WFB_PAGE_ARRAY_SIZE * 16)
#define ADE9078_WFB_FULL_BUFF_NR_SAMPLES \
	(ADE9078_WFB_PAGE_SIZE * 16)

#define ADE9078_SWRST_BIT		BIT(0)
#define ADE9078_SWRST_MASK		BIT(0)

/*Status and Mask register bits*/
#define ADE9078_ST0_WFB_TRIG		16
#define ADE9078_ST0_WFB_TRIG_BIT	BIT(ADE9078_ST0_WFB_TRIG)
#define ADE9078_ST0_PAGE_FULL		17
#define ADE9078_ST0_PAGE_FULL_BIT	BIT(ADE9078_ST0_PAGE_FULL)

#define ADE9078_ST1_ZXTOVA		6
#define ADE9078_ST1_ZXTOVA_BIT		BIT(ADE9078_ST1_ZXTOVA)
#define ADE9078_ST1_ZXTOVB		7
#define ADE9078_ST1_ZXTOVB_BIT		BIT(ADE9078_ST1_ZXTOVB)
#define ADE9078_ST1_ZXTOVC		8
#define ADE9078_ST1_ZXTOVC_BIT		BIT(ADE9078_ST1_ZXTOVC)
#define ADE9078_ST1_ZXVA		9
#define ADE9078_ST1_ZXVA_BIT		BIT(ADE9078_ST1_ZXVA)
#define ADE9078_ST1_ZXVB		10
#define ADE9078_ST1_ZXVB_BIT		BIT(ADE9078_ST1_ZXVB)
#define ADE9078_ST1_ZXVC		11
#define ADE9078_ST1_ZXVC_BIT		BIT(ADE9078_ST1_ZXVC)
#define ADE9078_ST1_ZXIA		13
#define ADE9078_ST1_ZXIA_BIT		BIT(ADE9078_ST1_ZXIA)
#define ADE9078_ST1_ZXIB		14
#define ADE9078_ST1_ZXIB_BIT		BIT(ADE9078_ST1_ZXIB)
#define ADE9078_ST1_ZXIC		15
#define ADE9078_ST1_ZXIC_BIT		BIT(ADE9078_ST1_ZXIC)
#define ADE9078_ST1_RSTDONE_BIT		BIT(16)
#define ADE9078_ST1_ERROR0		BIT(28)
#define ADE9078_ST1_ERROR1		BIT(29)
#define ADE9078_ST1_ERROR2		BIT(30)
#define ADE9078_ST1_ERROR3		BIT(31)
#define ADE9078_ST_ERROR \
	(ADE9078_ST1_ERROR0 | \
	ADE9078_ST1_ERROR1 | \
	ADE9078_ST1_ERROR2 | \
	ADE9078_ST1_ERROR3)
#define ADE9078_ST1_CROSSING_FIRST	6
#define ADE9078_ST1_CROSSING_DEPTH	16

#define ADE9078_WFB_TRG_ZXIA		3
#define ADE9078_WFB_TRG_ZXIA_BIT	BIT(ADE9078_WFB_TRG_ZXIA)
#define ADE9078_WFB_TRG_ZXIB		4
#define ADE9078_WFB_TRG_ZXIB_BIT	BIT(ADE9078_WFB_TRG_ZXIB)
#define ADE9078_WFB_TRG_ZXIC		5
#define ADE9078_WFB_TRG_ZXIC_BIT	BIT(ADE9078_WFB_TRG_ZXIC)
#define ADE9078_WFB_TRG_ZXVA		6
#define ADE9078_WFB_TRG_ZXVA_BIT	BIT(ADE9078_WFB_TRG_ZXVA)
#define ADE9078_WFB_TRG_ZXVB		7
#define ADE9078_WFB_TRG_ZXVB_BIT	BIT(ADE9078_WFB_TRG_ZXVB)
#define ADE9078_WFB_TRG_ZXVC		8
#define ADE9078_WFB_TRG_ZXVC_BIT	BIT(ADE9078_WFB_TRG_ZXVC)

#define ADE9078_MODE_0_1_PAGE_BIT	BIT(15)
#define ADE9078_MODE_2_PAGE_BIT		BIT(7)

/*
 * Full scale Codes referred from Datasheet.Respective digital codes are
 * produced when ADC inputs are at full scale. Do not Change.
 */
#define ADE9078_RMS_FULL_SCALE_CODES	52866837
#define ADE9000_WATT_FULL_SCALE_CODES	20694066
#define ADE9078_PCF_FULL_SCALE_CODES	74770000

/*Phase and channel definitions*/
#define ADE9078_PHASE_A_NR		0
#define ADE9078_PHASE_B_NR		2
#define ADE9078_PHASE_C_NR		4
#define ADE9078_PHASE_A_NAME		"A"
#define ADE9078_PHASE_B_NAME		"B"
#define ADE9078_PHASE_C_NAME		"C"

#define ADE9078_SCAN_POS_IA		BIT(0)
#define ADE9078_SCAN_POS_VA		BIT(1)
#define ADE9078_SCAN_POS_IB		BIT(2)
#define ADE9078_SCAN_POS_VB		BIT(3)
#define ADE9078_SCAN_POS_IC		BIT(4)
#define ADE9078_SCAN_POS_VC		BIT(5)

#define ADE9078_PHASE_B_POS		5
#define ADE9078_PHASE_C_POS		6

#define ADE9078_MAX_PHASE_NR		3

#define PHASE_ADDR_ADJUST(addr, chan)	(((chan) << 4) | (addr))

#define ADE9078_CURRENT_CHANNEL(_num, _name) {				\
	.type = IIO_CURRENT,						\
	.channel = _num,						\
	.extend_name = _name,						\
	.address = PHASE_ADDR_ADJUST(ADDR_AI_PCF, _num),		\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_SCALE) |		\
			      BIT(IIO_CHAN_INFO_HARDWAREGAIN),		\
	.event_spec = ade9078_events,					\
	.num_event_specs = ARRAY_SIZE(ade9078_events),			\
	.scan_index = _num,						\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = 32,						\
		.storagebits = 32,					\
		.shift = 0,						\
		.endianness = IIO_BE,					\
	},								\
}

#define ADE9078_VOLTAGE_CHANNEL(_num, _name) {				\
	.type = IIO_VOLTAGE,						\
	.channel = _num,						\
	.extend_name = _name,						\
	.address = PHASE_ADDR_ADJUST(ADDR_AV_PCF, _num),		\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_SCALE) |		\
			      BIT(IIO_CHAN_INFO_HARDWAREGAIN),		\
	.event_spec = ade9078_events,					\
	.num_event_specs = ARRAY_SIZE(ade9078_events),			\
	.scan_index = _num + 1,						\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = 32,						\
		.storagebits = 32,					\
		.shift = 0,						\
		.endianness = IIO_BE,					\
	},								\
}

#define ADE9078_CURRENT_RMS_CHANNEL(_num, _name) {			\
	.type = IIO_CURRENT,						\
	.channel = _num,						\
	.address = PHASE_ADDR_ADJUST(ADDR_AIRMS, _num),			\
	.extend_name = _name "_rms",					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_SCALE) |		\
			      BIT(IIO_CHAN_INFO_OFFSET),		\
	.scan_index = -1						\
}

#define ADE9078_VOLTAGE_RMS_CHANNEL(_num, _name) {			\
	.type = IIO_VOLTAGE,						\
	.channel = _num,						\
	.address = PHASE_ADDR_ADJUST(ADDR_AVRMS, _num),			\
	.extend_name = _name "_rms",					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_SCALE) |		\
			      BIT(IIO_CHAN_INFO_OFFSET),		\
	.scan_index = -1						\
}

#define ADE9078_POWER_ACTIV_CHANNEL(_num, _name) {			\
	.type = IIO_POWER,						\
	.channel = _num,						\
	.address = PHASE_ADDR_ADJUST(ADDR_AWATT, _num),			\
	.extend_name = _name "_activ",					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_SCALE) |		\
			      BIT(IIO_CHAN_INFO_OFFSET) |		\
			      BIT(IIO_CHAN_INFO_HARDWAREGAIN),		\
	.scan_index = -1						\
}

#define ADE9078_POWER_REACTIV_CHANNEL(_num, _name) {			\
	.type = IIO_POWER,						\
	.channel = _num,						\
	.address = PHASE_ADDR_ADJUST(ADDR_AVAR, _num),			\
	.extend_name = _name "_reactiv",				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_SCALE) |		\
			      BIT(IIO_CHAN_INFO_OFFSET),		\
	.scan_index = -1						\
}

#define ADE9078_POWER_APPARENT_CHANNEL(_num, _name) {			\
	.type = IIO_POWER,						\
	.channel = _num,						\
	.address = PHASE_ADDR_ADJUST(ADDR_AVA, _num),			\
	.extend_name = _name "_apparent",				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_SCALE),			\
	.scan_index = -1						\
}

#define ADE9078_POWER_FUND_REACTIV_CHANNEL(_num, _name) {		\
	.type = IIO_POWER,						\
	.channel = _num,						\
	.address = PHASE_ADDR_ADJUST(ADDR_AFVAR, _num),			\
	.extend_name = _name "_fund_reactiv",				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_SCALE) |		\
			      BIT(IIO_CHAN_INFO_OFFSET),		\
	.scan_index = -1						\
}

#define ADE9078_POWER_FACTOR_CHANNEL(_num, _name) {			\
	.type = IIO_POWER,						\
	.channel = _num,						\
	.address = PHASE_ADDR_ADJUST(ADDR_APF, _num),			\
	.extend_name = _name "_factor",					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_SCALE),			\
	.scan_index = -1						\
}

/*
 * struct ade9078_state - ade9078 specific data
 * @irq0_bits	IRQ0 mask and status bits, are set by the driver and are passed
 *		to the IC after being set
 * @irq1_bits	IRQ1 mask and status bits, are set by the driver and are passed
 *		to the IC after being set
 * @irq1_status status variable updated in irq1 and used in IIO event readout
 * @rst_done	flag for when reset sequence irq has been received
 * @wf_mode	wave form buffer mode, read datasheet for more details,
 *		retrieved from DT
 * @wfb_trg	wave form buffer triger configuration, read datasheet for more
 *		details, retrieved from DT
 * @spi		spi device associated to the ade9078
 * @tx		transmit buffer for the spi
 * @rx		receive buffer for the spi
 * @xfer	transfer setup used in iio buffer configuration
 * @spi_msg	message transfer trough spi, used in iio buffer
 *		configuration
 * @regmap	register map pointer
 * @indio_dev:	the IIO device
 * @trig	iio trigger pointer, is connected to IRQ0 and IRQ1
 * @rx_buff	receive buffer for the iio buffer trough spi, will
 *		contain the samples from the IC wave form buffer
 * @tx_buff	transmit buffer for the iio buffer trough spi, used
 *		in iio	buffer configuration
 */

struct ade9078_state {
	u32 irq0_bits;
	u32 irq1_bits;
	u32 irq1_status;
	bool rst_done;
	u8 wf_mode;
	u32 wfb_trg;
	struct spi_device *spi;
	u8 *tx;
	u8 *rx;
	struct spi_transfer xfer[2];
	struct spi_message spi_msg;
	struct regmap *regmap;
	struct iio_dev *indio_dev;
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
};

/*IIO channels of the ade9078 for each phase individually*/
static const struct iio_chan_spec ade9078_a_channels[] = {
	ADE9078_CURRENT_CHANNEL(ADE9078_PHASE_A_NR, ADE9078_PHASE_A_NAME),
	ADE9078_VOLTAGE_CHANNEL(ADE9078_PHASE_A_NR, ADE9078_PHASE_A_NAME),
	ADE9078_CURRENT_RMS_CHANNEL(ADE9078_PHASE_A_NR, ADE9078_PHASE_A_NAME),
	ADE9078_VOLTAGE_RMS_CHANNEL(ADE9078_PHASE_A_NR, ADE9078_PHASE_A_NAME),
	ADE9078_POWER_ACTIV_CHANNEL(ADE9078_PHASE_A_NR, ADE9078_PHASE_A_NAME),
	ADE9078_POWER_REACTIV_CHANNEL(ADE9078_PHASE_A_NR, ADE9078_PHASE_A_NAME),
	ADE9078_POWER_APPARENT_CHANNEL(ADE9078_PHASE_A_NR,
				       ADE9078_PHASE_A_NAME),
	ADE9078_POWER_FUND_REACTIV_CHANNEL(ADE9078_PHASE_A_NR,
					   ADE9078_PHASE_A_NAME),
	ADE9078_POWER_FACTOR_CHANNEL(ADE9078_PHASE_A_NR, ADE9078_PHASE_A_NAME),
};

static const struct iio_chan_spec ade9078_b_channels[] = {
	ADE9078_CURRENT_CHANNEL(ADE9078_PHASE_B_NR, ADE9078_PHASE_B_NAME),
	ADE9078_VOLTAGE_CHANNEL(ADE9078_PHASE_B_NR, ADE9078_PHASE_B_NAME),
	ADE9078_CURRENT_RMS_CHANNEL(ADE9078_PHASE_B_NR, ADE9078_PHASE_B_NAME),
	ADE9078_VOLTAGE_RMS_CHANNEL(ADE9078_PHASE_B_NR, ADE9078_PHASE_B_NAME),
	ADE9078_POWER_ACTIV_CHANNEL(ADE9078_PHASE_B_NR, ADE9078_PHASE_B_NAME),
	ADE9078_POWER_REACTIV_CHANNEL(ADE9078_PHASE_B_NR, ADE9078_PHASE_B_NAME),
	ADE9078_POWER_APPARENT_CHANNEL(ADE9078_PHASE_B_NR,
				       ADE9078_PHASE_B_NAME),
	ADE9078_POWER_FUND_REACTIV_CHANNEL(ADE9078_PHASE_B_NR,
					   ADE9078_PHASE_B_NAME),
	ADE9078_POWER_FACTOR_CHANNEL(ADE9078_PHASE_B_NR, ADE9078_PHASE_B_NAME),
};

static const struct iio_chan_spec ade9078_c_channels[] = {
	ADE9078_CURRENT_CHANNEL(ADE9078_PHASE_C_NR, ADE9078_PHASE_C_NAME),
	ADE9078_VOLTAGE_CHANNEL(ADE9078_PHASE_C_NR, ADE9078_PHASE_C_NAME),
	ADE9078_CURRENT_RMS_CHANNEL(ADE9078_PHASE_C_NR, ADE9078_PHASE_C_NAME),
	ADE9078_VOLTAGE_RMS_CHANNEL(ADE9078_PHASE_C_NR, ADE9078_PHASE_C_NAME),
	ADE9078_POWER_ACTIV_CHANNEL(ADE9078_PHASE_C_NR, ADE9078_PHASE_C_NAME),
	ADE9078_POWER_REACTIV_CHANNEL(ADE9078_PHASE_C_NR, ADE9078_PHASE_C_NAME),
	ADE9078_POWER_APPARENT_CHANNEL(ADE9078_PHASE_C_NR,
				       ADE9078_PHASE_C_NAME),
	ADE9078_POWER_FUND_REACTIV_CHANNEL(ADE9078_PHASE_C_NR,
					   ADE9078_PHASE_C_NAME),
	ADE9078_POWER_FACTOR_CHANNEL(ADE9078_PHASE_C_NR, ADE9078_PHASE_C_NAME),
};

static const struct reg_sequence ade9078_reg_sequence[] = {
	{ADDR_PGA_GAIN, ADE9078_PGA_GAIN},
	{ADDR_CONFIG0, ADE9078_CONFIG0},
	{ADDR_CONFIG1, ADE9078_CONFIG1},
	{ADDR_CONFIG2, ADE9078_CONFIG2},
	{ADDR_CONFIG3, ADE9078_CONFIG3},
	{ADDR_ACCMODE, ADE9078_ACCMODE},
	{ADDR_ZX_LP_SEL, ADE9078_ZX_LP_SEL},
	{ADDR_MASK0, ADE9078_MASK0},
	{ADDR_MASK1, ADE9078_MASK1},
	{ADDR_EVENT_MASK, ADE9078_EVENT_MASK},
	{ADDR_WFB_CFG, ADE9078_WFB_CFG},
	{ADDR_VLEVEL, ADE9078_VLEVEL},
	{ADDR_DICOEFF, ADE9078_DICOEFF},
	{ADDR_EGY_TIME, ADE9078_EGY_TIME},
	{ADDR_EP_CFG, ADE9078_EP_CFG},
	{ADDR_RUN, ADE9078_RUN_ON}
};

/*
 * ade9078_spi_write_reg() - ade9078 write register over SPI
 * the data format for communicating with the ade9078 over SPI
 * is very specific and can access both 32bit and 16bit registers
 * @context:	void pointer to the SPI device
 * @reg:	address of the of desired register
 * @val:	value to be written to the ade9078
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
			.bits_per_word = 8,
			.len = 6,
		},
	};

	addr = FIELD_PREP(ADE9078_REG_ADDR_MASK, reg);

	put_unaligned_be16(addr, st->tx);
	put_unaligned_be32(val, &st->tx[2]);

	//registers which are 16 bits
	if (reg > 0x480 && reg < 0x4FE) {
		put_unaligned_be16(val, &st->tx[2]);
		xfer[0].len = 4;
	}

	ret = spi_sync_transfer(st->spi, xfer, ARRAY_SIZE(xfer));
	if (ret) {
		dev_err(&st->spi->dev, "problem when writing register 0x%x",
			reg);
	}

	return ret;
}

/*
 * ade9078_spi_write_reg() - ade9078 read register over SPI
 * the data format for communicating with the ade9078 over SPI
 * is very specific and can access both 32bit and 16bit registers
 * @context:	void pointer to the SPI device
 * @reg:	address of the of desired register
 * @val:	value to be read to the ade9078
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
			.bits_per_word = 8,
			.len = 2,
		},
		{
			.rx_buf = st->rx,
			.bits_per_word = 8,
			.len = 6,
		},
	};

	addr = FIELD_PREP(ADE9078_REG_ADDR_MASK, reg) |
	       ADE9078_REG_READ_BIT_MASK;

	put_unaligned_be16(addr, st->tx);

	//registers which are 16 bits
	if (reg > 0x480 && reg < 0x4FE)
		xfer[1].len = 4;

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

/*
 * ade9078_en_wfb() - enables or disables the WFBuffer in the ADE9078
 * @st:ade9078 device data
 * @state:	true for enabled; false for disabled
 */
static int ade9078_en_wfb(struct ade9078_state *st, bool state)
{
	return regmap_update_bits(st->regmap, ADDR_WFB_CFG, BIT_MASK(4),
				  state ? BIT_MASK(4) : 0);
}

/*
 * ade9078_update_mask0() - updates interrupt mask0 and resets all of the status
 * register 0
 * @st:ade9078 device data
 */
static int ade9078_update_mask0(struct ade9078_state *st)
{
	int ret;

	ret = regmap_write(st->regmap, ADDR_STATUS0, GENMASK(31, 0));

	if (ret)
		return ret;

	return regmap_write(st->regmap, ADDR_MASK0, st->irq0_bits);
}

/*
 * ade9078_iio_push_buffer() - reads out the content of the waveform buffer and
 * pushes it to the IIO buffer.
 * @st:		ade9078 device data
 */
static int ade9078_iio_push_buffer(struct ade9078_state *st)
{
	u32 i;
	int ret;

	ret = spi_sync(st->spi, &st->spi_msg);
	if (ret) {
		dev_err(&st->spi->dev, "SPI fail in trigger handler");
		return ret;
	}

	for (i = 0; i <= ADE9078_WFB_FULL_BUFF_NR_SAMPLES; i++)
		iio_push_to_buffers(st->indio_dev, &st->rx_buff.word[i]);

	return 0;
}

/*
 * ade9078_irq0_thread() - Thread for IRQ0. It reads Status register 0 and
 * checks for the IRQ activation. This is configured to acquire samples in to
 * the IC buffer and dump it in to the iio_buffer according to Stop When Buffer
 * Is Full Mode, Stop Filling on Trigger and Capture Around Trigger from the
 * ADE9078 Datasheet
 */
static irqreturn_t ade9078_irq0_thread(int irq, void *data)
{
	struct ade9078_state *st = data;
	u32 status;
	u32 handled_irq = 0;
	unsigned long *irq0_bits = (unsigned long *)&st->irq0_bits;
	int ret;

	ret = regmap_read(st->regmap, ADDR_STATUS0, &status);
	if (ret) {
		dev_err(&st->spi->dev, "IRQ0 read status fail");
		goto irq0_done;
	}

	if ((status & ADE9078_ST0_PAGE_FULL_BIT) &&
	    (st->irq0_bits & ADE9078_ST0_PAGE_FULL_BIT)) {
		//Stop Filling on Trigger and Center Capture Around Trigger
		if (st->wf_mode) {
			ret = regmap_write(st->regmap, ADDR_WFB_TRG_CFG,
					   st->wfb_trg);
			if (ret) {
				dev_err(&st->spi->dev, "IRQ0 WFB write fail");
				goto irq0_done;
			}

			set_bit(ADE9078_ST0_WFB_TRIG, irq0_bits);

		} else {
			//Stop When Buffer Is Full Mode
			ret = ade9078_en_wfb(st, false);
			if (ret) {
				dev_err(&st->spi->dev, "IRQ0 WFB stop fail");
				goto irq0_done;
			}
			ret = ade9078_iio_push_buffer(st);
			if (ret) {
				dev_err(&st->spi->dev, "IRQ0 IIO push fail");
				goto irq0_done;
			}
		}

		//disable Page full interrupt
		clear_bit(ADE9078_ST0_PAGE_FULL, irq0_bits);
		ret = regmap_write(st->regmap, ADDR_MASK0, st->irq0_bits);
		if (ret) {
			dev_err(&st->spi->dev, "IRQ0 MAKS0 write fail");
			goto irq0_done;
		}

		set_bit(ADE9078_ST0_PAGE_FULL, (unsigned long *)&handled_irq);
	}

	if ((status & ADE9078_ST0_WFB_TRIG_BIT) &&
	    (st->irq0_bits & ADE9078_ST0_WFB_TRIG_BIT)) {
		//Stop Filling on Trigger and Center Capture Around Trigger
		ret = ade9078_en_wfb(st, false);
		if (ret) {
			dev_err(&st->spi->dev, "IRQ0 WFB fail");
			goto irq0_done;
		}

		ret = ade9078_iio_push_buffer(st);
		if (ret) {
			dev_err(&st->spi->dev, "IRQ0 IIO push fail @ WFB TRIG");
			goto irq0_done;
		}

		set_bit(ADE9078_ST0_WFB_TRIG, (unsigned long *)&handled_irq);
	}

	ret = regmap_write(st->regmap, ADDR_STATUS0, handled_irq);
	if (ret)
		dev_err(&st->spi->dev, "IRQ0 write status fail");

irq0_done:
	return IRQ_HANDLED;
}

/*
 * ade9078_irq1_thread() - Thread for IRQ1. It reads Status register 1 and
 * checks for the IRQ activation. This thread handles the reset condition and
 * the zero-crossing conditions for all 3 phases on Voltage and Current
 */
static irqreturn_t ade9078_irq1_thread(int irq, void *data)
{
	struct ade9078_state *st = data;
	struct iio_dev *indio_dev = st->indio_dev;
	u32 result;
	u32 status;
	u32 tmp;
	s64 timestamp = iio_get_time_ns(indio_dev);
	unsigned int bit = ADE9078_ST1_CROSSING_FIRST;
	unsigned long *irq1_bits = (unsigned long *)&st->irq1_bits;
	unsigned long *irq1_status = (unsigned long *)&st->irq1_status;
	int ret;

	//reset
	if (!st->rst_done) {
		ret = regmap_read(st->regmap, ADDR_STATUS1, &result);
		if (ret)
			return ret;
		if (result & ADE9078_ST1_RSTDONE_BIT)
			st->rst_done = true;
		else
			dev_err(&st->spi->dev, "Error testing reset done");
		goto irq1_done;
	}

	ret = regmap_read(st->regmap, ADDR_STATUS1, &status);
	if (ret)
		return ret;

	//crossings
	for_each_set_bit_from(bit, irq1_bits, ADE9078_ST1_CROSSING_DEPTH) {
		tmp = status & BIT(bit);
		if (tmp == ADE9078_ST1_ZXVA_BIT) {
			iio_push_event(indio_dev,
				       IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE,
							    ADE9078_PHASE_A_NR,
							    IIO_EV_TYPE_THRESH,
							    IIO_EV_DIR_EITHER),
				       timestamp);
			set_bit(bit, irq1_status);
		}
		if (tmp == ADE9078_ST1_ZXTOVA_BIT) {
			iio_push_event(indio_dev,
				       IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE,
							    ADE9078_PHASE_A_NR,
							    IIO_EV_TYPE_THRESH,
							    IIO_EV_DIR_EITHER),
				       timestamp);
			set_bit(bit, irq1_status);
		}
		if (tmp == ADE9078_ST1_ZXIA_BIT) {
			iio_push_event(indio_dev,
				       IIO_UNMOD_EVENT_CODE(IIO_CURRENT,
							    ADE9078_PHASE_A_NR,
							    IIO_EV_TYPE_THRESH,
							    IIO_EV_DIR_EITHER),
				       timestamp);
			set_bit(bit, irq1_status);
		}
		if (tmp == ADE9078_ST1_ZXVB_BIT) {
			iio_push_event(indio_dev,
				       IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE,
							    ADE9078_PHASE_B_NR,
							    IIO_EV_TYPE_THRESH,
							    IIO_EV_DIR_EITHER),
				       timestamp);
			set_bit(bit, irq1_status);
		}
		if (tmp == ADE9078_ST1_ZXTOVB_BIT) {
			iio_push_event(indio_dev,
				       IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE,
							    ADE9078_PHASE_B_NR,
							    IIO_EV_TYPE_THRESH,
							    IIO_EV_DIR_EITHER),
				       timestamp);
			set_bit(bit, irq1_status);
		}
		if (tmp == ADE9078_ST1_ZXIB_BIT) {
			iio_push_event(indio_dev,
				       IIO_UNMOD_EVENT_CODE(IIO_CURRENT,
							    ADE9078_PHASE_B_NR,
							    IIO_EV_TYPE_THRESH,
							    IIO_EV_DIR_EITHER),
				       timestamp);
			set_bit(bit, irq1_status);
		}
		if (tmp == ADE9078_ST1_ZXVC_BIT) {
			iio_push_event(indio_dev,
				       IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE,
							    ADE9078_PHASE_C_NR,
							    IIO_EV_TYPE_THRESH,
							    IIO_EV_DIR_EITHER),
				       timestamp);
			set_bit(bit, irq1_status);
		}
		if (tmp == ADE9078_ST1_ZXTOVC_BIT) {
			iio_push_event(indio_dev,
				       IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE,
							    ADE9078_PHASE_C_NR,
							    IIO_EV_TYPE_THRESH,
							    IIO_EV_DIR_EITHER),
				       timestamp);
			set_bit(bit, irq1_status);
		}
		if (tmp == ADE9078_ST1_ZXIC_BIT) {
			iio_push_event(indio_dev,
				       IIO_UNMOD_EVENT_CODE(IIO_CURRENT,
							    ADE9078_PHASE_C_NR,
							    IIO_EV_TYPE_THRESH,
							    IIO_EV_DIR_EITHER),
				       timestamp);
			set_bit(bit, irq1_status);
		}
	}

irq1_done:
	return IRQ_HANDLED;
}

/*
 * ade9078_configure_scan() - sets up the transfer parameters
 * as well as the tx and rx buffers
 * @indio_dev:	the IIO device
 */
static int ade9078_configure_scan(struct iio_dev *indio_dev)
{
	struct ade9078_state *st = iio_priv(indio_dev);
	u16 addr;

	indio_dev->modes |= INDIO_BUFFER_TRIGGERED;

	addr = FIELD_PREP(ADE9078_REG_ADDR_MASK, ADDR_WF_BUFF) |
	       ADE9078_REG_READ_BIT_MASK;

	put_unaligned_be16(addr, st->tx_buff);

	st->xfer[0].tx_buf = &st->tx_buff[0];
	st->xfer[0].bits_per_word = 8;
	st->xfer[0].len = 2;

	st->xfer[1].rx_buf = &st->rx_buff.byte[0];
	st->xfer[1].bits_per_word = 8;
	st->xfer[1].len = ADE9078_WFB_FULL_BUFF_SIZE;

	spi_message_init_with_transfers(&st->spi_msg, st->xfer, 2);
	return 0;
}

/*
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
	int measured;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;

		ret = regmap_read(st->regmap, chan->address, &measured);

		iio_device_release_direct_mode(indio_dev);
		*val = measured;

		return ret ?: IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_CURRENT:
			if (chan->address >= ADDR_AI_PCF &&
			    chan->address <= ADDR_CI_PCF){
				*val = 1;
				*val2 = ADE9078_PCF_FULL_SCALE_CODES;
				return IIO_VAL_FRACTIONAL;
			}
			if (chan->address >= ADDR_AIRMS &&
			    chan->address <= ADDR_CIRMS){
				*val = 1;
				*val2 = ADE9078_RMS_FULL_SCALE_CODES;
				return IIO_VAL_FRACTIONAL;
			}
			break;
		case IIO_VOLTAGE:
			if (chan->address >= ADDR_AV_PCF &&
			    chan->address <= ADDR_CV_PCF){
				*val = 1;
				*val2 = ADE9078_PCF_FULL_SCALE_CODES;
				return IIO_VAL_FRACTIONAL;
			}
			if (chan->address >= ADDR_AVRMS &&
			    chan->address <= ADDR_CVRMS){
				*val = 1;
				*val2 = ADE9078_RMS_FULL_SCALE_CODES;
				return IIO_VAL_FRACTIONAL;
			}
			break;
		case IIO_POWER:
			*val = 1;
			*val2 = ADE9000_WATT_FULL_SCALE_CODES;
			return IIO_VAL_FRACTIONAL;

		default:
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/*
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
	u32 addr = 0xFFFFF;
	unsigned long tmp;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_OFFSET:
		switch (chan->type) {
		case IIO_CURRENT:
			addr = PHASE_ADDR_ADJUST(ADDR_AIRMSOS, chan->channel);
			break;
		case IIO_VOLTAGE:
			addr = PHASE_ADDR_ADJUST(ADDR_AVRMSOS, chan->channel);
			break;
		case IIO_POWER:
			tmp = chan->address;
			clear_bit(ADE9078_PHASE_B_POS, &tmp);
			clear_bit(ADE9078_PHASE_C_POS, &tmp);

			switch (tmp) {
			case ADDR_AWATT:
				addr = PHASE_ADDR_ADJUST(ADDR_AWATTOS,
							 chan->channel);
				break;
			case ADDR_AVAR:
				addr = PHASE_ADDR_ADJUST(ADDR_AVAROS,
							 chan->channel);
				break;
			case ADDR_AFVAR:
				addr = PHASE_ADDR_ADJUST(ADDR_AFVAROS,
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
			addr = PHASE_ADDR_ADJUST(ADDR_AIGAIN, chan->channel);
			break;
		case IIO_VOLTAGE:
			addr = PHASE_ADDR_ADJUST(ADDR_AVGAIN, chan->channel);
			break;
		case IIO_POWER:
			addr = PHASE_ADDR_ADJUST(ADDR_APGAIN, chan->channel);
			break;
		default:
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	ret = regmap_write(st->regmap, addr, val);
	return ret;
}

/*
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

/*
 * ade9078_write_event_config() - IIO event configure to enable zero-crossing
 * and zero-crossing timeout on voltage and current for each phases. These
 * events will also influence the trigger conditions for the buffer capture.
 */
static int ade9078_write_event_config(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan,
				      enum iio_event_type type,
				      enum iio_event_direction dir,
				      int state)
{
	struct ade9078_state *st = iio_priv(indio_dev);
	u32 number;
	unsigned long *irq1_bits = (unsigned long *)&st->irq1_bits;
	unsigned long *irq1_status = (unsigned long *)&st->irq1_status;
	unsigned long *wfb_trg = (unsigned long *)&st->wfb_trg;
	int ret;

	number = chan->channel;

	switch (number) {
	case ADE9078_PHASE_A_NR:
		if (chan->type == IIO_VOLTAGE) {
			if (state) {
				set_bit(ADE9078_ST1_ZXVA, irq1_bits);
				set_bit(ADE9078_ST1_ZXTOVA, irq1_bits);
				set_bit(ADE9078_WFB_TRG_ZXVA, wfb_trg);
			} else {
				clear_bit(ADE9078_ST1_ZXVA, irq1_bits);
				clear_bit(ADE9078_ST1_ZXTOVA, irq1_bits);
				clear_bit(ADE9078_ST1_ZXVA, irq1_status);
				clear_bit(ADE9078_ST1_ZXTOVA, irq1_status);
				clear_bit(ADE9078_WFB_TRG_ZXVA, wfb_trg);
			}
		} else if (chan->type == IIO_CURRENT) {
			if (state) {
				set_bit(ADE9078_ST1_ZXIA, irq1_bits);
				set_bit(ADE9078_WFB_TRG_ZXIA, wfb_trg);
			} else {
				clear_bit(ADE9078_ST1_ZXIA, irq1_bits);
				clear_bit(ADE9078_ST1_ZXIA, irq1_status);
				clear_bit(ADE9078_WFB_TRG_ZXIA, wfb_trg);
			}
		}
		break;
	case ADE9078_PHASE_B_NR:
		if (chan->type == IIO_VOLTAGE) {
			if (state) {
				set_bit(ADE9078_ST1_ZXVB, irq1_bits);
				set_bit(ADE9078_ST1_ZXTOVB, irq1_bits);
				set_bit(ADE9078_WFB_TRG_ZXVB, wfb_trg);
			} else {
				clear_bit(ADE9078_ST1_ZXVB, irq1_bits);
				clear_bit(ADE9078_ST1_ZXTOVB, irq1_bits);
				clear_bit(ADE9078_ST1_ZXVB, irq1_status);
				clear_bit(ADE9078_ST1_ZXTOVB, irq1_status);
				clear_bit(ADE9078_WFB_TRG_ZXVB, wfb_trg);
			}
		} else if (chan->type == IIO_CURRENT) {
			if (state) {
				set_bit(ADE9078_ST1_ZXIB, irq1_bits);
				set_bit(ADE9078_WFB_TRG_ZXIB, wfb_trg);
			} else {
				clear_bit(ADE9078_ST1_ZXIB, irq1_bits);
				clear_bit(ADE9078_ST1_ZXIB, irq1_status);
				clear_bit(ADE9078_WFB_TRG_ZXIB, wfb_trg);
			}
		}
		break;
	case ADE9078_PHASE_C_NR:
		if (chan->type == IIO_VOLTAGE) {
			if (state) {
				set_bit(ADE9078_ST1_ZXVC, irq1_bits);
				set_bit(ADE9078_ST1_ZXTOVC, irq1_bits);
				set_bit(ADE9078_WFB_TRG_ZXVC, wfb_trg);
			} else {
				clear_bit(ADE9078_ST1_ZXVC, irq1_bits);
				clear_bit(ADE9078_ST1_ZXTOVC, irq1_bits);
				clear_bit(ADE9078_ST1_ZXVC, irq1_status);
				clear_bit(ADE9078_ST1_ZXTOVC, irq1_status);
				clear_bit(ADE9078_WFB_TRG_ZXVC, wfb_trg);
			}
		} else if (chan->type == IIO_CURRENT) {
			if (state) {
				set_bit(ADE9078_ST1_ZXIC, irq1_bits);
				set_bit(ADE9078_WFB_TRG_ZXIC, wfb_trg);
			} else {
				clear_bit(ADE9078_ST1_ZXIC, irq1_bits);
				clear_bit(ADE9078_ST1_ZXIC, irq1_status);
				clear_bit(ADE9078_WFB_TRG_ZXIC, wfb_trg);
			}
		}
		break;
	default:
		return -EINVAL;
	}

	ret = regmap_write(st->regmap, ADDR_STATUS1, GENMASK(31, 0));
	if (ret)
		return ret;

	return regmap_write(st->regmap, ADDR_MASK1, st->irq1_bits);
}

/*
 * ade9078_read_event_vlaue() - Outputs the result of the zero-crossing for
 * voltage and current for each phase.
 * Result:
 * 0 - if crossing event not set
 * 1 - if crossing event occurred
 * -1 - if crossing timeout (only for Voltages)
 */
static int ade9078_read_event_vlaue(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    enum iio_event_type type,
				    enum iio_event_direction dir,
				    enum iio_event_info info,
				    int *val, int *val2)
{
	struct ade9078_state *st = iio_priv(indio_dev);
	u32 number;
	u32 status1 = st->irq1_status;
	unsigned long handeled_irq = 0;
	unsigned long *irq1_status = (unsigned long *)&st->irq1_status;
	int ret;

	*val = 0;

	number = chan->channel;
	switch (number) {
	case ADE9078_PHASE_A_NR:
		if (chan->type == IIO_VOLTAGE) {
			if (status1 & ADE9078_ST1_ZXVA_BIT) {
				*val = 1;
				clear_bit(ADE9078_ST1_ZXVA, irq1_status);
				set_bit(ADE9078_ST1_ZXVA, &handeled_irq);
			} else if (status1 & ADE9078_ST1_ZXTOVA_BIT) {
				*val = -1;
				clear_bit(ADE9078_ST1_ZXTOVA, irq1_status);
				set_bit(ADE9078_ST1_ZXTOVA, &handeled_irq);
			}
			if (!(st->irq1_bits & ADE9078_ST1_ZXTOVA_BIT))
				*val = 0;
		} else if (chan->type == IIO_CURRENT) {
			if (status1 & ADE9078_ST1_ZXIA_BIT) {
				*val = 1;
				clear_bit(ADE9078_ST1_ZXIA, irq1_status);
				set_bit(ADE9078_ST1_ZXIA, &handeled_irq);
			}
		}
		break;
	case ADE9078_PHASE_B_NR:
		if (chan->type == IIO_VOLTAGE) {
			if (status1 & ADE9078_ST1_ZXVB_BIT) {
				*val = 1;
				clear_bit(ADE9078_ST1_ZXVB, irq1_status);
				set_bit(ADE9078_ST1_ZXVB, &handeled_irq);
			} else if (status1 & ADE9078_ST1_ZXTOVB_BIT) {
				*val = -1;
				clear_bit(ADE9078_ST1_ZXTOVB, irq1_status);
				set_bit(ADE9078_ST1_ZXTOVB, &handeled_irq);
			}
			if (!(st->irq1_bits & ADE9078_ST1_ZXTOVB_BIT))
				*val = 0;
		} else if (chan->type == IIO_CURRENT) {
			if (status1 & ADE9078_ST1_ZXIB_BIT) {
				*val = 1;
				clear_bit(ADE9078_ST1_ZXIB, irq1_status);
				set_bit(ADE9078_ST1_ZXIB, &handeled_irq);
			}
		}
		break;
	case ADE9078_PHASE_C_NR:
		if (chan->type == IIO_VOLTAGE) {
			if (status1 & ADE9078_ST1_ZXVC_BIT) {
				*val = 1;
				clear_bit(ADE9078_ST1_ZXVC, irq1_status);
				set_bit(ADE9078_ST1_ZXVC, &handeled_irq);
			} else if (status1 & ADE9078_ST1_ZXTOVC_BIT) {
				*val = -1;
				clear_bit(ADE9078_ST1_ZXTOVC, irq1_status);
				set_bit(ADE9078_ST1_ZXTOVC, &handeled_irq);
			}
			if (!(st->irq1_bits & ADE9078_ST1_ZXTOVC_BIT))
				*val = 0;
		} else if (chan->type == IIO_CURRENT) {
			if (status1 & ADE9078_ST1_ZXIC_BIT) {
				*val = 1;
				clear_bit(ADE9078_ST1_ZXIC, irq1_status);
				set_bit(ADE9078_ST1_ZXIC, &handeled_irq);
			}
		}
		break;
	default:
		return -EINVAL;
	}

	ret = regmap_write(st->regmap, ADDR_STATUS1, (u32)handeled_irq);
	if (ret)
		return ret;

	return IIO_VAL_INT;
}

/*
 * ade9078_config_wfb() - reads the ade9078 node and configures the wave form
 * buffer based on the options set. Additionally is reads the active scan mask
 * in order to set the input data of the buffer. There are only a few available
 * input configurations permitted by the IC, any unpermitted configuration will
 * result in all channels being active.
 * @indio_dev:	the IIO device
 */
static int ade9078_config_wfb(struct iio_dev *indio_dev)
{
	struct ade9078_state *st = iio_priv(indio_dev);
	u32 wfg_cfg_val = 0;
	u32 tmp;
	int ret;

	bitmap_to_arr32(&wfg_cfg_val, indio_dev->active_scan_mask,
			indio_dev->masklength);

	switch (wfg_cfg_val) {
	case ADE9078_SCAN_POS_IA | ADE9078_SCAN_POS_VA:
		wfg_cfg_val = 0x1;
		break;
	case ADE9078_SCAN_POS_IB | ADE9078_SCAN_POS_VB:
		wfg_cfg_val = 0x2;
		break;
	case ADE9078_SCAN_POS_IC | ADE9078_SCAN_POS_VC:
		wfg_cfg_val = 0x3;
		break;
	case ADE9078_SCAN_POS_IA:
		wfg_cfg_val = 0x8;
		break;
	case ADE9078_SCAN_POS_VA:
		wfg_cfg_val = 0x9;
		break;
	case ADE9078_SCAN_POS_IB:
		wfg_cfg_val = 0xA;
		break;
	case ADE9078_SCAN_POS_VB:
		wfg_cfg_val = 0xB;
		break;
	case ADE9078_SCAN_POS_IC:
		wfg_cfg_val = 0xC;
		break;
	case ADE9078_SCAN_POS_VC:
		wfg_cfg_val = 0xD;
		break;
	default:
		wfg_cfg_val = 0x0;
		break;
	}

	ret = of_property_read_u32((&st->spi->dev)->of_node, "adi,wf-cap-sel",
				   &tmp);
	if (ret) {
		dev_err(&st->spi->dev, "Failed to get wf-cap-sel: %d\n", ret);
		return ret;
	}
	wfg_cfg_val |= FIELD_PREP(ADE9078_WF_CAP_SEL_MASK, tmp);

	ret = of_property_read_u32((&st->spi->dev)->of_node, "adi,wf-mode",
				   &tmp);
	if (ret) {
		dev_err(&st->spi->dev, "Failed to get wf-mode: %d\n", ret);
		return ret;
	}
	wfg_cfg_val |= FIELD_PREP(ADE9078_WF_MODE_MASK, tmp);
	st->wf_mode = tmp;

	ret = of_property_read_u32((&st->spi->dev)->of_node, "adi,wf-src",
				   &tmp);
	if (ret) {
		dev_err(&st->spi->dev,
			"Failed to get wf-src: %d\n",
			ret);
		return ret;
	}
	wfg_cfg_val |= FIELD_PREP(ADE9078_WF_SRC_MASK, tmp);

	ret = of_property_read_u32((&st->spi->dev)->of_node, "adi,wf-in-en",
				   &tmp);
	if (ret) {
		dev_err(&st->spi->dev, "Failed to get wf-in-en: %d\n", ret);
		return ret;
	}
	wfg_cfg_val |= FIELD_PREP(ADE9078_WF_IN_EN_MASK, tmp);

	return regmap_write(st->regmap, ADDR_WFB_CFG, wfg_cfg_val);
}

/*
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
	unsigned long *irq0_bits = (unsigned long *)&st->irq0_bits;

	ret = regmap_write(st->regmap, ADDR_WFB_TRG_CFG, 0x0);
	if (ret)
		return ret;

	if (mode == 1 || mode == 0) {
		ret = regmap_write(st->regmap, ADDR_WFB_PG_IRQEN,
				   ADE9078_MODE_0_1_PAGE_BIT);
		if (ret)
			return ret;
	} else if (mode == 2) {
		ret = regmap_write(st->regmap, ADDR_WFB_PG_IRQEN,
				   ADE9078_MODE_2_PAGE_BIT);
		if (ret)
			return ret;
	}

	set_bit(ADE9078_ST0_PAGE_FULL, irq0_bits);

	return 0;
}

/*
 * ade9078_buffer_preenable() - configures the waveform buffer, sets the
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

	switch (st->wf_mode) {
	case 0:
		ret = ade9078_wfb_interrupt_setup(st, 0);
		if (ret)
			return ret;
		break;
	case 1:
		ret = ade9078_wfb_interrupt_setup(st, 1);
		if (ret)
			return ret;
		break;
	case 2:
		ret = ade9078_wfb_interrupt_setup(st, 2);
		if (ret)
			return ret;
		break;
	default:
		return -EINVAL;
	}

	ret = ade9078_update_mask0(st);
	if (ret) {
		dev_err(&st->spi->dev, "Post-enable update mask0 fail");
		return ret;
	}
	ret = ade9078_en_wfb(st, true);
	if (ret) {
		dev_err(&st->spi->dev, "Post-enable wfb enable fail");
		return ret;
	}

	return 0;
}

/*
 * ade9078_buffer_postdisable() - after the iio is disable
 * this will disable the ade9078 internal buffer for acquisition
 * @indio_dev:	the IIO device
 */
static int ade9078_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct ade9078_state *st = iio_priv(indio_dev);
	unsigned long *irq0_bits = (unsigned long *)&st->irq0_bits;
	int ret;

	ret = ade9078_en_wfb(st, false);
	if (ret) {
		dev_err(&st->spi->dev, "Post-disable wfb disable fail");
		return ret;
	}

	ret = regmap_write(st->regmap, ADDR_WFB_TRG_CFG, 0x0);
	if (ret)
		return ret;

	clear_bit(ADE9078_ST0_WFB_TRIG, irq0_bits);
	clear_bit(ADE9078_ST0_PAGE_FULL, irq0_bits);

	ret = ade9078_update_mask0(st);
	if (ret) {
		dev_err(&st->spi->dev, "Post-disable update maks0 fail");
		return ret;
	}

	return 0;
}

/*
 * ade9078_setup_iio_channels() - parses the phase nodes of the device-tree and
 * creates the iio channels based on the active phases in the DT.
 * @st:		ade9078 device data
 */
static int ade9078_setup_iio_channels(struct ade9078_state *st)
{
	struct iio_chan_spec *chan;
	struct fwnode_handle *phase_node = NULL;
	struct device *dev = &st->spi->dev;
	u32 phase_nr;
	u32 chan_size = 0;
	int ret;

	chan = devm_kcalloc(dev,
			    ADE9078_MAX_PHASE_NR *
			    ARRAY_SIZE(ade9078_a_channels),
			    sizeof(*ade9078_a_channels), GFP_KERNEL);
	if (!chan) {
		dev_err(dev, "Unable to allocate ADE9078 channels");
		return -ENOMEM;
	}
	st->indio_dev->num_channels = 0;
	st->indio_dev->channels = chan;

	fwnode_for_each_available_child_node(dev_fwnode(dev), phase_node) {
		ret = fwnode_property_read_u32(phase_node, "reg", &phase_nr);
		if (ret) {
			dev_err(dev, "Could not read channel reg : %d\n", ret);
			return ret;
		}

		switch (phase_nr) {
		case ADE9078_PHASE_A_NR:
			memcpy(chan, ade9078_a_channels,
			       sizeof(ade9078_a_channels));
			chan_size = ARRAY_SIZE(ade9078_a_channels);
			break;
		case ADE9078_PHASE_B_NR:
			memcpy(chan, ade9078_b_channels,
			       sizeof(ade9078_b_channels));
			chan_size = ARRAY_SIZE(ade9078_b_channels);
			break;
		case ADE9078_PHASE_C_NR:
			memcpy(chan, ade9078_c_channels,
			       sizeof(ade9078_c_channels));
			chan_size = ARRAY_SIZE(ade9078_c_channels);
			break;
		default:
			return -EINVAL;
		}

		chan += chan_size;
		st->indio_dev->num_channels += chan_size;
	}

	return 0;
}

/*
 * ade9078_reset() - Reset sequence for the ADE9078, the hardware reset is
 * optional in the DT. When no hardware reset has been declared a software
 * reset is executed
 * @st:		ade9078 device data
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
		ret = regmap_update_bits(st->regmap, ADDR_CONFIG1,
					 ADE9078_SWRST_MASK, ADE9078_SWRST_BIT);
		if (ret)
			return ret;
		usleep_range(80, 100);
	}

	if (!st->rst_done)
		return -EPERM;

	return 0;
}

/*
 * ade9078_setup() - initial register setup of the ade9078
 * @st:		ade9078 device data
 */
static int ade9078_setup(struct ade9078_state *st)
{
	int ret = 0;

	ret = regmap_multi_reg_write(st->regmap, ade9078_reg_sequence,
				     ARRAY_SIZE(ade9078_reg_sequence));
	if (ret)
		return ret;

	msleep_interruptible(2);

	ret = regmap_write(st->regmap, ADDR_STATUS0, GENMASK(31, 0));
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, ADDR_STATUS1, GENMASK(31, 0));
	if (ret)
		return ret;

	return ret;
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
	.reg_read = ade9078_spi_read_reg,
	.reg_write = ade9078_spi_write_reg,
	.zero_flag_mask = true,
};

static int ade9078_probe(struct spi_device *spi)
{
	struct ade9078_state *st;
	struct iio_dev *indio_dev;
	struct regmap *regmap;
	struct iio_buffer *buffer;
	int irq;
	unsigned long irqflags;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev) {
		dev_err(&spi->dev, "Unable to allocate ADE9078 IIO");
		return -ENOMEM;
	}
	st = iio_priv(indio_dev);
	if (!st) {
		dev_err(&spi->dev,
			"Unable to allocate ADE9078 device structure");
		return -ENOMEM;
	}

	st->rx = devm_kcalloc(&spi->dev, ADE9078_RX_DEPTH, sizeof(*st->rx),
			      GFP_KERNEL);
	if (!st->rx)
		return -ENOMEM;

	st->tx = devm_kcalloc(&spi->dev, ADE9078_TX_DEPTH, sizeof(*st->tx),
			      GFP_KERNEL);
	if (!st->tx)
		return -ENOMEM;

	regmap = devm_regmap_init(&spi->dev, NULL, spi, &ade9078_regmap_config);
	if (IS_ERR(regmap))	{
		dev_err(&spi->dev, "Unable to allocate ADE9078 regmap");
		return PTR_ERR(regmap);
	}
	spi_set_drvdata(spi, st);

	irq = of_irq_get_byname((&spi->dev)->of_node, "irq0");
	if (irq < 0) {
		dev_err(&spi->dev, "Unable to find irq0");
		return -EINVAL;
	}
	irqflags = irq_get_trigger_type(irq);
	ret = devm_request_threaded_irq(&spi->dev, irq, NULL,
					ade9078_irq0_thread,
					irqflags | IRQF_ONESHOT,
					KBUILD_MODNAME, st);
	if (ret) {
		dev_err(&spi->dev, "Failed to request threaded irq: %d\n", ret);
		return ret;
	}

	irq = of_irq_get_byname((&spi->dev)->of_node, "irq1");
	if (irq < 0) {
		dev_err(&spi->dev, "Unable to find irq1");
		return -EINVAL;
	}
	irqflags = irq_get_trigger_type(irq);
	ret = devm_request_threaded_irq(&spi->dev, irq, NULL,
					ade9078_irq1_thread,
					irqflags | IRQF_ONESHOT,
					KBUILD_MODNAME, st);
	if (ret) {
		dev_err(&spi->dev, "Failed to request threaded irq: %d\n", ret);
		return ret;
	}

	st->spi = spi;
	st->spi->mode = SPI_MODE_0;
	ret = spi_setup(st->spi);
	if (ret) {
		dev_err(&spi->dev, "Failed spi setup: %d\n", ret);
		return ret;
	}

	indio_dev->dev.parent = &st->spi->dev;
	indio_dev->info = &ade9078_info;
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_SOFTWARE;
	indio_dev->setup_ops = &ade9078_buffer_ops;

	st->regmap = regmap;
	st->indio_dev = indio_dev;
	ade9078_setup_iio_channels(st);
	if (ret) {
		dev_err(&spi->dev, "Failed to set up IIO channels");
		return ret;
	}

	ret = ade9078_configure_scan(indio_dev);
	if (ret)
		return ret;

	buffer = devm_iio_kfifo_allocate(&spi->dev);
	if (!buffer)
		return -ENOMEM;

	iio_device_attach_buffer(indio_dev, buffer);

	ret = devm_iio_device_register(&spi->dev, indio_dev);
	if (ret) {
		dev_err(&spi->dev, "Unable to register IIO device");
		return ret;
	}

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

	return ret;
};

static const struct spi_device_id ade9078_id[] = {
		{"ade9078", 0},
		{}
};
MODULE_DEVICE_TABLE(spi, ade9078_id);

static const struct of_device_id ade9078_of_match[] = {
	{ .compatible = "adi,ade9078" },
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
MODULE_DESCRIPTION("Analog Devices ADE9078 High Performance,Polyphase Energy Metering IC Driver");
MODULE_LICENSE("GPL v2");
