/*
 * ADI-AIM ADI ADC Interface Module
 *
 * Copyright 2012 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 *
 * http://wiki.analog.com/resources/fpga/xilinx/fmc/ad9467
 */

#ifndef ADI_AXI_ADC_H_
#define ADI_AXI_ADC_H_

#define ADI_REG_VERSION		0x0000				/*Version and Scratch Registers */
#define ADI_VERSION(x)		(((x) & 0xffffffff) << 0)	/* RO, Version number. */
#define VERSION_IS(x,y,z)	((x) << 16 | (y) << 8 | (z))
#define ADI_REG_ID		0x0004			 	/*Version and Scratch Registers */
#define ADI_ID(x)		(((x) & 0xffffffff) << 0)   	/* RO, Instance identifier number. */
#define ADI_REG_SCRATCH		0x0008			 	/*Version and Scratch Registers */
#define ADI_SCRATCH(x)		(((x) & 0xffffffff) << 0)	/* RW, Scratch register. */

#define PCORE_VERSION(major, minor, letter) ((major << 16) | (minor << 8) | letter)
#define PCORE_VERSION_MAJOR(version) (version >> 16)
#define PCORE_VERSION_MINOR(version) ((version >> 8) & 0xff)
#define PCORE_VERSION_LETTER(version) (version & 0xff)

/* ADC COMMON */

#define ADI_REG_RSTN			0x0040
#define ADI_RSTN				(1 << 0)
#define ADI_MMCM_RSTN 			(1 << 1)

#define ADI_REG_CNTRL			0x0044
#define ADI_R1_MODE			(1 << 2)
#define ADI_DDR_EDGESEL			(1 << 1)
#define ADI_PIN_MODE			(1 << 0)

#define ADI_REG_CLK_FREQ			0x0054
#define ADI_CLK_FREQ(x)			(((x) & 0xFFFFFFFF) << 0)
#define ADI_TO_CLK_FREQ(x)		(((x) >> 0) & 0xFFFFFFFF)

#define ADI_REG_CLK_RATIO		0x0058
#define ADI_CLK_RATIO(x)			(((x) & 0xFFFFFFFF) << 0)
#define ADI_TO_CLK_RATIO(x)		(((x) >> 0) & 0xFFFFFFFF)

#define ADI_REG_STATUS			0x005C
#define ADI_MUX_PN_ERR			(1 << 3)
#define ADI_MUX_PN_OOS			(1 << 2)
#define ADI_MUX_OVER_RANGE		(1 << 1)
#define ADI_STATUS			(1 << 0)

#define ADI_REG_DELAY_CNTRL		0x0060
#define ADI_DELAY_SEL			(1 << 17)
#define ADI_DELAY_RWN			(1 << 16)
#define ADI_DELAY_ADDRESS(x)		(((x) & 0xFF) << 8)
#define ADI_TO_DELAY_ADDRESS(x)		(((x) >> 8) & 0xFF)
#define ADI_DELAY_WDATA(x)		(((x) & 0x1F) << 0)
#define ADI_TO_DELAY_WDATA(x)		(((x) >> 0) & 0x1F)

#define ADI_REG_DELAY_STATUS		0x0064
#define ADI_DELAY_LOCKED			(1 << 9)
#define ADI_DELAY_STATUS			(1 << 8)
#define ADI_DELAY_RDATA(x)		(((x) & 0x1F) << 0)
#define ADI_TO_DELAY_RDATA(x)		(((x) >> 0) & 0x1F)

#define ADI_REG_DRP_CNTRL		0x0070
#define ADI_DRP_SEL			(1 << 29)
#define ADI_DRP_RWN			(1 << 28)
#define ADI_DRP_ADDRESS(x)		(((x) & 0xFFF) << 16)
#define ADI_TO_DRP_ADDRESS(x)		(((x) >> 16) & 0xFFF)
#define ADI_DRP_WDATA(x)			(((x) & 0xFFFF) << 0)
#define ADI_TO_DRP_WDATA(x)		(((x) >> 0) & 0xFFFF)

#define ADI_REG_DRP_STATUS		0x0074
#define ADI_DRP_STATUS			(1 << 16)
#define ADI_DRP_RDATA(x)			(((x) & 0xFFFF) << 0)
#define ADI_TO_DRP_RDATA(x)		(((x) >> 0) & 0xFFFF)

#define ADI_REG_DMA_CNTRL		0x0080
#define ADI_DMA_STREAM			(1 << 1)
#define ADI_DMA_START			(1 << 0)

#define ADI_REG_DMA_COUNT		0x0084
#define ADI_DMA_COUNT(x)			(((x) & 0xFFFFFFFF) << 0)
#define ADI_TO_DMA_COUNT(x)		(((x) >> 0) & 0xFFFFFFFF)

#define ADI_REG_DMA_STATUS		0x0088
#define ADI_DMA_OVF			(1 << 2)
#define ADI_DMA_UNF			(1 << 1)
#define ADI_DMA_STATUS			(1 << 0)

#define ADI_REG_DMA_BUSWIDTH		0x008C
#define ADI_DMA_BUSWIDTH(x)		(((x) & 0xFFFFFFFF) << 0)
#define ADI_TO_DMA_BUSWIDTH(x)		(((x) >> 0) & 0xFFFFFFFF)

#define ADI_REG_USR_CNTRL_1		0x00A0
#define ADI_USR_CHANMAX(x)		(((x) & 0xFF) << 0)
#define ADI_TO_USR_CHANMAX(x)		(((x) >> 0) & 0xFF)

/* ADC CHANNEL */

#define ADI_REG_CHAN_CNTRL(c)		(0x0400 + (c) * 0x40)
#define ADI_PN_SEL			(1 << 10) /* !v8.0 */
#define ADI_IQCOR_ENB			(1 << 9)
#define ADI_DCFILT_ENB			(1 << 8)
#define ADI_FORMAT_SIGNEXT		(1 << 6)
#define ADI_FORMAT_TYPE			(1 << 5)
#define ADI_FORMAT_ENABLE		(1 << 4)
#define ADI_PN23_TYPE			(1 << 1) /* !v8.0 */
#define ADI_ENABLE			(1 << 0)

#define ADI_REG_CHAN_STATUS(c)		(0x0404 + (c) * 0x40)
#define ADI_PN_ERR			(1 << 2)
#define ADI_PN_OOS			(1 << 1)
#define ADI_OVER_RANGE			(1 << 0)

#define ADI_REG_CHAN_CNTRL_1(c)		(0x0410 + (c) * 0x40)
#define ADI_DCFILT_OFFSET(x)		(((x) & 0xFFFF) << 16)
#define ADI_TO_DCFILT_OFFSET(x)		(((x) >> 16) & 0xFFFF)
#define ADI_DCFILT_COEFF(x)		(((x) & 0xFFFF) << 0)
#define ADI_TO_DCFILT_COEFF(x)		(((x) >> 0) & 0xFFFF)

#define ADI_REG_CHAN_CNTRL_2(c)		(0x0414 + (c) * 0x40)
#define ADI_IQCOR_COEFF_1(x)		(((x) & 0xFFFF) << 16)
#define ADI_TO_IQCOR_COEFF_1(x)		(((x) >> 16) & 0xFFFF)
#define ADI_IQCOR_COEFF_2(x)		(((x) & 0xFFFF) << 0)
#define ADI_TO_IQCOR_COEFF_2(x)		(((x) >> 0) & 0xFFFF)

#define ADI_REG_CHAN_CNTRL_3(c)		(0x0418 + (c) * 0x40) /* v8.0 */
#define ADI_ADC_PN_SEL(x)		(((x) & 0xF) << 16)
#define ADI_TO_ADC_PN_SEL(x)		(((x) >> 16) & 0xF)
#define ADI_ADC_DATA_SEL(x)		(((x) & 0xF) << 0)
#define ADI_TO_ADC_DATA_SEL(x)		(((x) >> 0) & 0xF)

enum adc_pn_sel {
	ADC_PN9 = 0,
	ADC_PN23A = 1,
	ADC_PN7 = 4,
	ADC_PN15 = 5,
	ADC_PN23 = 6,
	ADC_PN31 = 7,
	ADC_PN_CUSTOM = 9,
	ADC_PN_END = 10,
};

enum adc_data_sel {
	ADC_DATA_SEL_NORM,
	ADC_DATA_SEL_LB, /* DAC loopback */
	ADC_DATA_SEL_RAMP, /* TBD */
};

#define ADI_REG_CHAN_USR_CNTRL_1(c)		(0x0420 + (c) * 0x40)
#define ADI_USR_DATATYPE_BE			(1 << 25)
#define ADI_USR_DATATYPE_SIGNED			(1 << 24)
#define ADI_USR_DATATYPE_SHIFT(x)		(((x) & 0xFF) << 16)
#define ADI_TO_USR_DATATYPE_SHIFT(x)		(((x) >> 16) & 0xFF)
#define ADI_USR_DATATYPE_TOTAL_BITS(x)		(((x) & 0xFF) << 8)
#define ADI_TO_USR_DATATYPE_TOTAL_BITS(x)	(((x) >> 8) & 0xFF)
#define ADI_USR_DATATYPE_BITS(x)			(((x) & 0xFF) << 0)
#define ADI_TO_USR_DATATYPE_BITS(x)		(((x) >> 0) & 0xFF)

#define ADI_REG_CHAN_USR_CNTRL_2(c)		(0x0424 + (c) * 0x40)
#define ADI_USR_DECIMATION_M(x)			(((x) & 0xFFFF) << 16)
#define ADI_TO_USR_DECIMATION_M(x)		(((x) >> 16) & 0xFFFF)
#define ADI_USR_DECIMATION_N(x)			(((x) & 0xFFFF) << 0)
#define ADI_TO_USR_DECIMATION_N(x)		(((x) >> 0) & 0xFFFF)

#define ADI_REG_ADC_DP_DISABLE 			0x00C0

/*
 * ADI High-Speed ADC common spi interface registers
 * See Application-Note AN-877
 */

#define ADC_REG_CHIP_PORT_CONF		0x00
#define ADC_REG_CHIP_ID			0x01
#define ADC_REG_CHIP_GRADE		0x02
#define ADC_REG_CHAN_INDEX		0x05
#define ADC_REG_TRANSFER		0xFF
#define ADC_REG_MODES			0x08
#define ADC_REG_TEST_IO			0x0D
#define ADC_REG_ADC_INPUT		0x0F
#define ADC_REG_OFFSET			0x10
#define ADC_REG_OUTPUT_MODE		0x14
#define ADC_REG_OUTPUT_ADJUST		0x15
#define ADC_REG_OUTPUT_PHASE		0x16
#define ADC_REG_OUTPUT_DELAY		0x17
#define ADC_REG_VREF			0x18
#define ADC_REG_ANALOG_INPUT		0x2C

/* ADC_REG_TRANSFER */
#define TRANSFER_SYNC			0x1

/* ADC_REG_TEST_IO */
#define TESTMODE_OFF			0x0
#define TESTMODE_MIDSCALE_SHORT		0x1
#define TESTMODE_POS_FULLSCALE		0x2
#define TESTMODE_NEG_FULLSCALE		0x3
#define TESTMODE_ALT_CHECKERBOARD	0x4
#define TESTMODE_PN23_SEQ		0x5
#define TESTMODE_PN9_SEQ			0x6
#define TESTMODE_ONE_ZERO_TOGGLE		0x7
#define TESTMODE_RAMP			0xF

/* ADC_REG_OUTPUT_MODE */
#define OUTPUT_MODE_OFFSET_BINARY	0x0
#define OUTPUT_MODE_TWOS_COMPLEMENT	0x1
#define OUTPUT_MODE_GRAY_CODE		0x2

/* ADC_REG_OUTPUT_PHASE */
#define OUTPUT_EVEN_ODD_MODE_EN		0x20
#define INVERT_DCO_CLK			0x80

/* ADC_REG_OUTPUT_DELAY */
#define DCO_DELAY_ENABLE 		0x80


/*
 * Analog Devices AD9467 16-Bit, 200/250 MSPS ADC
 */

#define CHIPID_AD9467			0x50
#define AD9467_DEF_OUTPUT_MODE		0x08
#define AD9467_REG_VREF_MASK		0x0F

/*
 * Analog Devices AD9643 Dual 14-Bit, 170/210/250 MSPS ADC
 */

#define CHIPID_AD9643			0x82
#define AD9643_REG_VREF_MASK		0x1F
#define AD9643_DEF_OUTPUT_MODE		0x00

/*
 * Analog Devices AD9250 Dual 14-Bit, 170/250 MSPS ADC, JESD204B
 */

#define CHIPID_AD9250			0xB9
#define AD9250_REG_VREF_MASK		0x1F
#define AD9250_DEF_OUTPUT_MODE		0x00
#define AD9250_AXIADC_PCORE_DATA_SEL	0x28
#define AD9250_AXIADC_PCORE_DATA_SEL_F	(1 << 0)

/*
 * Analog Devices AD9683 14-Bit, 170/250 MSPS ADC, JESD204B
 */

#define CHIPID_AD9683			0xC3
#define AD9683_DEF_OUTPUT_MODE		0x00
#define AD9683_AXIADC_PCORE_DATA_SEL	0x28
#define AD9683_SIGNEXTEND		(1 << 0)

/*
 * Analog Devices AD9625 12-Bit, 2500 MSPS ADC, JESD204B
 */

#define CHIPID_AD9625			0x41
#define AD9625_DEF_OUTPUT_MODE		0x00
#define AD9625_AXIADC_PCORE_DATA_SEL	0x24
#define AD9625_SIGNEXTEND		(1 << 0)

/*
 * Analog Devices AD9265 16-Bit, 125/105/80 MSPS ADC
 */

#define CHIPID_AD9265			0x64
#define AD9265_DEF_OUTPUT_MODE		0x40
#define AD9265_REG_VREF_MASK		0xC0

/*
 * Analog Devices AD9434 12-Bit, 370/500 MSPS ADC
 */

#define CHIPID_AD9434			0x6A
#define AD9434_DEF_OUTPUT_MODE		0x00
#define AD9434_REG_VREF_MASK		0xC0

/*
 * Analog Devices AD9680
 */

#define AD9680_REG_CHIP_ID_LOW		0x004
#define AD9680_REG_CHIP_ID_HIGH		0x005
#define AD9680_REG_DEVICE_INDEX		0x008
#define AD9680_REG_INPUT_FS_RANGE	0x025

#define AD9680_REG_OUTPUT_MODE		0x561
#define AD9680_REG_TEST_MODE		0x550

#define CHIPID_AD9680			0xC5
#define AD9680_DEF_OUTPUT_MODE		0x00
#define AD9680_REG_VREF_MASK		0x0F

/*
 * Analog Devices AD9652
 */

#define CHIPID_AD9652			0xC1
#define AD9652_REG_VREF_MASK		0xC0

/*
 * Analog Devices AD9234
 */

#define CHIPID_AD9234			0xCE

/* debugfs direct register access */
#define DEBUGFS_DRA_PCORE_REG_MAGIC	0x80000000

#include <linux/spi/spi.h>

enum {
	ID_AD9467,
	ID_AD9643,
	ID_AD9250,
	ID_AD9265,
	ID_AD9680,
	ID_AD9683,
	ID_AD9625,
	ID_AD9434,
	ID_AD9652,
	ID_AD9234,
};

struct axiadc_chip_info {
	char				*name;
	unsigned			num_channels;
	unsigned 		num_shadow_slave_channels;
	const unsigned long 	*scan_masks;
	int			(*scale_table)[2];
	int				num_scales;
	int				max_testmode;
	unsigned long			max_rate;
	struct iio_chan_spec		channel[8];
};

struct axiadc_state {
	struct device 			*dev_spi;
	struct iio_info			iio_info;
	void __iomem			*regs;
	void __iomem			*slave_regs;
	unsigned				max_usr_channel;
	unsigned			adc_def_output_mode;
	unsigned			max_count;
	unsigned			id;
	unsigned			pcore_version;
	bool				has_fifo_interface;
	bool			dp_disable;
	unsigned char		testmode[2];
	unsigned long 		adc_clk;
	bool				streaming_dma;
	unsigned			have_slave_channels;

	struct iio_chan_spec	channels[16];
};

struct axiadc_converter {
	struct spi_device 	*spi;
	struct clk 		*clk;
	void 			*phy;
	struct gpio_desc		*pwrdown_gpio;
	struct gpio_desc		*reset_gpio;
	unsigned			id;
	unsigned			adc_output_mode;
	unsigned 		testmode[2];
	unsigned long 		adc_clk;
	const struct axiadc_chip_info	*chip_info;
	int		(*read)(struct spi_device *spi, unsigned reg);
	int		(*write)(struct spi_device *spi,
				 unsigned reg, unsigned val);
	int		(*setup)(struct spi_device *spi, unsigned mode);

	struct iio_chan_spec const	*channels;
	int				num_channels;
	const struct attribute_group	*attrs;
	struct iio_dev 	*indio_dev;
	int (*read_raw)(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			int *val,
			int *val2,
			long mask);

	int (*write_raw)(struct iio_dev *indio_dev,
			 struct iio_chan_spec const *chan,
			 int val,
			 int val2,
			 long mask);

	int (*read_event_value)(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			enum iio_event_type type,
			enum iio_event_direction dir,
			enum iio_event_info info,
			int *val,
			int *val2);

	int (*write_event_value)(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			enum iio_event_type type,
			enum iio_event_direction dir,
			enum iio_event_info info,
			int val,
			int val2);

	int (*read_event_config)(struct iio_dev *indio_dev,
			const struct iio_chan_spec *chan,
			enum iio_event_type type,
			enum iio_event_direction dir);

	int (*write_event_config)(struct iio_dev *indio_dev,
			const struct iio_chan_spec *chan,
			enum iio_event_type type,
			enum iio_event_direction dir,
			int state);

	int (*post_setup)(struct iio_dev *indio_dev);
	int (*testmode_set)(struct iio_dev *indio_dev, unsigned chan,
			unsigned mode);
};



static inline struct axiadc_converter *to_converter(struct device *dev)
{
	struct axiadc_converter *conv = spi_get_drvdata(to_spi_device(dev));

	if (conv)
		return conv;

	return ERR_PTR(-ENODEV);
};

struct axiadc_spidev {
	struct device_node *of_nspi;
	struct device *dev_spi;
};

/*
 * IO accessors
 */

static inline void axiadc_write(struct axiadc_state *st, unsigned reg, unsigned val)
{
	iowrite32(val, st->regs + reg);
}

static inline unsigned int axiadc_read(struct axiadc_state *st, unsigned reg)
{
	return ioread32(st->regs + reg);
}

static inline void axiadc_slave_write(struct axiadc_state *st, unsigned reg, unsigned val)
{
	iowrite32(val, st->slave_regs + reg);
}

static inline unsigned int axiadc_slave_read(struct axiadc_state *st, unsigned reg)
{
	return ioread32(st->slave_regs + reg);
}

int axiadc_set_pnsel(struct axiadc_state *st, int channel, enum adc_pn_sel sel);
enum adc_pn_sel axiadc_get_pnsel(struct axiadc_state *st,
			       int channel, const char **name);

int axiadc_configure_ring(struct iio_dev *indio_dev, const char *dma_name);
void axiadc_unconfigure_ring(struct iio_dev *indio_dev);
int axiadc_configure_ring_stream(struct iio_dev *indio_dev,
	const char *dma_name);
void axiadc_unconfigure_ring_stream(struct iio_dev *indio_dev);

#endif /* ADI_AXI_ADC_H_ */
