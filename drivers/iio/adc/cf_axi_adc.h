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

/* PCORE CoreFPGA register map */

#define AXIADC_PCORE_VERSION		0x00
#define AXIADC_PCORE_DMA_CTRL		0x0C
#define AXIADC_PCORE_DMA_STAT		0x10
#define AXIADC_PCORE_ADC_STAT		0x14
#define AXIADC_PCORE_PN_ERR_CTRL	0x24
#define AXIADC_PCORE_ADC_CTRL		0x2C
#define AXIADC_PCORE_IDENT		0x30
#define AXIADC_PCORE_CA_OFFS_SCALE	0x40
#define AXIADC_PCORE_CB_OFFS_SCALE	0x44

/* AXIADC_PCORE_DMA_CTRL */
#define AXIADC_DMA_CAP_EN		(1 << 16)
#define AXIADC_DMA_CNT(x)		(((x) & 0xFFFF) << 0)

/* AXIADC_PCORE_DMA_STAT */
#define AXIADC_DMA_STAT_BUSY		(1 << 0)
#define AXIADC_DMA_STAT_OVF		(1 << 1)
#define AXIADC_DMA_STAT_UNF		(1 << 2)

/* AXIADC_PCORE_ADC_STAT */
#define AXIADC_PCORE_ADC_STAT_OVR0	(1 << 0) /* W1C */
#define AXIADC_PCORE_ADC_STAT_OVR1	(1 << 1) /* W1C */
#define AXIADC_PCORE_ADC_STAT_PN_OOS0	(1 << 2) /* W1C */
#define AXIADC_PCORE_ADC_STAT_PN_OOS1	(1 << 3) /* W1C */
#define AXIADC_PCORE_ADC_STAT_PN_ERR0	(1 << 4) /* W1C */
#define AXIADC_PCORE_ADC_STAT_PN_ERR1	(1 << 5) /* W1C */
#define AXIADC_PCORE_ADC_STAT_MASK	0x3F

/* AXIADC_PCORE_PN_ERR_CTRL */
#define AXIADC_PN23_1_EN		(1 << 1)
#define AXIADC_PN23_0_EN		(1 << 0)
#define AXIADC_PN9_1_EN			(0 << 1)
#define AXIADC_PN9_0_EN			(0 << 0)
#define AXIADC_PN23_EN			(AXIADC_PN23_0_EN | AXIADC_PN23_1_EN)
#define AXIADC_PN9_EN			(AXIADC_PN9_0_EN | AXIADC_PN9_1_EN)

/* AXIADC_PCORE_ADC_CTRL */
#define AXIADC_INPUT_FMT_OFFSET_BIN	(1 << 3)
#define AXIADC_INPUT_FMT_TWOS_COMPL	(0 << 3)
#define AXIADC_SCALE_OFFSET_EN		(1 << 2)
#define AXIADC_SIGNEXTEND		(1 << 1)
#define AXIADC_STATUS_EN		(1 << 0)

/* AXIADC_PCORE_IDENT */
#define AXIADC_PCORE_IDENT_SLAVE	0x1

/* AXIADC_PCORE_C[A|B]_OFFS_SCALE */
#define AXIADC_OFFSET(x)		(((x) & 0xFFFF) << 16)
#define AXIADC_SCALE(x)			((x) & 0xFFFF)

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
#define TESTMODE_PN9_SEQ		0x6
#define TESTMODE_ONE_ZERO_TOGGLE	0x7

/* ADC_REG_OUTPUT_MODE */
#define OUTPUT_MODE_OFFSET_BINARY	0x0
#define OUTPUT_MODE_TWOS_COMPLEMENT	0x1
#define OUTPUT_MODE_GRAY_CODE		0x2

/* ADC_REG_OUTPUT_PHASE */
#define OUTPUT_EVEN_ODD_MODE_EN		0x20

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

#include <linux/spi/spi.h>

enum {
	ID_AD9467,
	ID_AD9643,
};

struct axiadc_chip_info {
	char				name[8];
	unsigned			num_channels;
	unsigned long			available_scan_masks[2];
	const int			(*scale_table)[2];
	int				num_scales;
	struct iio_chan_spec		channel[4];
};

struct axiadc_state {
	struct device 			*dev_spi;
	struct mutex			lock;
	struct completion		dma_complete;
	struct dma_chan			*rx_chan;
	const struct axiadc_chip_info	*chip_info;
	void __iomem			*regs;
	void				*buf_virt;
	dma_addr_t			buf_phys;
	int				compl_stat;
	unsigned			adc_def_output_mode;
	unsigned			ring_lenght;
	unsigned			rcount;
	unsigned			fftcount;
//	unsigned			bytes_per_datum;
	unsigned			id;
	unsigned char			testmode[2];
};

struct axiadc_converter {
	struct spi_device 		*spi;
	unsigned			id;
	int		(*read)(struct spi_device *spi, unsigned reg);
	int		(*write)(struct spi_device *spi,
				 unsigned reg, unsigned val);
	int		(*setup)(struct spi_device *spi, unsigned mode);
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

int axiadc_configure_ring(struct iio_dev *indio_dev);
void axiadc_unconfigure_ring(struct iio_dev *indio_dev);

#endif /* ADI_AXI_ADC_H_ */
