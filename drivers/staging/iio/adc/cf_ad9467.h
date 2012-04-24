/*
 * ADI-AIM ADI ADC Interface Module
 *
 * Copyright 2012 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 *
 * http://wiki.analog.com/resources/fpga/xilinx/fmc/ad9467
 */

#ifndef ADI_AIM_H_
#define ADI_AIM_H_

/* PCORE CoreFPGA register map */

#define AD9467_PCORE_VERSION		0x00
#define AD9467_PCORE_SPI_CTRL		0x04
#define AD9467_PCORE_SPI_RDSTAT		0x08
#define AD9467_PCORE_DMA_CTRL		0x0C
#define AD9467_PCORE_DMA_STAT		0x10
#define AD9467_PCORE_ADC_STAT		0x14
#define AD9467_PCORE_PN_ERR_CTRL	0x24
#define AD9467_PCORE_IDENT		0x28


/* AD9467_PCORE_SPI_CTRL */
#define AD9647_SPI_START		(1 << 25)
#define AD9647_SPI_SEL(x)		(((x) & 0x1) << 24)
#define AD9647_SPI_READ			(1 << 23)
#define AD9647_SPI_WRITE		(0 << 23)
#define AD9647_SPI_ADDR(x)		(((x) & 0x1FFF) << 8)
#define AD9647_SPI_DATA(x)		(((x) & 0xFF) << 0)

/* AD9467_PCORE_SPI_RDSTAT */
#define AD9647_SPI_IDLE			(1 << 8)
#define AD9647_SPI_READVAL(x)		((x) & 0xFF)

/* AD9467_PCORE_DMA_CTRL */
#define AD9647_DMA_CAP_EN		(1 << 16)
#define AD9647_DMA_CNT(x)		(((x) & 0xFFFF) << 0)

/* AD9467_PCORE_DMA_STAT */
#define AD9647_DMA_STAT_BUSY		(1 << 0)
#define AD9647_DMA_STAT_OVF		(1 << 1)
#define AD9647_DMA_STAT_UNF		(1 << 2)

/* AD9467_PCORE_ADC_STAT */
#define AD9467_PCORE_ADC_STAT_OVR	(1 << 0)
#define AD9467_PCORE_ADC_STAT_PN_OOS	(1 << 1) /* W1C */
#define AD9467_PCORE_ADC_STAT_PN_ERR	(1 << 2) /* W1C */

/* AD9467_PCORE_PN_ERR_CTRL */
#define AD9467_PN23_EN			(1 << 0)
#define AD9467_PN9_EN			(0 << 0)

/* AD9467_PCORE_IDENT */
#define AD9467_PCORE_IDENT_SLAVE	0x1

/*
 * ADI High-Speed ADC common spi interface registers
 * See Application-Note AN-877
 */

#define ADC_REG_CHIP_PORT_CONF		0x00
#define ADC_REG_CHIP_ID			0x01
#define ADC_REG_CHIP_GRADE		0x02
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

/*
 * Analog Devices AD9467 16-Bit, 200/250 MSPS ADC
 */

#define AD9467_DEF_OUTPUT_MODE		0x08
#define AD9467_REG_VREF_MASK		0x0F
#define CHIPID_AD9467			0x50

/*
 * Analog Devices AD9643 Dual 14-Bit, 170/210/250 MSPS ADC
 */

#define CHIPID_AD9643			0x82
#define AD9643_REG_VREF_MASK		0x1F
#define AD9643_DEF_OUTPUT_MODE		0x04

enum {
	ID_AD9467,
	ID_AD9643,
};

struct aim_chip_info {
	char				name[8];
	unsigned			num_channels;
	unsigned long			available_scan_masks[2];
	const int			(*scale_table)[2];
	int				num_scales;
	struct iio_chan_spec		channel[2];
};

struct aim_state {
	struct spi_device		*spi;
	struct mutex			lock;
	struct completion		dma_complete;
	struct dma_chan			*rx_chan;
	const struct aim_chip_info	*chip_info;
	void __iomem			*regs;
	void				*buf_virt;
	dma_addr_t			buf_phys;
	int				compl_stat;
	unsigned			spi_ssel;
	unsigned			ring_lenght;
	unsigned			rcount;
	unsigned			fftcount;
	unsigned			bytes_per_datum;
	unsigned			id;

	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */

	unsigned char			data[3] ____cacheline_aligned;
};

/*
 * IO accessors
 */

static inline void aim_write(struct aim_state *st, unsigned reg, unsigned val)
{
	iowrite32(val, st->regs + reg);
}

static inline unsigned int aim_read(struct aim_state *st, unsigned reg)
{
	return ioread32(st->regs + reg);
}

int aim_configure_ring(struct iio_dev *indio_dev);
void aim_unconfigure_ring(struct iio_dev *indio_dev);

#endif /* ADI_AIM_H_ */
