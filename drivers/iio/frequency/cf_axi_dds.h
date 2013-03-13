/*
 * DDS PCORE/COREFPGA Module
 *
 * Copyright 2012 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#ifndef ADI_AXI_DDS_H_
#define ADI_AXI_DDS_H_

#define CF_AXI_DDS_VERSION_ID		0x00
#define CF_AXI_DDS_CTRL			0x04
#define CF_AXI_DDS_1A_OUTPUT_CTRL	0x08
#define CF_AXI_DDS_1B_OUTPUT_CTRL	0x0C
#define CF_AXI_DDS_2A_OUTPUT_CTRL	0x10
#define CF_AXI_DDS_2B_OUTPUT_CTRL	0x14
#define CF_AXI_DDS_INTERPOL_CTRL		0x18
#define CF_AXI_DDS_SCALE			0x20
#define CF_AXI_DDS_FRAME			0x24
#define CF_AXI_DDS_DMA_STAT		0x28
#define CF_AXI_DDS_DMA_FRAMECNT		0x2C
#define CF_AXI_DDS_PAT_DATA1		0x40
#define CF_AXI_DDS_PAT_DATA2		0x44
#define CF_AXI_DDS_PCORE_IDENT		0x48

/* CF_AXI_DDS_CTRL */
#define CF_AXI_DDS_CTRL_TWOSCMPL_EN	(1 << 5)
#define CF_AXI_DDS_CTRL_OFFSETBIN_EN	(0 << 5)
#define CF_AXI_DDS_CTRL_PATTERN_EN	(1 << 4)
#define CF_AXI_DDS_CTRL_INTERPOLATE	(1 << 3)
#define CF_AXI_DDS_CTRL_DDS_SEL		(1 << 2)
#define CF_AXI_DDS_CTRL_DATA_EN		(1 << 1)
#define CF_AXI_DDS_CTRL_DDS_CLK_EN_V1	(1 << 8)
#define CF_AXI_DDS_CTRL_DDS_CLK_EN_V2	(1 << 0)

/* CF_AXI_DDS_FRAME */
#define CF_AXI_DDS_FRAME_SYNC		0x1

/* CF_AXI_DDS_PCORE_IDENT */
#define CF_AXI_DDS_PCORE_IDENT_SLAVE	0x1

/* CF_AXI_DDS_DMA_STAT W1C */
#define CF_AXI_DDS_DMA_STAT_OVF		(1 << 0)
#define CF_AXI_DDS_DMA_STAT_UNF		(1 << 1)

#define AXIDDS_MAX_DMA_SIZE		(4 * 1024 * 1024) /* Randomly picked */

/* debugfs direct register access */
#define DEBUGFS_DRA_PCORE_REG_MAGIC	0x80000000

enum {
	ID_AD9122,
	ID_AD9739A,
};

struct cf_axi_dds_chip_info {
	char 				name[8];
	struct iio_chan_spec		channel[5];
	struct iio_chan_spec		buf_channel[2];
};

#include <linux/amba/xilinx_dma.h>

struct cf_axi_dds_state {
	struct list_head		list;
	struct device 		*dev_spi;
	struct resource 		r_mem; /* IO mem resources */
	const struct cf_axi_dds_chip_info	*chip_info;
	struct iio_info		iio_info;
	void			*buf_virt;
	dma_addr_t		buf_phys;
	struct dma_chan		*tx_chan;
	struct xilinx_dma_config	dma_config;
	u16			int_vref_mv;
	int 			irq;
	void __iomem		*regs;
	unsigned int		flags;
	u32			dac_clk;
	unsigned			vers_id;
	unsigned			buffer_length;
	unsigned			txcount;
	unsigned 		ddr_dds_interp_en;
};

enum {
	CLK_DATA,
	CLK_DAC,
	CLK_REF,
	CLK_NUM,
};

struct cf_axi_converter {
	struct spi_device 	*spi;
	struct clk 	*clk[CLK_NUM];
	unsigned		id;
	unsigned		interp_factor;
	unsigned		fcenter_shift;
	unsigned long 	intp_modes[5];
	unsigned long 	cs_modes[17];
	int		(*read)(struct spi_device *spi, unsigned reg);
	int		(*write)(struct spi_device *spi,
				 unsigned reg, unsigned val);
	int		(*setup)(struct cf_axi_converter *conv);
	int		(*get_fifo_status)(struct cf_axi_converter *conv);
	unsigned long	(*get_data_clk)(struct cf_axi_converter *conv);

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
	const struct attribute_group	*attrs;
	struct iio_dev 	*indio_dev;
	void		(*pcore_set_sed_pattern)(struct iio_dev *indio_dev,
						 unsigned pat1, unsigned pat2);
	void		(*pcore_sync)(struct iio_dev *indio_dev);
};

static inline struct cf_axi_converter *to_converter(struct device *dev)
{
	struct cf_axi_converter *conv = spi_get_drvdata(to_spi_device(dev));

	if (conv)
		return conv;

	return ERR_PTR(-ENODEV);
};

int cf_axi_dds_configure_buffer(struct iio_dev *indio_dev);
void cf_axi_dds_unconfigure_buffer(struct iio_dev *indio_dev);
void cf_axi_dds_stop(struct cf_axi_dds_state *st);

/*
 * IO accessors
 */

static inline void dds_write(struct cf_axi_dds_state *st,
			     unsigned reg, unsigned val)
{
	iowrite32(val, st->regs + reg);
}

static inline unsigned int dds_read(struct cf_axi_dds_state *st, unsigned reg)
{
	return ioread32(st->regs + reg);
}
#endif /* ADI_AXI_DDS_H_ */
