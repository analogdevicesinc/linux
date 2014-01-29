/*
 * DDS PCORE/COREFPGA Module
 *
 * Copyright 2012 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#ifndef ADI_AXI_DDS_H_
#define ADI_AXI_DDS_H_

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

/* DAC COMMON */

#define ADI_REG_RSTN		0x0040
#define ADI_RSTN			(1 << 0)
#define ADI_MMCM_RSTN 		(1 << 1)

#define ADI_REG_CNTRL_1		0x0044
#define ADI_ENABLE		(1 << 0)

#define ADI_REG_CNTRL_2		0x0048
#define ADI_PAR_TYPE		(1 << 7)
#define ADI_PAR_ENB		(1 << 6)
#define ADI_R1_MODE		(1 << 5)
#define ADI_DATA_FORMAT		(1 << 4)
#define ADI_DATA_SEL(x)		(((x) & 0xF) << 0)
#define ADI_TO_DATA_SEL(x)	(((x) >> 0) & 0xF)

enum {
	DATA_SEL_DDS,
	DATA_SEL_SED,
	DATA_SEL_DMA,
};


#define ADI_REG_RATECNTRL	0x004C
#define ADI_RATE(x)		(((x) & 0xFF) << 0)
#define ADI_TO_RATE(x)		(((x) >> 0) & 0xFF)

#define ADI_REG_FRAME		0x0050
#define ADI_FRAME		(1 << 0)

#define ADI_REG_CLK_FREQ		0x0054
#define ADI_CLK_FREQ(x)		(((x) & 0xFFFFFFFF) << 0)
#define ADI_TO_CLK_FREQ(x)	(((x) >> 0) & 0xFFFFFFFF)

#define ADI_REG_CLK_RATIO		0x0058
#define ADI_CLK_RATIO(x)		(((x) & 0xFFFFFFFF) << 0)
#define ADI_TO_CLK_RATIO(x)	(((x) >> 0) & 0xFFFFFFFF)

#define ADI_REG_STATUS		0x005C
#define ADI_STATUS		(1 << 0)

#define ADI_REG_DRP_CNTRL	0x0070
#define ADI_DRP_SEL		(1 << 29)
#define ADI_DRP_RWN		(1 << 28)
#define ADI_DRP_ADDRESS(x)	(((x) & 0xFFF) << 16)
#define ADI_TO_DRP_ADDRESS(x)	(((x) >> 16) & 0xFFF)
#define ADI_DRP_WDATA(x)		(((x) & 0xFFFF) << 0)
#define ADI_TO_DRP_WDATA(x)	(((x) >> 0) & 0xFFFF)

#define ADI_REG_DRP_STATUS	0x0074
#define ADI_DRP_STATUS		(1 << 16)
#define ADI_DRP_RDATA(x)		(((x) & 0xFFFF) << 0)
#define ADI_TO_DRP_RDATA(x)	(((x) >> 0) & 0xFFFF)

#define ADI_REG_VDMA_FRMCNT	0x0084
#define ADI_VDMA_FRMCNT(x)	(((x) & 0xFFFFFFFF) << 0)
#define ADI_TO_VDMA_FRMCNT(x)	(((x) >> 0) & 0xFFFFFFFF)

#define ADI_REG_VDMA_STATUS	0x0088
#define ADI_VDMA_OVF		(1 << 1)
#define ADI_VDMA_UNF		(1 << 0)

#define ADI_REG_USR_CNTRL_1	0x00A0
#define ADI_USR_CHANMAX(x)	(((x) & 0xFF) << 0)
#define ADI_TO_USR_CHANMAX(x)	(((x) >> 0) & 0xFF)

#define ADI_REG_DAC_DP_DISABLE	0x00C0

/* DAC CHANNEL */

#define ADI_REG_CHAN_CNTRL_1_IIOCHAN(x)	(0x0400 + ((x) >> 1) * 0x40 + ((x) & 1) * 0x8)
#define ADI_DDS_SCALE(x)			(((x) & 0xF) << 0)
#define ADI_TO_DDS_SCALE(x)		(((x) >> 0) & 0xF)

#define ADI_REG_CHAN_CNTRL_2_IIOCHAN(x)	(0x0404 + ((x) >> 1) * 0x40 + ((x) & 1) * 0x8)
#define ADI_DDS_INIT(x)			(((x) & 0xFFFF) << 16)
#define ADI_TO_DDS_INIT(x)		(((x) >> 16) & 0xFFFF)
#define ADI_DDS_INCR(x)			(((x) & 0xFFFF) << 0)
#define ADI_TO_DDS_INCR(x)		(((x) >> 0) & 0xFFFF)

#define ADI_REG_CHAN_CNTRL_1(c)		(0x0400 + (c) * 0x40)
#define ADI_DDS_SCALE_1(x)		(((x) & 0xF) << 0)
#define ADI_TO_DDS_SCALE_1(x)		(((x) >> 0) & 0xF)

#define ADI_REG_CHAN_CNTRL_2(c)		(0x0404 + (c) * 0x40)
#define ADI_DDS_INIT_1(x)		(((x) & 0x1FFFF) << 15)
#define ADI_TO_DDS_INIT_1(x)		(((x) >> 15) & 0x1FFFF)
#define ADI_DDS_INCR_1(x)		(((x) & 0xFFFF) << 0)
#define ADI_TO_DDS_INCR_1(x)		(((x) >> 0) & 0xFFFF)

#define ADI_REG_CHAN_CNTRL_3(c)		(0x0408 + (c) * 0x40)
#define ADI_DDS_SCALE_2(x)		(((x) & 0xF) << 0)
#define ADI_TO_DDS_SCALE_2(x)		(((x) >> 0) & 0xF)

#define ADI_REG_CHAN_CNTRL_4(c)		(0x040C + (c) * 0x40)
#define ADI_DDS_INIT_2(x)		(((x) & 0x1FFFF) << 15)
#define ADI_TO_DDS_INIT_2(x)		(((x) >> 15) & 0x1FFFF)
#define ADI_DDS_INCR_2(x)		(((x) & 0xFFFF) << 0)
#define ADI_TO_DDS_INCR_2(x)		(((x) >> 0) & 0xFFFF)

#define ADI_REG_CHAN_CNTRL_5(c)		(0x0410 + (c) * 0x40)
#define ADI_DDS_PATT_2(x)		(((x) & 0xFFFF) << 16)
#define ADI_TO_DDS_PATT_2(x)		(((x) >> 16) & 0xFFFF)
#define ADI_DDS_PATT_1(x)		(((x) & 0xFFFF) << 0)
#define ADI_TO_DDS_PATT_1(x)		(((x) >> 0) & 0xFFFF)

#define ADI_REG_CHAN_CNTRL_6(c)		(0x0414 + (c) * 0x40)
#define ADI_DAC_LB_ENB  			(1 << 1)
#define ADI_DAC_PN_ENB 			(1 << 0)

#define ADI_REG_USR_CNTRL_3(c)		(0x0420 + (c) * 0x40)
#define ADI_USR_DATATYPE_BE		(1 << 25)
#define ADI_USR_DATATYPE_SIGNED		(1 << 24)
#define ADI_USR_DATATYPE_SHIFT(x)	(((x) & 0xFF) << 16)
#define ADI_TO_USR_DATATYPE_SHIFT(x)	(((x) >> 16) & 0xFF)
#define ADI_USR_DATATYPE_TOTAL_BITS(x)	(((x) & 0xFF) << 8)
#define ADI_TO_USR_DATATYPE_TOTAL_BITS(x) (((x) >> 8) & 0xFF)
#define ADI_USR_DATATYPE_BITS(x)		(((x) & 0xFF) << 0)
#define ADI_TO_USR_DATATYPE_BITS(x)	(((x) >> 0) & 0xFF)

#define ADI_REG_USR_CNTRL_4(c)		(0x0424 + (c) * 0x40)
#define ADI_USR_INTERPOLATION_M(x)	(((x) & 0x1FFFF) << 15)
#define ADI_TO_USR_INTERPOLATION_M(x)	(((x) >> 15) & 0x1FFFF)
#define ADI_USR_INTERPOLATION_N(x)	(((x) & 0xFFFF) << 0)
#define ADI_TO_USR_INTERPOLATION_N(x)	(((x) >> 0) & 0xFFFF)


#define AXIDDS_MAX_DMA_SIZE		(4 * 1024 * 1024) /* Randomly picked */

/* debugfs direct register access */
#define DEBUGFS_DRA_PCORE_REG_MAGIC	0x80000000

enum {
	ID_AD9122,
	ID_AD9739A,
};

struct cf_axi_dds_chip_info {
	char 				name[8];
	struct iio_chan_spec		channel[9];
	struct iio_chan_spec		buf_channel[4];
	unsigned			num_channels;
	unsigned			num_buf_channels;
	unsigned			num_dp_disable_channels;
};

#include <linux/amba/xilinx_dma.h>

struct cf_axi_dds_state {
	struct list_head		list;
	struct device 		*dev_spi;
	struct clk 		*clk;
	struct iio_dev 		*indio_dev;
	struct resource 		r_mem; /* IO mem resources */
	const struct cf_axi_dds_chip_info	*chip_info;
	bool			has_fifo_interface;
	bool			dp_disable;
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
	unsigned			buffer_length;
	unsigned			txcount;
	unsigned 		ddr_dds_interp_en;
	unsigned			cached_freq[8];
	bool			enable;
	struct notifier_block   clk_nb;
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
	int		temp_calib;
	unsigned		temp_calib_code;
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
						 unsigned chan, unsigned pat1,
						 unsigned pat2);
	int		(*pcore_sync)(struct iio_dev *indio_dev);
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
//void cf_axi_dds_stop(struct cf_axi_dds_state *st);

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
