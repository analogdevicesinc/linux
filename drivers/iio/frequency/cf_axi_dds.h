/*
 * DDS PCORE/COREFPGA Module
 *
 * Copyright 2012-2014 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#ifndef ADI_AXI_DDS_H_
#define ADI_AXI_DDS_H_

#include <linux/spi/spi.h>
#include <linux/clk/clkscale.h>
#include <linux/fpga/adi-axi-common.h>

#define ADI_REG_CONFIG			0x000C
#define ADI_IQCORRECTION_DISABLE	(1 << 0)
#define ADI_DCFILTER_DISABLE		(1 << 1)
#define ADI_DATAFORMAT_DISABLE		(1 << 2)
#define ADI_USERPORTS_DISABLE		(1 << 3)
#define ADI_MODE_1R1T			(1 << 4)
#define ADI_DELAY_CONTROL_DISABLE	(1 << 5)
#define ADI_DDS_DISABLE			(1 << 6)
#define ADI_CMOS_OR_LVDS_N		(1 << 7)
#define ADI_PPS_RECEIVER_ENABLE		(1 << 8)
#define ADI_SCALECORRECTION_ONLY	(1 << 9)

/* DAC COMMON */

#define ADI_REG_RSTN		0x0040
#define ADI_RSTN			(1 << 0)
#define ADI_MMCM_RSTN 		(1 << 1)

#define ADI_REG_CNTRL_1		0x0044
#define ADI_ENABLE		(1 << 0) /* v7.0 */
#define ADI_SYNC			(1 << 0) /* v8.0 */

#define ADI_REG_CNTRL_2		0x0048
#define ADI_PAR_TYPE		(1 << 7)
#define ADI_PAR_ENB		(1 << 6)
#define ADI_R1_MODE		(1 << 5)
#define ADI_DATA_FORMAT		(1 << 4)
#define ADI_DATA_SEL(x)		(((x) & 0xF) << 0) /* v7.0 */
#define ADI_TO_DATA_SEL(x)	(((x) >> 0) & 0xF) /* v7.0 */

enum dds_data_select {
	DATA_SEL_DDS,
	DATA_SEL_SED,
	DATA_SEL_DMA,
	DATA_SEL_ZERO,	/* OUTPUT 0 */
	DATA_SEL_PN7,
	DATA_SEL_PN15,
	DATA_SEL_PN23,
	DATA_SEL_PN31,
	DATA_SEL_LB,	/* loopback data (ADC) */
	DATA_SEL_PNXX,	/* (Device specific) */
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
#define ADI_DRP_LOCKED		(1 << 17)
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

#define ADI_REG_DAC_GP_CONTROL	0x00BC

#define ADI_REG_DAC_DP_DISABLE	0x00C0
#define ADI_DAC_DP_DISABLE	(1 << 0)

/* JESD TPL */

#define ADI_REG_TPL_CNTRL		0x0200
#define ADI_REG_TPL_STATUS		0x0204
#define ADI_REG_TPL_DESCRIPTOR_1	0x0240
#define ADI_REG_TPL_DESCRIPTOR_2	0x0244

#define ADI_TO_JESD_M(x)		(((x) >> 0) & 0xFF)
#define ADI_TO_JESD_L(x)		(((x) >> 8) & 0xFF)
#define ADI_TO_JESD_S(x)		(((x) >> 16) & 0xFF)
#define ADI_TO_JESD_F(x)		(((x) >> 24) & 0xFF)

#define ADI_TO_JESD_N(x)		(((x) >> 0) & 0xFF)
#define ADI_TO_JESD_NP(x)		(((x) >> 8) & 0xFF)

/* DAC CHANNEL */

#define ADI_REG_CHAN_CNTRL_1_IIOCHAN(x)	(0x0400 + ((x) >> 1) * 0x40 + ((x) & 1) * 0x8)
#define ADI_DDS_SCALE(x)			(((x) & 0xFFFF) << 0)
#define ADI_TO_DDS_SCALE(x)		(((x) >> 0) & 0xFFFF)

#define ADI_REG_CHAN_CNTRL_2_IIOCHAN(x)	(0x0404 + ((x) >> 1) * 0x40 + ((x) & 1) * 0x8)
#define ADI_DDS_INIT(x)			(((x) & 0xFFFF) << 16)
#define ADI_TO_DDS_INIT(x)		(((x) >> 16) & 0xFFFF)
#define ADI_DDS_INCR(x)			(((x) & 0xFFFF) << 0)
#define ADI_TO_DDS_INCR(x)		(((x) >> 0) & 0xFFFF)

#define ADI_REG_CHAN_CNTRL_1(c)		(0x0400 + (c) * 0x40)
#define ADI_DDS_SCALE_1(x)		(((x) & 0xFFFF) << 0)
#define ADI_TO_DDS_SCALE_1(x)		(((x) >> 0) & 0xFFFF)

#define ADI_REG_CHAN_CNTRL_2(c)		(0x0404 + (c) * 0x40)
#define ADI_DDS_INIT_1(x)		(((x) & 0xFFFF) << 16)
#define ADI_TO_DDS_INIT_1(x)		(((x) >> 16) & 0xFFFF)
#define ADI_DDS_INCR_1(x)		(((x) & 0xFFFF) << 0)
#define ADI_TO_DDS_INCR_1(x)		(((x) >> 0) & 0xFFFF)

#define ADI_REG_CHAN_CNTRL_3(c)		(0x0408 + (c) * 0x40)
#define ADI_DDS_SCALE_2(x)		(((x) & 0xFFFF) << 0)
#define ADI_TO_DDS_SCALE_2(x)		(((x) >> 0) & 0xFFFF)

#define ADI_REG_CHAN_CNTRL_4(c)		(0x040C + (c) * 0x40)
#define ADI_DDS_INIT_2(x)		(((x) & 0xFFFF) << 16)
#define ADI_TO_DDS_INIT_2(x)		(((x) >> 16) & 0xFFFF)
#define ADI_DDS_INCR_2(x)		(((x) & 0xFFFF) << 0)
#define ADI_TO_DDS_INCR_2(x)		(((x) >> 0) & 0xFFFF)

#define ADI_REG_CHAN_CNTRL_5(c)		(0x0410 + (c) * 0x40)
#define ADI_DDS_PATT_2(x)		(((x) & 0xFFFF) << 16)
#define ADI_TO_DDS_PATT_2(x)		(((x) >> 16) & 0xFFFF)
#define ADI_DDS_PATT_1(x)		(((x) & 0xFFFF) << 0)
#define ADI_TO_DDS_PATT_1(x)		(((x) >> 0) & 0xFFFF)

#define ADI_REG_CHAN_CNTRL_6(c)		(0x0414 + (c) * 0x40)
#define ADI_IQCOR_ENB			(1 << 2) /* v8.0 */
//#define ADI_DAC_LB_ENB  			(1 << 1) /* v7.0 */
//#define ADI_DAC_PN_ENB 			(1 << 0) /* v7.0 */

#define ADI_REG_CHAN_CNTRL_7(c)		(0x0418 + (c) * 0x40) /* v8.0 */
#define ADI_DAC_DDS_SEL(x)		(((x) & 0xF) << 0)
#define ADI_TO_DAC_DDS_SEL(x)		(((x) >> 0) & 0xF)

#define ADI_REG_CHAN_CNTRL_8(c)		(0x041C + (c) * 0x40) /* v8.0 */
#define ADI_IQCOR_COEFF_1(x)		(((x) & 0xFFFF) << 16)
#define ADI_TO_IQCOR_COEFF_1(x)		(((x) >> 16) & 0xFFFF)
#define ADI_IQCOR_COEFF_2(x)		(((x) & 0xFFFF) << 0)
#define ADI_TO_IQCOR_COEFF_2(x)		(((x) >> 0) & 0xFFFF)

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


#define AXIDDS_MAX_DMA_SIZE		(6 * 1024 * 1024) /* Randomly picked */

/* debugfs direct register access */
#define DEBUGFS_DRA_PCORE_REG_MAGIC	0x80000000

#define AXIDDS_MAX_NUM_BUF_CHAN 64
#define AXIDDS_MAX_NUM_DDS_CHAN (2 * AXIDDS_MAX_NUM_BUF_CHAN)
#define AXIDDS_MAX_NUM_CHANNELS (AXIDDS_MAX_NUM_BUF_CHAN + \
				 AXIDDS_MAX_NUM_DDS_CHAN)

enum {
	ID_AD9122,
	ID_AD9739A,
	ID_AD9135,
	ID_AD9136,
	ID_AD9144,
	ID_AD9152,
	ID_AD9154,
	ID_AD9162,
	ID_AD9162_COMPLEX,
	ID_AD9172_M2,
	ID_AD9172_M4,
	ID_AD9172_M6,
};

enum fifo_ctrl {
	FIFO_UNSET,
	FIFO_DISABLE,
	FIFO_ENABLE,
};

struct cf_axi_dds_chip_info {
	const char *name;
	unsigned int num_channels;
	unsigned int num_dds_channels;
	unsigned int num_dp_disable_channels;
	unsigned int num_buf_channels;
	unsigned num_shadow_slave_channels;
	const unsigned long *scan_masks;
	struct iio_chan_spec channel[AXIDDS_MAX_NUM_CHANNELS];
};

struct cf_axi_dds_state;

enum {
	CLK_DATA,
	CLK_DAC,
	CLK_REF,
	CLK_NUM,
};

struct cf_axi_converter {
	struct spi_device 	*spi;
	struct clk 	*clk[CLK_NUM];
	struct clock_scale	clkscale[CLK_NUM];
	void		*phy;
	struct gpio_desc			*pwrdown_gpio;
	struct gpio_desc			*reset_gpio;
	struct gpio_desc			*txen_gpio[2];
	unsigned		id;
	unsigned		interp_factor;
	unsigned		fcenter_shift;
	unsigned long 	intp_modes[5];
	unsigned long 	cs_modes[17];
	int		temp_calib;
	unsigned		temp_calib_code;
	int		temp_slope;
	int		(*read)(struct spi_device *spi, unsigned reg);
	int		(*write)(struct spi_device *spi,
				 unsigned reg, unsigned val);
	int		(*setup)(struct cf_axi_converter *conv);
	int		(*get_fifo_status)(struct cf_axi_converter *conv);
	unsigned long long	(*get_data_clk)(struct cf_axi_converter *conv);

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
int cf_axi_dds_datasel(struct cf_axi_dds_state *st,
			       int channel, enum dds_data_select sel);
void cf_axi_dds_start_sync(struct cf_axi_dds_state *st);
int cf_axi_dds_pl_ddr_fifo_ctrl(struct cf_axi_dds_state *st, bool enable);

/*
 * IO accessors
 */

void dds_write(struct cf_axi_dds_state *st,
	       unsigned int reg, unsigned int val);
int dds_read(struct cf_axi_dds_state *st, unsigned int reg);
void dds_slave_write(struct cf_axi_dds_state *st,
		     unsigned int reg, unsigned int val);
unsigned int dds_slave_read(struct cf_axi_dds_state *st, unsigned int reg);
void dds_master_write(struct cf_axi_dds_state *st,
		      unsigned int reg, unsigned int val);

bool cf_axi_dds_dma_fifo_en(struct cf_axi_dds_state *st);

#endif /* ADI_AXI_DDS_H_ */
