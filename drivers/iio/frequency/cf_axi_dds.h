/*
 * DDS PCORE/COREFPGA Module
 *
 * Copyright 2012-2014 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#ifndef ADI_AXI_DDS_H_
#define ADI_AXI_DDS_H_

#include <linux/bitfield.h>
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
#define ADI_XBAR_ENABLE			(1 << 10)
#define ADI_EXT_SYNC			(1 << 12)

/* DAC COMMON */

#define ADI_REG_RSTN		0x0040
#define ADI_RSTN		(1 << 0)
#define ADI_MMCM_RSTN		(1 << 1)

#define ADI_REG_CNTRL_1		0x0044
#define ADI_SYNC		(1 << 0) /* v8.0 */
#define ADI_EXT_SYNC_ARM	(1 << 1)
#define ADI_EXT_SYNC_DISARM	(1 << 2)
#define ADI_MANUAL_SYNC_REQUEST	(1 << 8)

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
	DATA_SEL_INV_PN7,
	DATA_SEL_INV_PN15,
	DATA_SEL_PN7,
	DATA_SEL_PN15,
	DATA_SEL_LB,	/* loopback data (ADC) */
	DATA_SEL_PNXX,	/* (Device specific) */
	DATA_SEL_RAMP_NIBBLE,
	DATA_SEL_RAMP_16,
};


#define ADI_REG_RATECNTRL	0x004C
#define ADI_RATE(x)		(((x) & 0xFF) << 0)
#define ADI_TO_RATE(x)		(((x) >> 0) & 0xFF)

#define ADI_REG_FRAME		0x0050
#define ADI_FRAME		(1 << 0)

#define ADI_REG_CLK_FREQ	0x0054
#define ADI_CLK_FREQ(x)		(((x) & 0xFFFFFFFF) << 0)
#define ADI_TO_CLK_FREQ(x)	(((x) >> 0) & 0xFFFFFFFF)

#define ADI_REG_CLK_RATIO	0x0058
#define ADI_CLK_RATIO(x)	(((x) & 0xFFFFFFFF) << 0)
#define ADI_TO_CLK_RATIO(x)	(((x) >> 0) & 0xFFFFFFFF)

#define ADI_REG_STATUS		0x005C
#define ADI_STATUS		(1 << 0)

#define ADI_REG_SYNC_STATUS	0x0068
#define ADI_ADC_SYNC_STATUS	(1 << 0)

#define ADI_REG_DRP_CNTRL	0x0070
#define ADI_DRP_SEL		(1 << 29)
#define ADI_DRP_RWN		(1 << 28)
#define ADI_DRP_ADDRESS(x)	(((x) & 0xFFF) << 16)
#define ADI_TO_DRP_ADDRESS(x)	(((x) >> 16) & 0xFFF)
#define ADI_DRP_WDATA(x)	(((x) & 0xFFFF) << 0)
#define ADI_TO_DRP_WDATA(x)	(((x) >> 0) & 0xFFFF)

#define ADI_REG_DRP_STATUS	0x0074
#define ADI_DRP_LOCKED		(1 << 17)
#define ADI_DRP_STATUS		(1 << 16)
#define ADI_DRP_RDATA(x)	(((x) & 0xFFFF) << 0)
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

/* DAC CHANNEL */

#define ADI_REG_CHAN_CNTRL_1_IIOCHAN(x)	(0x0400 + ((x) >> 1) * 0x40 + ((x) & 1) * 0x8)
#define ADI_DDS_SCALE(x)		(((x) & 0xFFFF) << 0)
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
//#define ADI_DAC_LB_ENB			(1 << 1) /* v7.0 */
//#define ADI_DAC_PN_ENB 			(1 << 0) /* v7.0 */

#define ADI_REG_CHAN_CNTRL_7(c)		(0x0418 + (c) * 0x40) /* v8.0 */
#define ADI_DAC_DDS_SEL(x)		(((x) & 0xF) << 0)
#define ADI_TO_DAC_DDS_SEL(x)		(((x) >> 0) & 0xF)
#define ADI_DAC_SRC_CH_SEL(x)		(((x) & 0xFF) << 8)
#define ADI_TO_DAC_SRC_CH_SEL(x)	(((x) >> 8) & 0xFF)
#define ADI_DAC_ENABLE_MASK		(1 << 16)

#define ADI_REG_CHAN_CNTRL_8(c)		(0x041C + (c) * 0x40) /* v8.0 */
#define ADI_IQCOR_COEFF_1(x)		(((x) & 0xFFFF) << 16)
#define ADI_TO_IQCOR_COEFF_1(x)		(((x) >> 16) & 0xFFFF)
#define ADI_IQCOR_COEFF_2(x)		(((x) & 0xFFFF) << 0)
#define ADI_TO_IQCOR_COEFF_2(x)		(((x) >> 0) & 0xFFFF)

#define ADI_REG_USR_CNTRL_3(c)			(0x0420 + (c) * 0x40)
#define ADI_USR_DATATYPE_BE			(1 << 25)
#define ADI_USR_DATATYPE_SIGNED			(1 << 24)
#define ADI_USR_DATATYPE_SHIFT(x)		(((x) & 0xFF) << 16)
#define ADI_TO_USR_DATATYPE_SHIFT(x)		(((x) >> 16) & 0xFF)
#define ADI_USR_DATATYPE_TOTAL_BITS(x)		(((x) & 0xFF) << 8)
#define ADI_TO_USR_DATATYPE_TOTAL_BITS(x)	(((x) >> 8) & 0xFF)
#define ADI_USR_DATATYPE_BITS(x)		(((x) & 0xFF) << 0)
#define ADI_TO_USR_DATATYPE_BITS(x)		(((x) >> 0) & 0xFF)

#define ADI_REG_USR_CNTRL_4(c)		(0x0424 + (c) * 0x40)
#define ADI_USR_INTERPOLATION_M(x)	(((x) & 0x1FFFF) << 15)
#define ADI_TO_USR_INTERPOLATION_M(x)	(((x) >> 15) & 0x1FFFF)
#define ADI_USR_INTERPOLATION_N(x)	(((x) & 0xFFFF) << 0)
#define ADI_TO_USR_INTERPOLATION_N(x)	(((x) >> 0) & 0xFFFF)


#define AXI_REG_CNTRL_1             0x44
#define AXI_REG_CNTRL_2				0x48
#define   AXI_MSK_USIGN_DATA			BIT(4)
#define   AXI_MSK_SYMB_8B			BIT(14)
#define   AXI_MSK_SDR_DDR_N			BIT(16)

#define AXI_REG_CNTRL_DATA_RD			0x80
#define   AXI_MSK_DATA_RD_8			GENMASK(7, 0)
#define   AXI_MSK_DATA_RD_16			GENMASK(15, 0)
#define   AXI_MSK_ADDRESS			GENMASK(31, 24)

#define AXI_REG_CNTRL_DATA_WR			0x84
#define   AXI_MSK_DATA_WR_8			GENMASK(23, 16)
#define   AXI_MSK_DATA_WR_16			GENMASK(23, 8)

#define AXI_REG_UI_STATUS			0x88
#define   AXI_MSK_BUSY				BIT(4)

#define AXI_REG_CHAN_CNTRL_7_CH0		0x418
#define AXI_REG_CHAN_CNTRL_7_CH1		0x458
#define   AXI_EXT_SYNC_ARM             0x02

#define AXI_REG_CNTRL_CSTM			0x8C
#define   AXI_MSK_TRANSFER_DATA			BIT(0)
#define   AXI_MSK_STREAM			BIT(1)
#define   AXI_MSK_SYNCED_TRANSFER		BIT(2)

#define   AXI_SEL_SRC_DMA			0x02
#define   AXI_SEL_SRC_ADC			0x08
#define   AXI_SEL_SRC_DDS			0x0b

#define AD3552R_REG_OUTPUT_RANGE		0x19
#define   AD3552R_MASK_OUT_RANGE        GENMASK(7, 0)
#define   AD3552R_MASK_CH0_RANGE		GENMASK(2, 0)
#define   AD3552R_MASK_CH1_RANGE		GENMASK(6, 4)

#define AD3552R_TFER_8BIT_SDR			(AXI_MSK_SYMB_8B | \
						AXI_MSK_SDR_DDR_N)

#define AD3552R_STREAM_SATRT			(AXI_MSK_TRANSFER_DATA | \
						AXI_MSK_STREAM)

#define CNTRL_CSTM_ADDR(x)			FIELD_PREP(AXI_MSK_ADDRESS, x)
#define CNTRL_DATA_WR_8(x)			FIELD_PREP(AXI_MSK_DATA_WR_8, x)
#define CNTRL_DATA_WR_16(x)			FIELD_PREP(AXI_MSK_DATA_WR_16, x)

#define RD_ADDR(x)				(BIT(7) | (x))

#define SET_CH0_RANGE(x)			FIELD_PREP(AD3552R_MASK_CH0_RANGE, x)
#define SET_CH1_RANGE(x)			FIELD_PREP(AD3552R_MASK_CH1_RANGE, x)
#define GET_CH0_RANGE(x)			FIELD_GET(AD3552R_MASK_CH0_RANGE, x)
#define GET_CH1_RANGE(x)			FIELD_GET(AD3552R_MASK_CH1_RANGE, x)


#define AXIDDS_MAX_DMA_SIZE		(6 * 1024 * 1024) /* Randomly picked */

/* debugfs direct register access */
#define DEBUGFS_DRA_PCORE_REG_MAGIC	0x80000000

#define AXIDDS_MAX_NUM_BUF_CHAN 64
#define AXIDDS_MAX_NUM_DDS_CHAN (2 * AXIDDS_MAX_NUM_BUF_CHAN)
#define AXIDDS_MAX_NUM_CHANNELS (AXIDDS_MAX_NUM_BUF_CHAN + \
				 AXIDDS_MAX_NUM_DDS_CHAN)

enum {
	ID_AD3552R,
	ID_AD9122,
	ID_AD9739A,
	ID_AD9783,
	ID_AD9135,
	ID_AD9136,
	ID_AD9144,
	ID_AD9152,
	ID_AD9154,
	ID_AD9162,
	ID_AD9162_COMPLEX,
	ID_AUTO_SYNTH_PARAM = ~0,
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

enum cf_axi_dds_ext_info {
	CHANNEL_XBAR,
};

enum ad35525_out_range {
	AD3552R_0_2_5,
	AD3552R_0_5,
	AD3552R_0_10,
	AD3552R_5_5,
	AD3552R_10_10
};

enum ad35525_source {
	AD3552R_ADC	= AXI_SEL_SRC_ADC,
	AD3552R_DMA	= AXI_SEL_SRC_DMA,
	AD3552R_RAMP	= AXI_SEL_SRC_DDS
};

enum ad35525_stream_status {
	AD3552R_STOP_STREAM,
	AD3552R_START_STREAM,
	AD3552R_START_STREAM_SYNCED,
};

static const char *const stream_status[] = {
	[AD3552R_STOP_STREAM] = "stop_stream",
	[AD3552R_START_STREAM] = "start_stream",
	[AD3552R_START_STREAM_SYNCED] = "start_stream_synced"
};

static const char * const input_source[] = {
	[AD3552R_ADC]	= "adc_input",
	[AD3552R_DMA]	= "dma_input",
	[AD3552R_RAMP]	= "ramp_input"
};

static const char * const output_range[] = {
	[AD3552R_0_2_5]	= "0/2.5V",
	[AD3552R_0_5]	= "0/5V",
	[AD3552R_0_10]	= "0/10V",
	[AD3552R_5_5]	= "-5/+5V",
	[AD3552R_10_10]	= "-10/+10V"
};

struct cf_axi_converter {
	struct spi_device	*spi;
	struct clk		*clk[CLK_NUM];
	struct clock_scale	clkscale[CLK_NUM];
	void		*phy;
	struct gpio_desc			*pwrdown_gpio;
	struct gpio_desc			*reset_gpio;
	struct gpio_desc			*txen_gpio[2];
	unsigned	id;
	unsigned	interp_factor;
	unsigned	fcenter_shift;
	unsigned long	intp_modes[5];
	unsigned long	cs_modes[17];
	int		temp_calib;
	unsigned	temp_calib_code;
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
	struct iio_dev	*indio_dev;
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
void cf_axi_dds_start_sync(struct cf_axi_dds_state *st, int sync_dma);
int cf_axi_dds_pl_ddr_fifo_ctrl(struct cf_axi_dds_state *st, bool enable);
int cf_axi_dds_pl_ddr_fifo_ctrl_oneshot(struct cf_axi_dds_state *st, bool enable);

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
