/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Analog Devices AD3552R
 * Digital to Analog converter driver
 *
 * Copyright 2023 Analog Devices, Inc.
 */

#ifndef __DRIVERS_IIO_ADC_AD3552R_BASE_H__
#define __DRIVERS_IIO_ADC_AD3552R_BASE_H__

// axi-ad3552r "header" {
#define AXI_REG_RSTN				0x40
#define   AXI_MSK_RSTN				BIT(0)
#define   AXI_MSK_MMCM_RSTN			BIT(1)
#define AXI_REG_CNTRL_1             0x44
#define AXI_REG_CNTRL_2				0x48
#define   AXI_MSK_USIGN_DATA			BIT(4)
#define   AXI_MSK_SYMB_8B			BIT(14)
#define   AXI_MSK_SDR_DDR_N			BIT(16)
#define AXI_REG_CNTRL_DATA_RD			0x80
#define   AXI_MSK_DATA_RD_8			GENMASK(7, 0)
#define   AXI_MSK_DATA_RD_16			GENMASK(15, 0)
#define AXI_REG_CNTRL_DATA_WR			0x84
#define   AXI_MSK_DATA_WR_8			GENMASK(23, 16)
#define   AXI_MSK_DATA_WR_16			GENMASK(23, 8)
#define AXI_REG_UI_STATUS			0x88
#define   AXI_MSK_BUSY				BIT(4)
#define AXI_REG_CNTRL_CSTM			0x8C
#define   AXI_MSK_TRANSFER_DATA			BIT(0)
#define   AXI_MSK_STREAM			BIT(1)
#define   AXI_MSK_SYNCED_TRANSFER		BIT(2)
#define   AXI_MSK_ADDRESS			GENMASK(31, 24)
#define AXI_REG_CHAN_CNTRL_7_CH0		0x418
#define AXI_REG_CHAN_CNTRL_7_CH1		0x458
#define   AXI_EXT_SYNC_ARM             0x02
#define   AXI_SEL_SRC_DMA			0x02
#define   AXI_SEL_SRC_ADC			0x08
#define   AXI_SEL_SRC_DDS			0x0b

#define AD3552R_REG_INTERFACE_CONFIG_A		0x00
#define   AD3552R_MASK_SW_RST			(BIT(7) | BIT(0))
#define AD3552R_REG_PRODUCT_ID_L		0x04
#define AD3552R_REG_PRODUCT_ID_H		0x05
#define AD3552R_REG_SCRATCH_PAD			0x0A
#define AD3552R_REG_STREAM_MODE			0x0E
#define AD3552R_REG_TRANSFER			0x0F
#define   AD3552R_MASK_STREAM_LENGTH_KEEP	BIT(2)
#define   AD3552R_MASK_MULTI_IO_MODE		GENMASK(7, 6)
#define AD3552R_REG_INTERFACE_CONFIG_D		0x14
#define   AD3552R_MASK_SPI_CONFIG_DDR		BIT(0)
#define   AD3552R_MASK_DUAL_SPI_SYNC_EN		BIT(1)
#define   AD3552R_MASK_SDO_DRIVE_STRENGTH	GENMASK(3, 2)
#define   AD3552R_MASK_MEM_CRC_EN		BIT(4)
#define   AD3552R_MASK_ALERT_ENABLE_PULLUP	BIT(6)
#define	AD3552R_REG_REF_CONFIG			0x15
#define AD3552R_REG_OUTPUT_RANGE		0x19
#define   AD3552R_MASK_OUT_RANGE        GENMASK(7, 0)
#define   AD3552R_MASK_CH0_RANGE		GENMASK(2, 0)
#define   AD3552R_MASK_CH1_RANGE		GENMASK(6, 4)
#define AD3552R_REG_CH0_DAC_16B			0x2A
#define AD3552R_REG_CH1_DAC_16B			0x2C

#define AD3552R_TFER_8BIT_SDR			(AXI_MSK_SYMB_8B | \
						AXI_MSK_SDR_DDR_N)
#define AD3552R_TFER_8BIT_DDR			AXI_MSK_SYMB_8B
#define AD3552R_TFER_16BIT_SDR			AXI_MSK_SDR_DDR_N
#define AD3552R_TFER_16BIT_DDR			0x00

#define AD3552R_SINGLE_SPI			0x00
#define AD3552R_DUAL_SPI			0x01
#define AD3552R_QUAD_SPI			0x02

#define AD3552R_SCRATCH_PAD_TEST_VAL		0x5A
//#define AD3552R_ID				0x4008

#define AD3552R_REF_INIT			0x00
#define AD3552R_TRANSFER_INIT			(FIELD_PREP(AD3552R_MASK_MULTI_IO_MODE,\
							    AD3552R_QUAD_SPI) |\
						AD3552R_MASK_STREAM_LENGTH_KEEP)

#define AD3552R_STREAM_2BYTE_LOOP		0x02
#define AD3552R_STREAM_4BYTE_LOOP		0x04
#define AD3552R_STREAM_SATRT			(AXI_MSK_TRANSFER_DATA | \
						AXI_MSK_STREAM)

#define AD3552R_CH0_ACTIVE			BIT(0)
#define AD3552R_CH1_ACTIVE			BIT(1)
#define AD3552R_CH0_CH1_ACTIVE			(AD3552R_CH0_ACTIVE | \
						 AD3552R_CH1_ACTIVE)

#define AXI_RST					(AXI_MSK_RSTN | AXI_MSK_MMCM_RSTN)

#define CNTRL_CSTM_ADDR(x)			FIELD_PREP(AXI_MSK_ADDRESS, x)
#define CNTRL_DATA_WR_8(x)			FIELD_PREP(AXI_MSK_DATA_WR_8, x)
#define CNTRL_DATA_WR_16(x)			FIELD_PREP(AXI_MSK_DATA_WR_16, x)

#define RD_ADDR(x)				(BIT(7) | (x))

#define SET_CH0_RANGE(x)			FIELD_PREP(AD3552R_MASK_CH0_RANGE, x)
#define SET_CH1_RANGE(x)			FIELD_PREP(AD3552R_MASK_CH1_RANGE, x)
#define GET_CH0_RANGE(x)			FIELD_GET(AD3552R_MASK_CH0_RANGE, x)
#define GET_CH1_RANGE(x)			FIELD_GET(AD3552R_MASK_CH1_RANGE, x)

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

struct axi_ad3552r_state {
	struct gpio_desc *reset_gpio;
	struct clk *ref_clk;
	struct device *dev;
	/* protect device accesses */
	struct mutex lock;
	bool has_lock;
	bool ddr;
	bool single_channel;
	bool synced_transfer;
};

struct reg_addr_poll {
	struct axi_ad3552r_state *st;
	u8 reg;
};

// } axi-ad3552r "header"


enum ad3542r_id {
	AD3542R_ID = 0x4009,
	AD3552R_ID = 0x4008,
};

#endif /* __DRIVERS_IIO_ADC_AD3552R_BASE_H__ */
