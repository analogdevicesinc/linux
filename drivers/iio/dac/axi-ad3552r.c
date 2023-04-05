// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * AXI AD3552R Driver
 *
 * Copyright (C) 2022 Analog Devices, Inc.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>

#define AXI_REG_RSTN				0x40
#define   AXI_MSK_RSTN				BIT(0)
#define   AXI_MSK_MMCM_RSTN			BIT(1)
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
#define AD3552R_ID				0x4008

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
	void __iomem *regs;
	struct clk *ref_clk;
	struct device *dev;
	bool ddr;
	bool single_channel;
};

struct reg_addr_poll {
	struct axi_ad3552r_state *st;
	u8 reg;
};

static void axi_ad3552r_write(struct axi_ad3552r_state *st, u32 reg, u32 val)
{
	iowrite32(val, st->regs + reg);
}

static u32 axi_ad3552r_read(struct axi_ad3552r_state *st, u32 reg)
{
	return ioread32(st->regs + reg);
}

static u32 axi_ad3552r_read_wrapper(struct reg_addr_poll *addr)
{
	return axi_ad3552r_read(addr->st, addr->reg);
}

static void axi_ad3552r_update_bits(struct axi_ad3552r_state *st, u32 reg,
				    u32 mask, u32 val)
{
	u32 tmp, orig;

	orig = axi_ad3552r_read(st, reg);
	tmp = orig & ~mask;
	tmp |= val & mask;

	if (tmp != orig)
		axi_ad3552r_write(st, reg, tmp);
}

static void axi_ad3552r_spi_write(struct axi_ad3552r_state *st, u32 reg, u32 val,
				  u32 transfer_params)
{
	struct reg_addr_poll addr;
	u32 check;

	if (transfer_params & AXI_MSK_SYMB_8B)
		axi_ad3552r_write(st, AXI_REG_CNTRL_DATA_WR, CNTRL_DATA_WR_8(val));
	else
		axi_ad3552r_write(st, AXI_REG_CNTRL_DATA_WR, CNTRL_DATA_WR_16(val));

	axi_ad3552r_write(st, AXI_REG_CNTRL_2, transfer_params);

	axi_ad3552r_update_bits(st, AXI_REG_CNTRL_CSTM, AXI_MSK_ADDRESS,
				CNTRL_CSTM_ADDR(reg));
	axi_ad3552r_update_bits(st, AXI_REG_CNTRL_CSTM, AXI_MSK_TRANSFER_DATA,
				AXI_MSK_TRANSFER_DATA);
	addr.st = st;
	addr.reg = AXI_REG_UI_STATUS;
	readx_poll_timeout(axi_ad3552r_read_wrapper, &addr, check,
			   check == AXI_MSK_BUSY, 10, 100);

	axi_ad3552r_update_bits(st, AXI_REG_CNTRL_CSTM, AXI_MSK_TRANSFER_DATA, 0);
}

static u32 axi_ad3552r_spi_read(struct axi_ad3552r_state *st, u32 reg,
				u32 transfer_params)
{
	axi_ad3552r_spi_write(st, RD_ADDR(reg), 0x00, transfer_params);
	return axi_ad3552r_read(st, AXI_REG_CNTRL_DATA_RD);
}

static void axi_ad3552r_spi_update_bits(struct axi_ad3552r_state *st, u32 reg,
					u32 mask, u32 val, u32 transfer_params)
{
	u32 tmp, orig;

	orig = axi_ad3552r_spi_read(st, reg, transfer_params);
	tmp = orig & ~mask;
	tmp |= val & mask;

	if (tmp != orig)
		axi_ad3552r_spi_write(st, reg, tmp, transfer_params);
}

static int axi_ad3552r_read_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int *val,
				int *val2,
				long mask)
{
	struct axi_ad3552r_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (st->ddr)
			if (st->single_channel)
				*val = DIV_ROUND_UP(clk_get_rate(st->ref_clk),4);
			else
				*val = DIV_ROUND_UP(clk_get_rate(st->ref_clk),8);
		else
			if (st->single_channel)
				*val = DIV_ROUND_UP(clk_get_rate(st->ref_clk),8);
			else
				*val = DIV_ROUND_UP(clk_get_rate(st->ref_clk),16);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_RAW:
		if (chan->channel) {
			*val = axi_ad3552r_spi_read(st, AD3552R_REG_CH1_DAC_16B,
						    AD3552R_TFER_16BIT_SDR);
		} else {
			*val = axi_ad3552r_spi_read(st, AD3552R_REG_CH0_DAC_16B,
						    AD3552R_TFER_16BIT_SDR);
		}
		return IIO_VAL_INT;
	}

	return -EINVAL;
}

static int axi_ad3552r_write_raw(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan,
				 int val,
				 int val2,
				 long mask)
{
	struct axi_ad3552r_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (chan->channel)
			axi_ad3552r_spi_write(st, AD3552R_REG_CH1_DAC_16B,
					      (u32)val, AD3552R_TFER_16BIT_SDR);
		else
			axi_ad3552r_spi_write(st, AD3552R_REG_CH0_DAC_16B,
					      (u32)val, AD3552R_TFER_16BIT_SDR);
		return 0;
	}

	return -EINVAL;
}

static int axi_ad3552r_reg_access(struct iio_dev *indio_dev,
				  unsigned int reg, unsigned int writeval,
				  unsigned int *readval)
{
	struct axi_ad3552r_state *st = iio_priv(indio_dev);

	if (readval) {
		*readval = axi_ad3552r_read(st, reg);
		return 0;
	}

	axi_ad3552r_write(st, reg, writeval);
	return 0;
}

static int ad3552r_set_output_range(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    unsigned int mode)
{
	struct axi_ad3552r_state *st = iio_priv(indio_dev);

	if (chan->channel)
		axi_ad3552r_spi_update_bits(st, AD3552R_REG_OUTPUT_RANGE,
					    AD3552R_MASK_CH1_RANGE,
					    SET_CH1_RANGE(mode),
					    AD3552R_TFER_8BIT_SDR);
	else
		axi_ad3552r_spi_update_bits(st, AD3552R_REG_OUTPUT_RANGE,
					    AD3552R_MASK_CH0_RANGE,
					    SET_CH0_RANGE(mode),
					    AD3552R_TFER_8BIT_SDR);
	mdelay(100);

	return 0;
}

static int ad3552r_get_output_range(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan)
{
	struct axi_ad3552r_state *st = iio_priv(indio_dev);
	u32 val;

	val = axi_ad3552r_spi_read(st, AD3552R_REG_OUTPUT_RANGE,
				   AD3552R_TFER_8BIT_SDR);
	if (chan->channel)
		return GET_CH1_RANGE(val);
	else
		return GET_CH0_RANGE(val);
}

static int ad3552r_set_input_source(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    unsigned int mode)
{
	struct axi_ad3552r_state *st = iio_priv(indio_dev);

	axi_ad3552r_write(st, AXI_REG_CHAN_CNTRL_7_CH0, mode);
	axi_ad3552r_write(st, AXI_REG_CHAN_CNTRL_7_CH1, mode);

	return 0;
}

static int ad3552r_get_input_source(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan)
{
	struct axi_ad3552r_state *st = iio_priv(indio_dev);

	return axi_ad3552r_read(st, AXI_REG_CHAN_CNTRL_7_CH0);
}

static const struct iio_enum ad35525_source_enum = {
	.items = input_source,
	.num_items = ARRAY_SIZE(input_source),
	.get = ad3552r_get_input_source,
	.set = ad3552r_set_input_source,
};

static const struct iio_enum ad35525_output_enum = {
	.items = output_range,
	.num_items = ARRAY_SIZE(output_range),
	.get = ad3552r_get_output_range,
	.set = ad3552r_set_output_range,
};

static const struct iio_chan_spec_ext_info ad3552r_ext_info[] = {
	IIO_ENUM("input_source", IIO_SHARED_BY_ALL, &ad35525_source_enum),
	IIO_ENUM_AVAILABLE_SHARED("input_source", IIO_SHARED_BY_ALL,
				  &ad35525_source_enum),
	IIO_ENUM("output_range", IIO_SEPARATE, &ad35525_output_enum),
	IIO_ENUM_AVAILABLE_SHARED("output_range", IIO_SEPARATE,
				  &ad35525_output_enum),
	{},
};

static const struct iio_chan_spec axi_ad3552r_channels[] = {
	{
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.output = 1,
		.indexed = 1,
		.channel = 0,
		.scan_index = 0,
		.ext_info = ad3552r_ext_info,
		.scan_type = {
			.sign = 'u',
			.realbits = 16,
			.storagebits = 16,
			.shift = 0,
			.endianness = IIO_BE,
		},
	},
	{
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.output = 1,
		.indexed = 1,
		.channel = 1,
		.scan_index = 1,
		.ext_info = ad3552r_ext_info,
		.scan_type = {
			.sign = 'u',
			.realbits = 16,
			.storagebits = 16,
			.shift = 0,
			.endianness = IIO_BE,
		},
	}
};

static int axi_ad3552r_update_scan_mode(struct iio_dev *indio_dev,
					const unsigned long *active_scan_mask)
{
	struct axi_ad3552r_state *st = iio_priv(indio_dev);

	st->ddr = true;
	axi_ad3552r_spi_update_bits(st, AD3552R_REG_INTERFACE_CONFIG_D,
				    AD3552R_MASK_SPI_CONFIG_DDR,
				    AD3552R_MASK_SPI_CONFIG_DDR,
				    AD3552R_TFER_8BIT_SDR);

	switch (*active_scan_mask) {
	case AD3552R_CH0_ACTIVE:
		st->single_channel = true;
		axi_ad3552r_spi_write(st, AD3552R_REG_STREAM_MODE,
				      AD3552R_STREAM_2BYTE_LOOP,
				      AD3552R_TFER_8BIT_DDR);
		axi_ad3552r_update_bits(st, AXI_REG_CNTRL_CSTM, AXI_MSK_ADDRESS,
					CNTRL_CSTM_ADDR(AD3552R_REG_CH0_DAC_16B));
		return 0;
	case AD3552R_CH1_ACTIVE:
		st->single_channel = true;
		axi_ad3552r_spi_write(st, AD3552R_REG_STREAM_MODE,
				      AD3552R_STREAM_2BYTE_LOOP,
				      AD3552R_TFER_8BIT_DDR);
		axi_ad3552r_update_bits(st, AXI_REG_CNTRL_CSTM, AXI_MSK_ADDRESS,
					CNTRL_CSTM_ADDR(AD3552R_REG_CH1_DAC_16B));
		return 0;
	case AD3552R_CH0_CH1_ACTIVE:
		st->single_channel = false;
		axi_ad3552r_spi_write(st, AD3552R_REG_STREAM_MODE,
				      AD3552R_STREAM_4BYTE_LOOP,
				      AD3552R_TFER_8BIT_DDR);
		axi_ad3552r_update_bits(st, AXI_REG_CNTRL_CSTM, AXI_MSK_ADDRESS,
					CNTRL_CSTM_ADDR(AD3552R_REG_CH1_DAC_16B));
		return 0;
	}

	return -EINVAL;
}

static const struct iio_info axi_ad3552r_info = {
	.read_raw = &axi_ad3552r_read_raw,
	.write_raw = &axi_ad3552r_write_raw,
	.debugfs_reg_access = &axi_ad3552r_reg_access,
	.update_scan_mode = axi_ad3552r_update_scan_mode,
};

static int axi_ad3552r_buffer_postenable(struct iio_dev *indio_dev)
{
	struct axi_ad3552r_state *st = iio_priv(indio_dev);

	st->ddr = true;
	axi_ad3552r_write(st, AXI_REG_CNTRL_2, AXI_MSK_USIGN_DATA |
			  ~AXI_MSK_SDR_DDR_N);
	axi_ad3552r_update_bits(st, AXI_REG_CNTRL_CSTM, AD3552R_STREAM_SATRT,
				AD3552R_STREAM_SATRT);
	return 0;
}

static int axi_ad3552r_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct axi_ad3552r_state *st = iio_priv(indio_dev);

	st->ddr = false;
	axi_ad3552r_update_bits(st, AXI_REG_CNTRL_CSTM, AD3552R_STREAM_SATRT, 0);
	axi_ad3552r_spi_update_bits(st, AD3552R_REG_INTERFACE_CONFIG_D,
				    AD3552R_MASK_SPI_CONFIG_DDR, 0,
				    AD3552R_TFER_8BIT_DDR);

	return 0;
}

static int axi_ad3552r_dma_buffer_submit(struct iio_dma_buffer_queue *queue,
					 struct iio_dma_buffer_block *block)
{
	return iio_dmaengine_buffer_submit_block(queue, block, DMA_TO_DEVICE);
}

static const struct iio_buffer_setup_ops axi_ad3552r_buffer_setup_ops = {
	.postenable = axi_ad3552r_buffer_postenable,
	.postdisable = axi_ad3552r_buffer_postdisable,
};

static const struct iio_dma_buffer_ops axi_ad3552r_dma_buffer_ops = {
	.submit = axi_ad3552r_dma_buffer_submit,
	.abort = iio_dmaengine_buffer_abort,
};

static int axi_ad3552r_reset(struct axi_ad3552r_state *st)
{
	axi_ad3552r_update_bits(st, AXI_REG_RSTN, AXI_RST, 0x00);
	axi_ad3552r_update_bits(st, AXI_REG_RSTN, AXI_RST, AXI_RST);

	st->reset_gpio = devm_gpiod_get_optional(st->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(st->reset_gpio))
		return PTR_ERR(st->reset_gpio);

	if (st->reset_gpio) {
		gpiod_set_value_cansleep(st->reset_gpio, 1);
		usleep_range(1, 10);
		gpiod_set_value_cansleep(st->reset_gpio, 0);
	} else {
		axi_ad3552r_spi_update_bits(st, AD3552R_REG_INTERFACE_CONFIG_A,
					    AD3552R_MASK_SW_RST,
					    AD3552R_MASK_SW_RST,
					    AD3552R_TFER_8BIT_SDR);
	}
	msleep_interruptible(100);

	return 0;
}

static int axi_ad3552r_setup(struct axi_ad3552r_state *st)
{
	u8 val;
	u16 id;
	int ret;

	ret = axi_ad3552r_reset(st);
	if (ret)
		return ret;

	axi_ad3552r_spi_write(st, AD3552R_REG_SCRATCH_PAD,
			      AD3552R_SCRATCH_PAD_TEST_VAL,
			      AD3552R_TFER_8BIT_SDR);
	val = axi_ad3552r_spi_read(st, AD3552R_REG_SCRATCH_PAD,
				   AD3552R_TFER_8BIT_SDR);

	if (val != AD3552R_SCRATCH_PAD_TEST_VAL)
		return -EINVAL;

	val = axi_ad3552r_spi_read(st, AD3552R_REG_PRODUCT_ID_L,
				   AD3552R_TFER_8BIT_SDR);

	id = val;
	val = axi_ad3552r_spi_read(st, AD3552R_REG_PRODUCT_ID_H,
				   AD3552R_TFER_8BIT_SDR);

	id |= val << 8;
	if (id != AD3552R_ID)
		return -ENODEV;

	axi_ad3552r_spi_write(st, AD3552R_REG_REF_CONFIG, AD3552R_REF_INIT,
			      AD3552R_TFER_8BIT_SDR);
	axi_ad3552r_spi_write(st, AD3552R_REG_TRANSFER, AD3552R_TRANSFER_INIT,
			      AD3552R_TFER_8BIT_SDR);

	return 0;
}

static int axi_ad3552r_probe(struct platform_device *pdev)
{
	struct axi_ad3552r_state *st;
	struct iio_dev *indio_dev;
	struct iio_buffer *buffer;
	struct resource *mem;
	int ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	st->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(st->regs))
		return PTR_ERR(st->regs);

	st->ref_clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(st->ref_clk))
		return PTR_ERR(st->ref_clk);

	ret = clk_prepare_enable(st->ref_clk);
	if (ret < 0)
		return ret;

	st->dev = &pdev->dev;

	indio_dev->name = pdev->dev.of_node->name;
	indio_dev->modes = INDIO_BUFFER_HARDWARE;
	indio_dev->direction = IIO_DEVICE_DIRECTION_OUT;
	indio_dev->setup_ops = &axi_ad3552r_buffer_setup_ops;
	indio_dev->channels = axi_ad3552r_channels;
	indio_dev->num_channels = ARRAY_SIZE(axi_ad3552r_channels);
	indio_dev->info = &axi_ad3552r_info;

	buffer = devm_iio_dmaengine_buffer_alloc(indio_dev->dev.parent, "tx",
						 &axi_ad3552r_dma_buffer_ops,
						 indio_dev);
	if (IS_ERR(buffer))
		return PTR_ERR(buffer);

	iio_device_attach_buffer(indio_dev, buffer);

	ret = axi_ad3552r_setup(st);
	if (ret)
		return ret;

	return devm_iio_device_register(&pdev->dev, indio_dev);
}

static const struct of_device_id axi_ad3552r_of_id[] = {
	{ .compatible = "adi,axi-ad3552r" },
	{}
};
MODULE_DEVICE_TABLE(of, axi_ad3552r_of_id);

static struct platform_driver axi_ad3552r_driver = {
	.driver = {
		.name = "axi-ad3552r",
		.owner = THIS_MODULE,
		.of_match_table = axi_ad3552r_of_id,
	},
	.probe = axi_ad3552r_probe,
};
module_platform_driver(axi_ad3552r_driver);

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("AXI AD3552R Driver");
MODULE_LICENSE("GPL v2");
