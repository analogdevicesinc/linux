
/*
 * AD9854 SPI DDS driver
 *
 * Copyright 2015 SYSU SIST JinWei Hwang.
 *
 * Licensed under the GPL-2.
 */
#ifndef IIO_DDS_AD9854_H_
#define IIO_DDS_AD9854_H_

/* Registers */
#define AD9854_REG_SER_SIZE		12

#define AD9854_REG_SER_PHASE_ADJ_1		0x00
#define AD9854_REG_SER_PHASE_ADJ_2		0x01
#define AD9854_REG_SER_FREQ_TUNING_WORD_1		0x02
#define AD9854_REG_SER_FREQ_TUNING_WORD_2		0x03
#define AD9854_REG_SER_DELTA_FREQ_WORD		0x04
#define AD9854_REG_SER_UPDATE_CLOCK		0x05
#define AD9854_REG_SER_RAMP_RATE_CLOCK		0x06
#define AD9854_REG_SER_CTRL		0x07
#define AD9854_REG_SER_OUTPUT_I_MULTIPLIER		0x08
#define AD9854_REG_SER_OUTPUT_Q_MULTIPLIER		0x09
#define AD9854_REG_SER_OUTPUT_RAMP_RATE		0x0A
#define AD9854_REG_SER_QDAC		0x0B

/* Instruction byte */
#define AD9854_INST_R		(1<<7)
#define AD9854_INST_W		(0<<7)
#define AD9854_INST_ADDR_R(addr)		(AD9854_INST_R | (addr & (0x0F)))
#define AD9854_INST_ADDR_W(addr)		(AD9854_INST_W | (addr & (0x0F)))

/* control reg bits(31:24) */
#define CTRL_CR_COMP_PU		(1<<4)
#define CTRL_CR_QDAC_PD		(1<<2)
#define CTRL_CR_QDAC_PD		(1<<1)
#define CTRL_CR_DIG_PD		(1<<0)
/* control reg bits(24:16) */
#define CTRL_CR_PLL_RANGE		(1<<6)
#define CTRL_CR_BYPASS_PLL		(1<<5)
#define CTRL_CR_REF_MULT4		(1<<4)
#define CTRL_CR_REF_MULT_3		(1<<3)
#define CTRL_CR_REF_MULT_2		(1<<2)
#define CTRL_CR_REF_MULT_1		(1<<1)
#define CTRL_CR_REF_MULT_0		(1<<0)
/* control reg bits(15:8) */
#define CTRL_CR_CLR_ACC_1		(1<<7)
#define CTRL_CR_CLR_ACC_2		(1<<6)
#define CTRL_CR_TRIANGLE		(1<<5)
#define CTRL_CR_SRC_QDAC		(1<<4)
#define CTRL_CR_MODE_2		(1<<3)
#define CTRL_CR_MODE_1		(1<<2)
#define CTRL_CR_MODE_0		(1<<1)
#define CTRL_CR_IN_EXT_UP_CLK		(1<<0)
/* control reg bits(7:0) */
#define CTRL_CR_BYPASS_INV_SINC		(1<<6)
#define CTRL_CR_OSK_EN		(1<<5)
#define CTRL_CR_OSK_INT		(1<<4)
#define CTRL_CR_LSB_FIRST		(1<<1)
#define CTRL_CR_SDO_ACTIVE		(1<<0)

struct ad9854_ser_reg
{
	char reg_addr;
	unsigned int reg_len;
	char reg_val[6];
};

struct ad9854_ser_reg ad9854_ser_reg_tbl[AD9854_REG_SER_SIZE]
{
	[AD9854_REG_SER_PHASE_ADJ_1] = {
		.reg_addr = AD9854_REG_SER_PHASE_ADJ_1,
		.reg_len = 2,
		.reg_val[0 ... 5] = 0x00,
	},

	[AD9854_REG_SER_PHASE_ADJ_2] = {
		.reg_addr = AD9854_REG_SER_PHASE_ADJ_2,
		.reg_len = 2,
		.reg_val[0 ... 5] = 0x00,
	},

	[AD9854_REG_SER_FREQ_TUNING_WORD_1] = {
		.reg_addr = AD9854_REG_SER_FREQ_TUNING_WORD_1,
		.reg_len = 6,
		.reg_val[0 ... 5] = 0x00,
	},

	[AD9854_REG_SER_FREQ_TUNING_WORD_2] = {
		.reg_addr = AD9854_REG_SER_FREQ_TUNING_WORD_2,
		.reg_len = 6,
		.reg_val[0 ... 5] = 0x00,
	},

	[AD9854_REG_SER_DELTA_FREQ_WORD] = {
		.reg_addr = AD9854_REG_SER_DELTA_FREQ_WORD,
		.reg_len = 6,
		.reg_val[0 ... 5] = 0x00,
	},

	[AD9854_REG_SER_UPDATE_CLOCK] = {
		.reg_addr = AD9854_REG_SER_UPDATE_CLOCK,
		.reg_len = 4,
		.reg_val = { [0] = 0x40},
	},

	[AD9854_REG_SER_RAMP_RATE_CLOCK] = {
		.reg_addr = AD9854_REG_SER_RAMP_RATE_CLOCK,
		.reg_len = 3,
		.reg_val[0 ... 5] = 0x00,
	},

	[AD9854_REG_SER_CTRL] = {
		.reg_addr = AD9854_REG_SER_CTRL,
		.reg_len = 4,
		.reg_val = {
			[0] = 0x20,
			[1] = 0x01,
			[2] = 0x64,
			[3] = 0x10,
		},
	},

	[AD9854_REG_SER_OUTPUT_I_MULTIPLIER] = {
		.reg_addr = AD9854_REG_SER_OUTPUT_I_MULTIPLIER,
		.reg_len = 2,
		.reg_val[0 ... 5] = 0x00,
	},

	[AD9854_REG_SER_OUTPUT_Q_MULTIPLIER] = {
		.reg_addr = AD9854_REG_SER_OUTPUT_Q_MULTIPLIER,
		.reg_len = 2,
		.reg_val[0 ... 5] = 0x00,
	},

	[AD9854_REG_SER_OUTPUT_RAMP_RATE] = {
		.reg_addr = AD9854_REG_SER_OUTPUT_RAMP_RATE,
		.reg_len = 1,
		.reg_val = { [0] = 0x80 },
	},

	[AD9854_REG_SER_QDAC] = {
		.reg_addr = AD9854_REG_SER_QDAC,
		.reg_len = 2,
		.reg_val[0 ... 5] = 0x00,
	},

};
/**
 * struct ad9854_state - driver instance specific data
 * @spi:		spi_device
 * @reg:		supply regulator
 * @mclk:		external master clock
 * @control:		cached control word
 * @xfer:		default spi transfer
 * @msg:		default spi message
 * @freq_xfer:		tuning word spi transfer
 * @freq_msg:		tuning word spi message
 * @data:		spi transmit buffer
 * @freq_data:		tuning word spi transmit buffer
 */

struct ad9834_state {
	struct spi_device		*spi;
	struct regulator		*reg;
	unsigned int			mclk;
	unsigned short			control;
	unsigned short			devid;
	struct spi_transfer		xfer;
	struct spi_message		msg;
	struct spi_transfer		freq_xfer[2];
	struct spi_message		freq_msg;

	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	__be16				data ____cacheline_aligned;
	__be16				freq_data[2];
};


/**
 * struct ad9854_platform_data - platform specific information
 * @mclk:		master clock in Hz
 * @freq0:		power up freq0 tuning word in Hz
 * @freq1:		power up freq1 tuning word in Hz
 * @phase0:		power up phase0 value [0..4095] correlates with 0..2PI
 * @phase1:		power up phase1 value [0..4095] correlates with 0..2PI
 * @en_div2:		digital output/2 is passed to the SIGN BIT OUT pin
 * @en_signbit_msb_out:	the MSB (or MSB/2) of the DAC data is connected to the
 *			SIGN BIT OUT pin. en_div2 controls whether it is the MSB
 *			or MSB/2 that is output. if en_signbit_msb_out=false,
 *			the on-board comparator is connected to SIGN BIT OUT
 */

struct ad9854_platform_data {
	unsigned int		mclk;
	unsigned int		freq0;
	unsigned int		freq1;
	unsigned short		phase0;
	unsigned short		phase1;
	bool			en_div2;
	bool			en_signbit_msb_out;
	unsigned		gpio_io_ud_clk;
	unsigned		gpio_m_reset;
	unsigned		gpio_io_reset;
	unsigned		gpio_sp_select;
};

/**
 * ad9854_supported_device_ids:
 */

enum ad9854_supported_device_ids {
	ID_AD9854,
};

#endif /* IIO_DDS_AD9854_H_ */
