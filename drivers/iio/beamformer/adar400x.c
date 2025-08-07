// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
/*
 * Analog Devices ADAR400x Driver
 *
 * Copyright 2024 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/iio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>
#include <linux/iio/sysfs.h>

/* ADAR400x Registers */
#define ADAR400X_SPI_CONFIG			0x00
#define ADAR400X_0001				0x01
#define ADAR400X_CHIP_TYPE			0x03
#define ADAR400X_PROD_ID_L			0x04
#define ADAR400X_PROD_ID_H			0x05
#define ADAR400X_ADDR_PAGE			0x08
#define ADAR400X_SCRATCH_PAD			0x0A
#define ADAR400X_BEAMSTATE_MODE			0x11
#define ADAR400X_BM0_MEM_SEQ_PTR_START		0x15
#define ADAR400X_BM0_MEM_SEQ_PTR_STOP		0x16
#define ADAR400X_PIN_SPI_CTRL			0x40
#define ADAR400X_ALL_BEAM_MUTE_RESET		0x41
#define ADAR400X_ALL_BEAM_UPDATE		0x42
#define ADAR400X_BM0_FIFO_WRITE_PTR		0x50
#define ADAR400X_BM0_FIFO_READ_PTR		0x51
#define ADAR400X_BM0_SEQUENCER_PTR		0x52
#define ADAR400X_RESET_DELAY_BM0_EL(x)		(0x60 + (2 * x))
#define ADAR400X_RESET_ATTN_BM0_EL(x)		(0x61 + (2 * x))
#define ADAR400X_MUTE_DELAY_BM0_EL(x)		(0x80 + (2 * x))
#define ADAR400X_MUTE_ATTN_BM0_EL(x)		(0x81 + (2 * x))
#define ADAR400X_ELX_A_RESET_AMP(x)		0xA0 + (x)
#define ADAR400X_ELX_A_OP_AMP(x)		0xA4 + (x)
#define ADAR400X_ELX_A_MUTE_AMP(x)		0xA8 + (x)
#define ADAR400X_ELX_A_SLEEP_AMP(x)		0xAC + (x)
#define ADAR400X_ELX_B_RESET_AMP(x)		0xB0 + (x)
#define ADAR400X_ELX_B_OP_AMP(x)		0xB4 + (x)
#define ADAR400X_ELX_B_MUTE_AMP(x)		0xB8 + (x)
#define ADAR400X_ELX_B_SLEEP_AMP(x)		0xBC + (x)

/* RAM / FIFO */
#define ADAR400X_DIR_CTL_DELAY_BM0_EL(x)	(0x100 + (2 * x))
#define ADAR400X_DIR_CTL_ATTN_BM0_EL(x)		(0x101 + (2 * x))
#define ADAR400X_BM0_SEQ_PTR(x)			0x130 + (x)
#define ADAR400X_BEAMSTATE_MEMORY(x)		(0x100 + (7 * x))

/* Update SPI PinB Ctl */
#define ADAR400X_SPI_PIN_CTL			BIT(0)

/* Element Amp Bias and En Masks */
#define ADAR400X_EL_AMP_BIAS_MASK		GENMASK(2, 0)
#define ADAR400X_EL_AMP_EN_MASK			BIT(3)

/* Other Macros */
#define ADAR400X_SPI_SETUP			0xBD
#define ADAR4000_CHIP_ID			0x4000
#define ADAR4001_CHIP_ID			0x4001
#define ADAR4002_CHIP_ID			0x4002
#define ADAR400X_SPI_ADDR_MASK			GENMASK(13, 10)
#define ADAR400X_SPI_DEV_ADDR(x)		FIELD_PREP(ADAR400X_SPI_ADDR_MASK, (x))
#define ADAR400X_MAX_DEV			16
#define ADAR400X_MAX_FIFO_STATES		16
#define ADAR400X_ELEMENT_PER_BEAM		4
#define ADAR400x_RAM_BEAMSTATE_PER_INDEX	7
#define ADAR400X_MAX_LINE_SIZE			256

/* Limits */
#define ADAR400X_MAX_ATTENUATION_DB		31.5
#define ADAR400X_MAX_ATTENUATION_RAW		0x3f
#define ADAR400X_TD1_MASK			0x80
#define ADAR400X_MAX_DELAY_RAW			0xff
#define ADAR400X_MAX_DELAY_RANGE		128
#define ADAR400X_MAX_DELAY_TD0_PS		508
#define ADAR400X_MAX_DELAY_TD1_PS		254
#define ADAR400X_MAX_AMP_BIAS			0x7
#define ADAR400X_MAX_PTR_INDEX			0x3f
#define ADAR400X_8BIT_MAX_DATA			0xff

/* RAM Registers */
#define ADAR400X_RAM_EL0_DELAY			0x100
#define ADAR400X_RAM_EL0_ATTN			0x101
#define ADAR400X_RAM_EL1_DELAY_H		0x101
#define ADAR400X_RAM_EL1_DELAY_L		0x102
#define ADAR400X_RAM_EL1_ATTN_H			0x102
#define ADAR400X_RAM_EL1_ATTN_L			0x103
#define ADAR400X_RAM_EL2_DELAY_H		0x103
#define ADAR400X_RAM_EL2_DELAY_L		0x104
#define ADAR400X_RAM_EL2_ATTN_H			0x104
#define ADAR400X_RAM_EL2_ATTN_L			0x105
#define ADAR400X_RAM_EL3_DELAY_H		0x105
#define ADAR400X_RAM_EL3_DELAY_L		0x106
#define ADAR400X_RAM_EL3_ATTN			0x106

/* RAM/FIFO MASKS */
#define ADAR400X_RAM_EL0_DELAY_MSK		GENMASK(7, 0)
#define ADAR400X_RAM_EL0_ATTN_MSK		GENMASK(7, 2)
#define ADAR400X_RAM_EL1_DELAY_H_MSK		GENMASK(1, 0)
#define ADAR400X_RAM_EL1_DELAY_L_MSK		GENMASK(7, 2)
#define ADAR400X_RAM_EL1_ATTN_H_MSK		GENMASK(1, 0)
#define ADAR400X_RAM_EL1_ATTN_L_MSK		GENMASK(7, 4)
#define ADAR400X_RAM_EL2_DELAY_H_MSK		GENMASK(3, 0)
#define ADAR400X_RAM_EL2_DELAY_L_MSK		GENMASK(7, 4)
#define ADAR400X_RAM_EL2_ATTN_H_MSK		GENMASK(3, 0)
#define ADAR400X_RAM_EL2_ATTN_L_MSK		GENMASK(7, 6)
#define ADAR400X_RAM_EL3_DELAY_H_MSK		GENMASK(5, 0)
#define ADAR400X_RAM_EL3_DELAY_L_MSK		GENMASK(7, 6)
#define ADAR400X_RAM_EL3_ATTN_MSK		GENMASK(5, 0)


enum adar400x_ids {
	ID_ADAR4000 = ADAR4000_CHIP_ID,
	ID_ADAR4001,
};

enum adar400x_elements {
	ADAR400X_EL0,
	ADAR400X_EL1,
	ADAR400X_EL2,
	ADAR400X_EL3,
};

enum adar400x_address_pages {
	ADAR400X_CONFIG_PAGE,
	ADAR400X_BEAM0_RAM_PAGE,
	ADAR400X_FIFO_LOAD_PAGE = 0x5,
};

static const char *const adar400x_address_pages_string[] = {
	[ADAR400X_CONFIG_PAGE] = "config",
	[ADAR400X_BEAM0_RAM_PAGE] = "ram",
	[ADAR400X_BEAM0_RAM_PAGE + 1] = "unused_1",
	[ADAR400X_BEAM0_RAM_PAGE + 2] = "unused_2",
	[ADAR400X_BEAM0_RAM_PAGE + 3] = "unused_3",
	[ADAR400X_FIFO_LOAD_PAGE] = "fifo",
};

enum adar400x_bm0_modes {
	ADAR400X_DIRECT_CONTROL_MODE,
	ADAR400X_MEMORY_CONTROL_MODE,
	ADAR400X_FIFO_CONTROL_MODE,
	ADAR400X_INST_DIRECT_CONTROL_MODE,
	ADAR400X_MUTE_MODE,
	ADAR400X_RESET_MODE,
};

static const char * const adar400x_bm0_modes_string[] = {
	[ADAR400X_DIRECT_CONTROL_MODE] = "direct",
	[ADAR400X_MEMORY_CONTROL_MODE] = "ram",
	[ADAR400X_FIFO_CONTROL_MODE] = "fifo",
	[ADAR400X_INST_DIRECT_CONTROL_MODE] = "instant_direct",
	[ADAR400X_MUTE_MODE] = "mute",
	[ADAR400X_RESET_MODE] = "reset",
};

enum adar400x_beam_cmd {
	ADAR400X_MUTE,
	ADAR400X_RESET,
	ADAR400X_UPDATE,
};

static const char *const adar400x_beamstate_commands[] = {
	[ADAR400X_MUTE] = "mute",
	[ADAR400X_RESET] = "reset",
	[ADAR400X_UPDATE] = "update",
};

struct adar400x_chip_info {
	const char *name;
	const struct iio_chan_spec *channels;
	const struct iio_chan_spec_ext_info *ext_info;
	unsigned int chip_id;
	unsigned int num_channels;
};

struct adar400x_state {
	struct spi_device *spi;
	struct mutex lock;
    	struct regmap *regmap;
	struct iio_chan_spec *channels;
	struct iio_info *iio_info;
	const struct adar400x_chip_info *chip_info;
        enum adar400x_bm0_modes bm0_mode[ADAR400X_MAX_DEV]; /* data sourcing mode (bm0_modes) */
	u32 num_devices; /* number of devices detected */
	u8 active_dev_addr;
        u8 ptr_index;
	bool is_td_range0[ADAR400X_ELEMENT_PER_BEAM]; /* Last delay applied (for resolution) */
};

static int adar400x_reg_write(struct adar400x_state *st, u16 reg, u32 val)
{
        return regmap_write(st->regmap,
			    ADAR400X_SPI_DEV_ADDR(st->active_dev_addr) | reg,
			    val);
}

static int adar400x_reg_read(struct adar400x_state *st, u16 reg, u32 *val)
{
        return regmap_read(st->regmap,
			   ADAR400X_SPI_DEV_ADDR(st->active_dev_addr) | reg,
			   val);
}

static int adar400x_reg_update_bits(struct adar400x_state *st, u16 reg, u32 mask, u32 val)
{
        return regmap_update_bits(st->regmap,
				  ADAR400X_SPI_DEV_ADDR(st->active_dev_addr) | reg,
				  mask, val);
}

static int adar400x_set_page(struct adar400x_state *st, enum adar400x_address_pages page)
{
	return adar400x_reg_write(st, ADAR400X_ADDR_PAGE, page);
}

static int adar400x_get_ctl(struct adar400x_state *st, bool *is_spi_ctrl)
{
        int ret;
        u32 temp;

        ret =  adar400x_reg_read(st, ADAR400X_PIN_SPI_CTRL, &temp);
        if (ret)
                return ret;

        *is_spi_ctrl = FIELD_GET(ADAR400X_SPI_PIN_CTL, temp);

        return 0;
}

static int adar400x_beam_update_spi(struct adar400x_state *st, unsigned int val)
{
	int ret;
        bool is_spi_ctl;

	if (st->bm0_mode[st->active_dev_addr] == ADAR400X_INST_DIRECT_CONTROL_MODE)
		return -EINVAL;

	ret = adar400x_set_page(st, ADAR400X_CONFIG_PAGE);
	if (ret)
		return ret;

        /* Get control mechanism */
        ret = adar400x_get_ctl(st, &is_spi_ctl);
        if (ret)
                return ret;

        /* Exit if device isn't in SPI control mode */
        if (!is_spi_ctl)
                return -ENOSYS;

	ret = adar400x_reg_write(st, ADAR400X_ALL_BEAM_UPDATE, val);
	if (ret)
		return ret;

	return adar400x_reg_write(st, ADAR400X_ALL_BEAM_UPDATE, 0);
}

static int adar400x_beam_mute_spi(struct adar400x_state *st)
{
	int ret;
	unsigned int val;
        bool is_spi_ctl;

	if (st->bm0_mode[st->active_dev_addr] == ADAR400X_INST_DIRECT_CONTROL_MODE)
		return -EINVAL;

	ret = adar400x_set_page(st, ADAR400X_CONFIG_PAGE);
	if (ret)
		return ret;

        ret = adar400x_get_ctl(st, &is_spi_ctl);
        if (ret)
                return ret;

        /* Exit if device isn't in SPI control mode */
        if (!is_spi_ctl)
		return -ENOSYS;

	ret = adar400x_reg_update_bits(st, ADAR400X_ALL_BEAM_MUTE_RESET,
				       BIT(1), BIT(1));
	if (ret)
		return ret;

	ret = adar400x_reg_update_bits(st, ADAR400X_ALL_BEAM_MUTE_RESET,
				       BIT(0), 0);
	if (ret)
		return ret;

	ret = adar400x_reg_read(st, ADAR400X_ALL_BEAM_MUTE_RESET, &val);
	if (ret)
		return ret;

	st->bm0_mode[st->active_dev_addr] = val & BIT(1) ?
					    ADAR400X_MUTE_MODE :
					    ADAR400X_RESET_MODE;

	return 0;
}

static int adar400x_beam_reset_spi(struct adar400x_state *st)
{
	int ret;
	unsigned int val;
        bool is_spi_ctl;

	if (st->bm0_mode[st->active_dev_addr] == ADAR400X_INST_DIRECT_CONTROL_MODE)
		return -EINVAL;

	ret = adar400x_set_page(st, ADAR400X_CONFIG_PAGE);
	if (ret)
		return ret;

        ret = adar400x_get_ctl(st, &is_spi_ctl);
        if (ret)
                return ret;

        /* Exit if device isn't in SPI control mode */
        if (!is_spi_ctl)
                return -ENOSYS;

	ret = adar400x_reg_update_bits(st, ADAR400X_ALL_BEAM_MUTE_RESET,
				       BIT(0), BIT(0));
	if (ret)
		return ret;

	ret = adar400x_reg_update_bits(st, ADAR400X_ALL_BEAM_MUTE_RESET,
				       BIT(1), 0);
	if (ret)
		return ret;

	ret = adar400x_reg_read(st, ADAR400X_ALL_BEAM_MUTE_RESET, &val);
	if (ret)
		return ret;

	st->bm0_mode[st->active_dev_addr] = val & BIT(0) ?
					    ADAR400X_RESET_MODE :
					    ADAR400X_MUTE_MODE;

	return 0;
}

static int adar400x_clear_mute_reset(struct adar400x_state *st)
{
	int ret;

	ret = adar400x_set_page(st, ADAR400X_CONFIG_PAGE);
	if (ret)
		return ret;

	return adar400x_reg_write(st, ADAR400X_ALL_BEAM_MUTE_RESET, 0);
}

static int adar400x_non_memory_get_elem(struct iio_dev *indio_dev,
					struct iio_chan_spec const *chan,
					int *val)
{
	struct adar400x_state *st = iio_priv(indio_dev);

	guard(mutex)(&st->lock);
	switch (st->bm0_mode[st->active_dev_addr]) {
		case ADAR400X_MUTE_MODE:
			if (chan->type == IIO_POWER)
				return adar400x_reg_read(st, ADAR400X_MUTE_ATTN_BM0_EL(chan->channel),
							 val);
			else
				return adar400x_reg_read(st, ADAR400X_MUTE_DELAY_BM0_EL(chan->channel),
					 		 val);
		case ADAR400X_RESET_MODE:
			if (chan->type == IIO_POWER)
				return adar400x_reg_read(st, ADAR400X_RESET_ATTN_BM0_EL(chan->channel),
							 val);
			else
				return adar400x_reg_read(st, ADAR400X_RESET_DELAY_BM0_EL(chan->channel),
							 val);
		case ADAR400X_DIRECT_CONTROL_MODE:
			if (chan->type == IIO_POWER)
				return adar400x_reg_read(st, ADAR400X_DIR_CTL_ATTN_BM0_EL(chan->channel),
							 val);
			else
				return adar400x_reg_read(st, ADAR400X_DIR_CTL_DELAY_BM0_EL(chan->channel),
							 val);
		default:
			return -EINVAL;
	}
}

static int adar400x_non_memory_set_elem(struct iio_dev *indio_dev,
				 	struct iio_chan_spec const *chan,
					int val)
{
	struct adar400x_state *st = iio_priv(indio_dev);

	guard(mutex)(&st->lock);
	switch (st->bm0_mode[st->active_dev_addr]) {
		case ADAR400X_MUTE_MODE:
			if (chan->type == IIO_POWER)
				return adar400x_reg_write(st, ADAR400X_MUTE_ATTN_BM0_EL(chan->channel),
					 		  val);
			else
				return adar400x_reg_write(st, ADAR400X_MUTE_DELAY_BM0_EL(chan->channel),
					 		  val);
		case ADAR400X_RESET_MODE:
			if (chan->type == IIO_POWER)
				return adar400x_reg_write(st, ADAR400X_RESET_ATTN_BM0_EL(chan->channel),
					 		  val);
			else
				return adar400x_reg_write(st, ADAR400X_RESET_DELAY_BM0_EL(chan->channel),
					 		  val);
		case ADAR400X_DIRECT_CONTROL_MODE:
		case ADAR400X_INST_DIRECT_CONTROL_MODE:
			if (chan->type == IIO_POWER)
				return adar400x_reg_write(st, ADAR400X_DIR_CTL_ATTN_BM0_EL(chan->channel),
							  val);
			else
				return adar400x_reg_write(st, ADAR400X_DIR_CTL_DELAY_BM0_EL(chan->channel),
							  val);
		default:
			return -EINVAL;
	}
}

static int adar400x_read_beamstate_elem(struct iio_dev *indio_dev,
					struct iio_chan_spec const *chan,
					int *val)
{
	struct adar400x_state *st = iio_priv(indio_dev);

	switch (st->bm0_mode[st->active_dev_addr]) {
		case ADAR400X_DIRECT_CONTROL_MODE:
		case ADAR400X_INST_DIRECT_CONTROL_MODE:
		case ADAR400X_MUTE_MODE:
		case ADAR400X_RESET_MODE:
			return adar400x_non_memory_get_elem(indio_dev, chan, val);
		default:
			return -EINVAL;
	}
}

static int adar400x_write_beamstate_elem(struct iio_dev *indio_dev,
					 struct iio_chan_spec const *chan,
					 int val)
{
	struct adar400x_state *st = iio_priv(indio_dev);

	switch (st->bm0_mode[st->active_dev_addr]) {
		case ADAR400X_DIRECT_CONTROL_MODE:
		case ADAR400X_INST_DIRECT_CONTROL_MODE:
			return adar400x_non_memory_set_elem(indio_dev, chan, val);
		default:
			return -EINVAL;
	}
}

static int adar400x_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int *val, int *val2, long m)
{
	struct adar400x_state *st = iio_priv(indio_dev);
	int ret;

	switch (m) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->type) {
		case IIO_POWER:
		case IIO_PHASE:
			ret = adar400x_read_beamstate_elem(indio_dev, chan, val);
			if (ret)
				return ret;

			return IIO_VAL_INT;
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_POWER:
			*val = ADAR400X_MAX_ATTENUATION_DB;
			*val2 = ADAR400X_MAX_ATTENUATION_RAW;

			return IIO_VAL_FRACTIONAL;
		case IIO_PHASE:
			/* Time Delay Range 0 */
			if (st->is_td_range0[chan->scan_index])
				*val = ADAR400X_MAX_DELAY_TD0_PS;
			/* Time Delay Range 1 */
			else
				*val = ADAR400X_MAX_DELAY_TD1_PS;

			*val2 = ADAR400X_MAX_DELAY_RANGE;

			return IIO_VAL_FRACTIONAL;
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_OFFSET:
		if (chan->type == IIO_PHASE) {
			/* Time Delay Range 0 */
			if (st->is_td_range0[chan->scan_index])
				*val = 0;
			/* Time Delay Range 1 */
			else
				*val = ADAR400X_TD1_MASK;

			return IIO_VAL_INT;
		}
		else {
			return -EINVAL;
		}
	default:
		return -EINVAL;
	}
}

static int adar400x_write_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int val, int val2, long mask)
{
	struct adar400x_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->type) {
		case IIO_POWER:
		case IIO_PHASE:
			if (val & ADAR400X_TD1_MASK)
				st->is_td_range0[chan->scan_index] = false;
			else
				st->is_td_range0[chan->scan_index] = true;

			return adar400x_write_beamstate_elem(indio_dev, chan, val);
		default:
			return -EINVAL;
		}
	default:
		return -EINVAL;
	}
}

static int adar400x_bm0_mode_show(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *chan)
{
	struct adar400x_state *st = iio_priv(indio_dev);
	int ret, mode;
	u32 val;

	mode = st->bm0_mode[st->active_dev_addr];

	ret = adar400x_set_page(st, ADAR400X_CONFIG_PAGE);
	if (ret)
		return ret;

	ret = adar400x_reg_read(st, ADAR400X_BEAMSTATE_MODE, &val);
	if (ret)
		return ret;

	/* when in RAM/FIFO mode need to return to its corresponding page */
	if (val == ADAR400X_MEMORY_CONTROL_MODE ||
	    val == ADAR400X_FIFO_CONTROL_MODE) {
		ret = adar400x_set_page(st, val == ADAR400X_MEMORY_CONTROL_MODE ?
				        ADAR400X_BEAM0_RAM_PAGE : ADAR400X_FIFO_LOAD_PAGE);
		if (ret)
			return ret;
	}

	return mode <= ADAR400X_INST_DIRECT_CONTROL_MODE ? val : mode;
}

static int adar400x_bm0_setup(struct adar400x_state *st, unsigned int mode)
{
	int ret;

	ret = adar400x_clear_mute_reset(st);
	if (ret)
		return ret;

	ret = adar400x_set_page(st, ADAR400X_CONFIG_PAGE);
	if (ret)
		return ret;

	return adar400x_reg_write(st, ADAR400X_BEAMSTATE_MODE, mode);
}

static int adar400x_bm0_mode_store(struct iio_dev *indio_dev,
				   const struct iio_chan_spec *chan,
				   unsigned int mode)
{
	struct adar400x_state *st = iio_priv(indio_dev);
	int ret;

	st->bm0_mode[st->active_dev_addr] = mode;

	switch (mode) {
	case ADAR400X_DIRECT_CONTROL_MODE:
	case ADAR400X_INST_DIRECT_CONTROL_MODE:
		return adar400x_bm0_setup(st, ADAR400X_DIRECT_CONTROL_MODE);
	case ADAR400X_MEMORY_CONTROL_MODE:
		ret = adar400x_bm0_setup(st, ADAR400X_MEMORY_CONTROL_MODE);
		if (ret)
			return ret;

		return adar400x_set_page(st, ADAR400X_BEAM0_RAM_PAGE);
	case ADAR400X_FIFO_CONTROL_MODE:
		ret = adar400x_bm0_setup(st, ADAR400X_FIFO_CONTROL_MODE);
		if (ret)
			return ret;

		return adar400x_set_page(st, ADAR400X_FIFO_LOAD_PAGE);
	case ADAR400X_MUTE_MODE:
		return adar400x_beam_mute_spi(st);
	case ADAR400X_RESET_MODE:
		return adar400x_beam_reset_spi(st);
	default:
		return -EINVAL;
	}
}

static const struct iio_enum adar400x_bm0_mode_enum = {
	.items = adar400x_bm0_modes_string,
	.num_items = ARRAY_SIZE(adar400x_bm0_modes_string),
	.get = adar400x_bm0_mode_show,
	.set = adar400x_bm0_mode_store,
};

static ssize_t adar400x_dev_addr_store(struct iio_dev *indio_dev,
				       uintptr_t private,
				       const struct iio_chan_spec *chan,
				       const char *buf, size_t count)
{
	struct adar400x_state *st = iio_priv(indio_dev);
    	int ret;
	u8 val;

	guard(mutex)(&st->lock);
	ret = kstrtou8(buf, 10, &val);
	if (ret)
		return ret;

	if (val > st->num_devices - 1)
		return -EINVAL;

	st->active_dev_addr = val;

	return ret ? ret : count;
}

static ssize_t adar400x_dev_addr_show(struct iio_dev *indio_dev,
				      uintptr_t private,
				      const struct iio_chan_spec *chan,
				      char *buf)
{
	struct adar400x_state *st = iio_priv(indio_dev);

	guard(mutex)(&st->lock);
	return sysfs_emit(buf, "%d\n", st->active_dev_addr);
}

static ssize_t adar400x_beam_cmd_store(struct iio_dev *indio_dev,
				       uintptr_t private,
				       const struct iio_chan_spec *chan,
				       const char *buf, size_t count)
{
	struct adar400x_state *st = iio_priv(indio_dev);
    	int ret;
	bool val;

	ret = kstrtobool(buf, &val);
	if (ret)
		return ret;

	ret = adar400x_beam_update_spi(st, val);

	return ret ? ret : count;
}

static ssize_t adar400x_beam_cmd_show(struct iio_dev *indio_dev,
				      uintptr_t private,
				      const struct iio_chan_spec *chan,
				      char *buf)
{
	return -EOPNOTSUPP;
}

static int adar400x_pointer_access(struct adar400x_state *st, u16 reg,
				   u32 ptr_write, u32 *ptr_read)
{
	if (ptr_read)
		return regmap_read(st->regmap, st->active_dev_addr | reg, ptr_read);

	return regmap_write(st->regmap, st->active_dev_addr | reg, ptr_write);
}

static ssize_t adar400x_pointer_index_show(struct iio_dev *indio_dev,
					   uintptr_t private,
					   const struct iio_chan_spec *chan,
					   char *buf)
{
	struct adar400x_state *st = iio_priv(indio_dev);

	return sysfs_emit(buf, "%d\n", st->ptr_index);
}

static ssize_t adar400x_pointer_index_store(struct iio_dev *indio_dev,
					uintptr_t private,
					const struct iio_chan_spec *chan,
					const char *buf, size_t count)
{
	struct adar400x_state *st = iio_priv(indio_dev);
    	int ret;
	u8 val;

	guard(mutex)(&st->lock);
	ret = kstrtou8(buf, 10, &val);
	if (ret)
		return ret;

	if (val > ADAR400X_MAX_PTR_INDEX)
		return -EINVAL;

	st->ptr_index = val;

	return ret ? ret : count;
}

static ssize_t adar400x_pointer_show(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     char *buf)
{
	struct adar400x_state *st = iio_priv(indio_dev);
	u32 val;
	int ret;

	guard(mutex)(&st->lock);
	ret = adar400x_set_page(st, ADAR400X_CONFIG_PAGE);
	if (ret)
		return ret;

	if (private == ADAR400X_BM0_SEQ_PTR(0))
		ret = adar400x_pointer_access(st, ADAR400X_BM0_SEQ_PTR(st->ptr_index), 0, &val);
	else
		ret = adar400x_pointer_access(st, private, 0, &val);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%d\n", val);
}

static ssize_t adar400x_pointer_store(struct iio_dev *indio_dev,
				      uintptr_t private,
				      const struct iio_chan_spec *chan,
				      const char *buf, size_t count)
{
	struct adar400x_state *st = iio_priv(indio_dev);
    	int ret;
	u8 val;

	guard(mutex)(&st->lock);
	ret = kstrtou8(buf, 10, &val);
	if (ret)
		return ret;

	if (val > ADAR400X_8BIT_MAX_DATA)
		return -EINVAL;

	ret = adar400x_set_page(st, ADAR400X_CONFIG_PAGE);
	if (ret)
		return ret;

	if (private == ADAR400X_BM0_SEQ_PTR(0))
		ret = adar400x_pointer_access(st, ADAR400X_BM0_SEQ_PTR(st->ptr_index), val, NULL);
	else
		ret = adar400x_pointer_access(st, private, val, NULL);

	return ret ? ret : count;
}

static int adar400x_pack_data(struct adar400x_state *st, u8 *packed, u8 index,
			      u8 element, u8 delay, u8 attn)
{
	int ret, i;

	/* FIFO only utilized 0x100 to 0x106 load register addresses, index set to 0 */
	if (st->bm0_mode[st->active_dev_addr] == ADAR400X_FIFO_CONTROL_MODE)
		index = 0;

	switch (element) {
	case ADAR400X_EL0:
		packed[0] = delay;
		packed[1] = FIELD_PREP(ADAR400X_RAM_EL0_ATTN_MSK, attn);
		break;
	case ADAR400X_EL1:
		packed[1] |= FIELD_PREP(ADAR400X_RAM_EL1_DELAY_H_MSK, delay >> 6);
		packed[2] = FIELD_PREP(ADAR400X_RAM_EL1_DELAY_L_MSK, delay & GENMASK(5, 0));
		packed[2] |= FIELD_PREP(ADAR400X_RAM_EL1_ATTN_H_MSK, attn >> 4);
		packed[3] = FIELD_PREP(ADAR400X_RAM_EL1_ATTN_L_MSK, attn & GENMASK(3, 0));
		break;
	case ADAR400X_EL2:
		packed[3] |= FIELD_PREP(ADAR400X_RAM_EL2_DELAY_H_MSK, delay >> 4);
		packed[4] = FIELD_PREP(ADAR400X_RAM_EL2_DELAY_L_MSK, delay & GENMASK(3, 0));
		packed[4] |= FIELD_PREP(ADAR400X_RAM_EL2_ATTN_H_MSK, attn >> 2);
		packed[5] = FIELD_PREP(ADAR400X_RAM_EL2_ATTN_L_MSK, attn & GENMASK(1, 0));
		break;
	case ADAR400X_EL3:
		packed[5] |= FIELD_PREP(ADAR400X_RAM_EL3_DELAY_H_MSK, delay >> 2);
		packed[6] = FIELD_PREP(ADAR400X_RAM_EL3_DELAY_L_MSK, delay & GENMASK(1, 0));
		packed[6] |= FIELD_PREP(ADAR400X_RAM_EL3_ATTN_MSK, attn);
		break;
	default:
		return -EINVAL;
	}

	if (element == ADAR400X_EL3) {
		for (i = 0; i < ADAR400x_RAM_BEAMSTATE_PER_INDEX; i++) {
			ret = adar400x_reg_write(st, ADAR400X_BEAMSTATE_MEMORY(index) + i,
						 packed[i]);
			if (ret)
				return ret;
		}

		for (int i = 0; i < ADAR400x_RAM_BEAMSTATE_PER_INDEX; i++)
			packed[i] = 0;
	}

	return 0;
}

static ssize_t adar400x_ram_fifo_write(struct iio_dev *indio_dev,
				       uintptr_t private,
				       const struct iio_chan_spec *chan,
				       const char *buf, size_t count)
{
	struct adar400x_state *st = iio_priv(indio_dev);
	char line[ADAR400X_MAX_LINE_SIZE];
	u8 packed[ADAR400x_RAM_BEAMSTATE_PER_INDEX];
	u8 element, delay, attn, index;
	const char *ptr = buf;
	bool header_found;
	bool footer_found;
	int i, ret;

	guard(mutex)(&st->lock);
	for (i = 0; i < ARRAY_SIZE(packed); i++)
		packed[i] = 0;

	if (st->bm0_mode[st->active_dev_addr] != ADAR400X_FIFO_CONTROL_MODE &&
	    st->bm0_mode[st->active_dev_addr] != ADAR400X_MEMORY_CONTROL_MODE) {
		dev_err(&st->spi->dev, "Device not in FIFO or Memory Control Mode\n");
		return -EINVAL;
	}

	header_found = false;
	footer_found = false;

	ret = adar400x_set_page(st, private);
	if (ret)
		return ret;

	while (ptr < buf + count) {
		ret = sscanf(ptr, "%256[^\n]\n", line);
		if (ret != 1) {
			ptr++;
			continue;
		}

		if (strstr(line, "<index") != NULL) {
			header_found = true;
			ptr += strlen(line) + 1;
			continue;
		}

		if (strstr(line, "</beamstate>") != NULL) {
			footer_found = true;
			break;
		}

		ret = sscanf(line, "%hhu, %hhu, %hhu, %hhu", &index, &element, &delay, &attn);
		if (ret != 4) {
			ptr += strlen(line) + 1;
			continue;
		}

		if (element > ADAR400X_ELEMENT_PER_BEAM - 1 ||
		    delay > ADAR400X_MAX_DELAY_RAW ||
		    attn > ADAR400X_MAX_ATTENUATION_RAW)
			return -EINVAL;

		ret = adar400x_pack_data(st, packed, index, element, delay, attn);
		if (ret)
			return ret;

	        ptr += strlen(line) + 1;
	}

	return count;
}

static ssize_t adar400x_amp_en_show(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan,
				    char *buf)
{
	struct adar400x_state *st = iio_priv(indio_dev);
	int ret;
	u32 val;

	guard(mutex)(&st->lock);
	ret = adar400x_set_page(st, ADAR400X_CONFIG_PAGE);
	if (ret)
		return ret;

	ret = adar400x_reg_read(st, private, &val);
	if (ret)
		return ret;

	val = FIELD_GET(ADAR400X_EL_AMP_EN_MASK, val);

	return sysfs_emit(buf, "%d\n", val);
}

static ssize_t adar400x_amp_en_store(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     const char *buf, size_t count)
{
	struct adar400x_state *st = iio_priv(indio_dev);
    	int ret;
	bool val;

	guard(mutex)(&st->lock);
	ret = kstrtobool(buf, &val);
	if (ret)
		return ret;

	ret = adar400x_set_page(st, ADAR400X_CONFIG_PAGE);
	if (ret)
		return ret;

	ret = adar400x_reg_update_bits(st, private,
				       ADAR400X_EL_AMP_EN_MASK,
				       FIELD_PREP(ADAR400X_EL_AMP_EN_MASK, val));

	return ret ? ret : count;
}

static ssize_t adar400x_amp_bias_show(struct iio_dev *indio_dev,
				      uintptr_t private,
				      const struct iio_chan_spec *chan,
				      char *buf)
{
	struct adar400x_state *st = iio_priv(indio_dev);
	int ret;
	u32 val;

	guard(mutex)(&st->lock);
	ret = adar400x_set_page(st, ADAR400X_CONFIG_PAGE);
	if (ret)
		return ret;

	ret = adar400x_reg_read(st, private, &val);
	if (ret)
		return ret;

	val = FIELD_GET(ADAR400X_EL_AMP_BIAS_MASK, val);

	return sysfs_emit(buf, "%d\n", val);
}

static ssize_t adar400x_amp_bias_store(struct iio_dev *indio_dev,
				       uintptr_t private,
				       const struct iio_chan_spec *chan,
				       const char *buf, size_t count)
{
	struct adar400x_state *st = iio_priv(indio_dev);
    	int ret;
	u8 val;

	guard(mutex)(&st->lock);
	ret = kstrtou8(buf, 10, &val);
	if (ret)
		return ret;

	if (val > ADAR400X_MAX_AMP_BIAS)
		return -EINVAL;

	ret = adar400x_set_page(st, ADAR400X_CONFIG_PAGE);
	if (ret)
		return ret;

	ret = adar400x_reg_update_bits(st, private,
				       ADAR400X_EL_AMP_BIAS_MASK, val);

	return ret ? ret : count;
}

static int adar400x_reg_access(struct iio_dev *indio_dev,
			       u32 reg, u32 writeval,
			       u32 *readval)
{
	struct adar400x_state *st = iio_priv(indio_dev);

	if (readval)
                return adar400x_reg_read(st, reg, readval);

        return adar400x_reg_write(st, reg, writeval);
}

#define ADAR400X_ATTR(_name, _what, _shared, _read, _write) { 	\
	.name = _name,						\
	.read = _read,						\
	.write = _write,					\
	.shared = _shared,					\
	.private = _what,					\
}

static const struct iio_chan_spec_ext_info adar4000_ext_info[] = {
	ADAR400X_ATTR("active_device", 0, IIO_SHARED_BY_ALL,
		      adar400x_dev_addr_show, adar400x_dev_addr_store),
	ADAR400X_ATTR("beam0_update", 0, IIO_SHARED_BY_ALL, adar400x_beam_cmd_show,
		      adar400x_beam_cmd_store),
	IIO_ENUM("beam0_mode", IIO_SHARED_BY_ALL, &adar400x_bm0_mode_enum),
	IIO_ENUM_AVAILABLE("beam0_mode", IIO_SHARED_BY_ALL,
			   &adar400x_bm0_mode_enum),
	ADAR400X_ATTR("beam0_ram_start", ADAR400X_BM0_MEM_SEQ_PTR_START,
		      IIO_SHARED_BY_ALL, adar400x_pointer_show, adar400x_pointer_store),
	ADAR400X_ATTR("beam0_ram_stop", ADAR400X_BM0_MEM_SEQ_PTR_STOP,
		      IIO_SHARED_BY_ALL, adar400x_pointer_show, adar400x_pointer_store),
	ADAR400X_ATTR("beam0_fifo_wr", ADAR400X_BM0_FIFO_WRITE_PTR,
		      IIO_SHARED_BY_ALL, adar400x_pointer_show, NULL),
	ADAR400X_ATTR("beam0_fifo_rd", ADAR400X_BM0_FIFO_READ_PTR,
		      IIO_SHARED_BY_ALL, adar400x_pointer_show, NULL),
	ADAR400X_ATTR("beam0_sequencer_ptr_index", 0, IIO_SHARED_BY_ALL,
		      adar400x_pointer_index_show, adar400x_pointer_index_store),
	ADAR400X_ATTR("beamstate_ram", ADAR400X_BEAM0_RAM_PAGE, IIO_SHARED_BY_ALL,
		      NULL, adar400x_ram_fifo_write),
	ADAR400X_ATTR("beamstate_fifo", ADAR400X_FIFO_LOAD_PAGE,
		      IIO_SHARED_BY_ALL, NULL, adar400x_ram_fifo_write),

	ADAR400X_ATTR("amp_bias_reset_EL0A", ADAR400X_ELX_A_RESET_AMP(0),
		      IIO_SHARED_BY_ALL, adar400x_amp_bias_show, adar400x_amp_bias_store),
	ADAR400X_ATTR("amp_bias_reset_EL1A", ADAR400X_ELX_A_RESET_AMP(1),
		      IIO_SHARED_BY_ALL, adar400x_amp_bias_show, adar400x_amp_bias_store),
	ADAR400X_ATTR("amp_bias_reset_EL2A", ADAR400X_ELX_A_RESET_AMP(2),
		      IIO_SHARED_BY_ALL, adar400x_amp_bias_show, adar400x_amp_bias_store),
	ADAR400X_ATTR("amp_bias_reset_EL3A", ADAR400X_ELX_A_RESET_AMP(3),
		      IIO_SHARED_BY_ALL, adar400x_amp_bias_show, adar400x_amp_bias_store),
	ADAR400X_ATTR("amp_bias_reset_EL0B", ADAR400X_ELX_B_RESET_AMP(0),
		      IIO_SHARED_BY_ALL, adar400x_amp_bias_show, adar400x_amp_bias_store),
	ADAR400X_ATTR("amp_bias_reset_EL1B", ADAR400X_ELX_B_RESET_AMP(1),
		      IIO_SHARED_BY_ALL, adar400x_amp_bias_show, adar400x_amp_bias_store),
	ADAR400X_ATTR("amp_bias_reset_EL2B", ADAR400X_ELX_B_RESET_AMP(2),
		      IIO_SHARED_BY_ALL, adar400x_amp_bias_show, adar400x_amp_bias_store),
	ADAR400X_ATTR("amp_bias_reset_EL3B", ADAR400X_ELX_B_RESET_AMP(3),
		      IIO_SHARED_BY_ALL, adar400x_amp_bias_show, adar400x_amp_bias_store),
	ADAR400X_ATTR("amp_bias_operational_EL0A", ADAR400X_ELX_A_OP_AMP(0),
		      IIO_SHARED_BY_ALL, adar400x_amp_bias_show, adar400x_amp_bias_store),
	ADAR400X_ATTR("amp_bias_operational_EL1A", ADAR400X_ELX_A_OP_AMP(1),
		      IIO_SHARED_BY_ALL, adar400x_amp_bias_show, adar400x_amp_bias_store),
	ADAR400X_ATTR("amp_bias_operational_EL2A", ADAR400X_ELX_A_OP_AMP(2),
		      IIO_SHARED_BY_ALL, adar400x_amp_bias_show, adar400x_amp_bias_store),
	ADAR400X_ATTR("amp_bias_operational_EL3A", ADAR400X_ELX_A_OP_AMP(3),
		      IIO_SHARED_BY_ALL, adar400x_amp_bias_show, adar400x_amp_bias_store),
	ADAR400X_ATTR("amp_bias_operational_EL0B", ADAR400X_ELX_B_OP_AMP(0),
		      IIO_SHARED_BY_ALL, adar400x_amp_bias_show, adar400x_amp_bias_store),
	ADAR400X_ATTR("amp_bias_operational_EL1B", ADAR400X_ELX_B_OP_AMP(1),
		      IIO_SHARED_BY_ALL, adar400x_amp_bias_show, adar400x_amp_bias_store),
	ADAR400X_ATTR("amp_bias_operational_EL2B", ADAR400X_ELX_B_OP_AMP(2),
		      IIO_SHARED_BY_ALL, adar400x_amp_bias_show, adar400x_amp_bias_store),
	ADAR400X_ATTR("amp_bias_operational_EL3B", ADAR400X_ELX_B_OP_AMP(3),
		      IIO_SHARED_BY_ALL, adar400x_amp_bias_show, adar400x_amp_bias_store),
	ADAR400X_ATTR("amp_bias_mute_EL0A", ADAR400X_ELX_A_MUTE_AMP(0),
		      IIO_SHARED_BY_ALL, adar400x_amp_bias_show, adar400x_amp_bias_store),
	ADAR400X_ATTR("amp_bias_mute_EL1A", ADAR400X_ELX_A_MUTE_AMP(1),
		      IIO_SHARED_BY_ALL, adar400x_amp_bias_show, adar400x_amp_bias_store),
	ADAR400X_ATTR("amp_bias_mute_EL2A", ADAR400X_ELX_A_MUTE_AMP(2),
		      IIO_SHARED_BY_ALL, adar400x_amp_bias_show, adar400x_amp_bias_store),
	ADAR400X_ATTR("amp_bias_mute_EL3A", ADAR400X_ELX_A_MUTE_AMP(3),
		      IIO_SHARED_BY_ALL, adar400x_amp_bias_show, adar400x_amp_bias_store),
	ADAR400X_ATTR("amp_bias_mute_EL0B", ADAR400X_ELX_B_MUTE_AMP(0),
		      IIO_SHARED_BY_ALL, adar400x_amp_bias_show, adar400x_amp_bias_store),
	ADAR400X_ATTR("amp_bias_mute_EL1B", ADAR400X_ELX_B_MUTE_AMP(1),
		      IIO_SHARED_BY_ALL, adar400x_amp_bias_show, adar400x_amp_bias_store),
	ADAR400X_ATTR("amp_bias_mute_EL2B", ADAR400X_ELX_B_MUTE_AMP(2),
		      IIO_SHARED_BY_ALL, adar400x_amp_bias_show, adar400x_amp_bias_store),
	ADAR400X_ATTR("amp_bias_mute_EL3B", ADAR400X_ELX_B_MUTE_AMP(3),
		      IIO_SHARED_BY_ALL, adar400x_amp_bias_show, adar400x_amp_bias_store),
	ADAR400X_ATTR("amp_bias_sleep_EL0A", ADAR400X_ELX_A_SLEEP_AMP(0),
		      IIO_SHARED_BY_ALL, adar400x_amp_bias_show, adar400x_amp_bias_store),
	ADAR400X_ATTR("amp_bias_sleep_EL1A", ADAR400X_ELX_A_SLEEP_AMP(1),
		      IIO_SHARED_BY_ALL, adar400x_amp_bias_show, adar400x_amp_bias_store),
	ADAR400X_ATTR("amp_bias_sleep_EL2A", ADAR400X_ELX_A_SLEEP_AMP(2),
		      IIO_SHARED_BY_ALL, adar400x_amp_bias_show, adar400x_amp_bias_store),
	ADAR400X_ATTR("amp_bias_sleep_EL3A", ADAR400X_ELX_A_SLEEP_AMP(3),
		      IIO_SHARED_BY_ALL, adar400x_amp_bias_show, adar400x_amp_bias_store),
	ADAR400X_ATTR("amp_bias_sleep_EL0B", ADAR400X_ELX_B_SLEEP_AMP(0),
		      IIO_SHARED_BY_ALL, adar400x_amp_bias_show, adar400x_amp_bias_store),
	ADAR400X_ATTR("amp_bias_sleep_EL1B", ADAR400X_ELX_B_SLEEP_AMP(1),
		      IIO_SHARED_BY_ALL, adar400x_amp_bias_show, adar400x_amp_bias_store),
	ADAR400X_ATTR("amp_bias_sleep_EL2B", ADAR400X_ELX_B_SLEEP_AMP(2),
		      IIO_SHARED_BY_ALL, adar400x_amp_bias_show, adar400x_amp_bias_store),
	ADAR400X_ATTR("amp_bias_sleep_EL3B", ADAR400X_ELX_B_SLEEP_AMP(3),
		      IIO_SHARED_BY_ALL, adar400x_amp_bias_show, adar400x_amp_bias_store),

	ADAR400X_ATTR("amp_en_reset_EL0", ADAR400X_ELX_A_RESET_AMP(0),
		      IIO_SHARED_BY_ALL, adar400x_amp_en_show, adar400x_amp_en_store),
	ADAR400X_ATTR("amp_en_reset_EL1", ADAR400X_ELX_A_RESET_AMP(1),
		      IIO_SHARED_BY_ALL, adar400x_amp_en_show, adar400x_amp_en_store),
	ADAR400X_ATTR("amp_en_reset_EL2", ADAR400X_ELX_A_RESET_AMP(2),
		      IIO_SHARED_BY_ALL, adar400x_amp_en_show, adar400x_amp_en_store),
	ADAR400X_ATTR("amp_en_reset_EL3", ADAR400X_ELX_A_RESET_AMP(3),
		      IIO_SHARED_BY_ALL, adar400x_amp_en_show, adar400x_amp_en_store),
	ADAR400X_ATTR("amp_en_operational_EL0", ADAR400X_ELX_A_OP_AMP(0),
		      IIO_SHARED_BY_ALL, adar400x_amp_en_show, adar400x_amp_en_store),
	ADAR400X_ATTR("amp_en_operational_EL1", ADAR400X_ELX_A_OP_AMP(1),
		      IIO_SHARED_BY_ALL, adar400x_amp_en_show, adar400x_amp_en_store),
	ADAR400X_ATTR("amp_en_operational_EL2", ADAR400X_ELX_A_OP_AMP(2),
		      IIO_SHARED_BY_ALL, adar400x_amp_en_show, adar400x_amp_en_store),
	ADAR400X_ATTR("amp_en_operational_EL3", ADAR400X_ELX_A_OP_AMP(3),
		      IIO_SHARED_BY_ALL, adar400x_amp_en_show, adar400x_amp_en_store),
	ADAR400X_ATTR("amp_en_mute_EL0", ADAR400X_ELX_A_MUTE_AMP(0),
		      IIO_SHARED_BY_ALL, adar400x_amp_en_show, adar400x_amp_en_store),
	ADAR400X_ATTR("amp_en_mute_EL1", ADAR400X_ELX_A_MUTE_AMP(1),
		      IIO_SHARED_BY_ALL, adar400x_amp_en_show, adar400x_amp_en_store),
	ADAR400X_ATTR("amp_en_mute_EL2", ADAR400X_ELX_A_MUTE_AMP(2),
		      IIO_SHARED_BY_ALL, adar400x_amp_en_show, adar400x_amp_en_store),
	ADAR400X_ATTR("amp_en_mute_EL3", ADAR400X_ELX_A_MUTE_AMP(3),
		      IIO_SHARED_BY_ALL, adar400x_amp_en_show, adar400x_amp_en_store),
	ADAR400X_ATTR("amp_en_sleep_EL0", ADAR400X_ELX_A_SLEEP_AMP(0),
		      IIO_SHARED_BY_ALL, adar400x_amp_en_show, adar400x_amp_en_store),
	ADAR400X_ATTR("amp_en_sleep_EL1", ADAR400X_ELX_A_SLEEP_AMP(1),
		      IIO_SHARED_BY_ALL, adar400x_amp_en_show, adar400x_amp_en_store),
	ADAR400X_ATTR("amp_en_sleep_EL2", ADAR400X_ELX_A_SLEEP_AMP(2),
		      IIO_SHARED_BY_ALL, adar400x_amp_en_show, adar400x_amp_en_store),
	ADAR400X_ATTR("amp_en_sleep_EL3", ADAR400X_ELX_A_SLEEP_AMP(3),
		      IIO_SHARED_BY_ALL, adar400x_amp_en_show, adar400x_amp_en_store),
	{ },
};

#define ADAR400X_DELAY_CH(_id, _num)				\
{								\
	.type = IIO_PHASE,					\
	.output = 1,						\
	.indexed = 1,						\
	.channel = (_num),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
			      BIT(IIO_CHAN_INFO_SCALE),		\
	.ext_info = adar4000_ext_info,				\
}

#define ADAR400X_ATTEN_CH(_id, _num)				\
{								\
	.type = IIO_POWER,					\
	.indexed = 1,						\
	.output = 1,						\
	.channel = (_num),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
			      BIT(IIO_CHAN_INFO_SCALE),		\
	.ext_info = adar4000_ext_info,				\
}

static const struct iio_chan_spec adar4000_channels[] = {
	ADAR400X_DELAY_CH(0, 0),
	ADAR400X_ATTEN_CH(1, 0),
	ADAR400X_DELAY_CH(2, 1),
	ADAR400X_ATTEN_CH(3, 1),
	ADAR400X_DELAY_CH(4, 2),
	ADAR400X_ATTEN_CH(5, 2),
	ADAR400X_DELAY_CH(6, 3),
	ADAR400X_ATTEN_CH(7, 3),
};

static const struct adar400x_chip_info adar4000_chip = {
	.name = "adar4000",
	.chip_id = ID_ADAR4000,
	.channels = adar4000_channels,
	.num_channels = ARRAY_SIZE(adar4000_channels),
	.ext_info = adar4000_ext_info,
};

static const struct adar400x_chip_info adar4001_chip = {
	.name = "adar4001",
	.chip_id = ID_ADAR4001,
	.channels = adar4000_channels,
	.num_channels = ARRAY_SIZE(adar4000_channels),
	.ext_info = adar4000_ext_info,
};

static int adar400x_setup(struct adar400x_state *st)
{
	struct device *dev = &st->spi->dev;
	const struct adar400x_chip_info *info = st->chip_info;
	int ret, i, j;
	u32 val, val2;
	u16 id;

	st->num_devices = 1;
	ret = device_property_read_u32(dev, "adi,num-devices", &st->num_devices);
	if (!ret) {
		if (st->num_devices < 1 || st->num_devices > ADAR400X_MAX_DEV)
			return dev_err_probe(dev, -EINVAL, "Invalid number of devices.\n");
	}

	for (i = 0; i < st->num_devices; i++) {
		st->active_dev_addr = i;

		/* 3 times soft reset as per datasheet */
		for (j = 0; j < 3; j++) {
			ret = adar400x_reg_write(st, ADAR400X_SPI_CONFIG, ADAR400X_SPI_SETUP);
			if (ret)
				return ret;
		}

		ret = adar400x_reg_write(st, ADAR400X_PIN_SPI_CTRL, BIT(0));
		if (ret)
			return ret;

		ret = adar400x_reg_read(st, ADAR400X_PROD_ID_H, &val);
		if (ret)
			return ret;

		ret = adar400x_reg_read(st, ADAR400X_PROD_ID_L, &val2);
		if (ret)
			return ret;

		id = (val << 8) | val2;

		if (id != st->chip_info->chip_id)
			return dev_err_probe(dev, -ENODEV, "Invalid device ID: 0x%04x\n", id);

		st->bm0_mode[i] = ADAR400X_DIRECT_CONTROL_MODE;
	}

	st->active_dev_addr = 0;

	/* duplicate the default channel configuration as it can change during config */
	st->channels = devm_kmemdup(dev, info->channels,
				    info->num_channels * sizeof(*info->channels),
				    GFP_KERNEL);
	if (!st->channels)
		return -ENOMEM;

	for (i = 0; i < info->num_channels; i++)
		st->channels[i].ext_info = info->ext_info;

	return 0;
}

static struct regmap_config adar400x_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.read_flag_mask = BIT(7),
};

static const struct iio_info adar400x_info = {
	.read_raw = adar400x_read_raw,
	.write_raw = adar400x_write_raw,
	.debugfs_reg_access = &adar400x_reg_access,
};

static int adar400x_probe(struct spi_device *spi)
{
	const struct adar400x_chip_info *info;
	struct device *dev = &spi->dev;
	struct adar400x_state *st;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;

	info = spi_get_device_match_data(spi);
	if (!info)
		return -ENODEV;

	st->chip_info = info;

	mutex_init(&st->lock);

	st->regmap = devm_regmap_init_spi(spi, &adar400x_regmap_config);
	if (IS_ERR(st->regmap))
		return PTR_ERR(st->regmap);

	ret = adar400x_setup(st);
	if (ret)
		return ret;

	indio_dev->info = &adar400x_info;
	indio_dev->name = info->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = st->channels;
	indio_dev->num_channels = info->num_channels;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct spi_device_id adar400x_id[] = {
	{ "adar4000", (kernel_ulong_t)&adar4000_chip },
	{ "adar4001", (kernel_ulong_t)&adar4001_chip },
	{ }
};
MODULE_DEVICE_TABLE(spi, adar400x_id);

static const struct of_device_id adar400x_of_match[] = {
	{ .compatible = "adi,adar4000", .data = &adar4000_chip },
	{ .compatible = "adi,adar4001", .data = &adar4001_chip },
	{ }
};
MODULE_DEVICE_TABLE(of, adar400x_of_match);

static struct spi_driver adar400x_driver = {
	.driver = {
		.name = "adar4000",
		.of_match_table = adar400x_of_match,
	},
	.probe = adar400x_probe,
	.id_table = adar400x_id,
};
module_spi_driver(adar400x_driver);

MODULE_AUTHOR("Kim Seer Paller <kimseer.paller@analog.com>");
MODULE_DESCRIPTION("ADAR400X Driver");
MODULE_LICENSE("GPL");
