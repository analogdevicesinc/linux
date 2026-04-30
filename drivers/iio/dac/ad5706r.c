// SPDX-License-Identifier: GPL-2.0-only
/*
 * AD5706R 16-bit Current Output Digital to Analog Converter
 *
 * Copyright 2026 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/module.h>
#include <linux/pwm.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>
#include <linux/units.h>

/* SPI Defines */
#define AD5706R_RD_MASK		BIT(15)
#define AD5706R_ADDR_PIN_MASK	GENMASK(14, 12)
#define AD5706R_ADDR_MASK	GENMASK(11, 0)
#define AD5706R_VAL_MASK	GENMASK(7, 0)

/* Registers and Masks */
#define AD5706R_MASK_RESET			(BIT(7) | BIT(0))
#define AD5706R_MASK_DEV_ADDR(x)		((x) & GENMASK(2, 0))
#define AD5706R_REG_INTERFACE_CONFIG_A		0x00
#define AD5706R_MASK_INTERFACE_CONFIG_A(x)	((x) & GENMASK(7, 0))
#define AD5706R_MASK_ADDR_ASCENSION		BIT(5)
#define AD5706R_REG_INTERFACE_CONFIG_B		0x01
#define AD5706R_MASK_INTERFACE_CONFIG_B(x)	((x) & GENMASK(7, 0))
#define AD5706R_MASK_SINGLE_INSTR		BIT(7)
#define AD5706R_REG_MULTI_DAC_SEL_CH		0x14
#define AD5706R_MASK_MULTI_DAC_SEL_CH(x)	((x) & GENMASK(3, 0))
#define AD5706R_REG_LDAC_SYNC_ASYNC		0x16
#define AD5706R_MASK_LDAC_SYNC_ASYNC(x)		((x) & GENMASK(3, 0))
#define AD5706R_REG_LDAC_HW_SW			0x18
#define AD5706R_MASK_LDAC_HW_SW(x)		((x) & GENMASK(3, 0))
#define AD5706R_REG_LDAC_EDGE_SEL_CH(x)		(0x1A + ((x) * 2))
#define AD5706R_MASK_LDAC_EDGE_SEL_CH(x)	((x) & GENMASK(1, 0))
#define AD5706R_REG_OUT_OPERATING_MODE		0x22
#define AD5706R_MASK_OUT_OPERATING_MODE(x)      ((x) & GENMASK(3, 0))
#define AD5706R_REG_OUT_SWITCH_EN		0x24
#define AD5706R_MASK_OUT_SWITCH_EN(x)		((x) & GENMASK(3, 0))
#define AD5706R_REG_SHDN_EN			0x26
#define AD5706R_MASK_SHDN_EN(x)			((x) & GENMASK(3, 0))
#define AD5706R_REG_OUT_RANGE_CH(x)		(0x28 + ((x) * 2))
#define AD5706R_MASK_OUT_RANGE_CH(x)		((x) & GENMASK(1, 0))
#define AD5706R_REG_FUNC_EN			0x30
#define AD5706R_MASK_FUNC_EN(x)			((x) & GENMASK(3, 0))
#define AD5706R_REG_FUNC_MODE_SEL_CH(x)		(0x32 + ((x) * 2))
#define AD5706R_MASK_FUNC_MODE_SEL_CH(x)	((x) & BIT(0))
#define AD5706R_REG_FUNC_DAC_INPUT_B_CH(x)	(0x3A + ((x) * 2))
#define AD5706R_MASK_FUNC_DAC_INPUT_B_CH(x)	((x) & GENMASK(15, 0))
#define AD5706R_REG_MUX_OUT_SEL			0x54
#define AD5706R_MASK_MUX_OUT_SEL(x)		((x) & (BIT(7) | GENMASK(4, 0)))
#define AD5706R_REG_MUX_OUT_CONTROL		0x56
#define AD5706R_MASK_MUX_OUT_CONTROL(x)		((x) & BIT(0))
#define AD5706R_REG_MULTI_DAC_SW_LDAC		0x5A
#define AD5706R_MASK_MULTI_DAC_SW_LDAC		BIT(0)
#define AD5706R_REG_MULTI_DAC_INPUT_A		0x5C
#define AD5706R_MASK_MULTI_DAC_INPUT_A(x)	((x) & GENMASK(15, 0))
#define AD5706R_REG_DAC_SW_LDAC			0x5E
#define AD5706R_MASK_DAC_SW_LDAC(x)		((x) & GENMASK(3, 0))
#define AD5706R_REG_DAC_INPUT_A_CH(x)		(0x60 + ((x) * 2))
#define AD5706R_MASK_DAC_INPUT_A_CH(x)		((x) & GENMASK(15, 0))
#define AD5706R_REG_DAC_DATA_READBACK_CH(x)	(0x68 + ((x) * 2))
#define AD5706R_MASK_DAC_DATA_READBACK_CH(x)	((x) & GENMASK(15, 0))
#define AD5706R_REG_BANDGAP_CONTROL		0x73
#define AD5706R_MASK_BANDGAP_CONTROL		BIT(0)

#define NUM_CHANNELS			4
#define SPI_MAX_SPEED_HZ		(100 * HZ_PER_MHZ)	/* 100 MHz */
#define SPI_MIN_SPEED_HZ		(3 * HZ_PER_MHZ)	/* 3 MHz */
#define SAMPLING_FREQUENCY_MIN_HZ	1
#define SAMPLING_FREQUENCY_MAX_HZ	10000000
#define AD5706R_DAC_RESOLUTION		16
#define AD5706R_DAC_MAX_CODE		BIT(AD5706R_DAC_RESOLUTION)  /* 65536 */
#define AD5706R_MULTIBYTE_REG_START	0x14
#define AD5706R_MULTIBYTE_REG_END	0x71
#define AD5706R_SINGLE_BYTE_LEN		1
#define AD5706R_DOUBLE_BYTE_LEN		2

enum set_clk_mode_values {
	CLK_MODE_UNKNOWN = 0,
	CLK_MODE_CLKGEN = 1,
	CLK_MODE_SPI_ENGINE = 2,
};

/*
 * Order of attributes in code:
 *
 * Device Attributes:
 *   - dev_addr
 *   - addr_ascension
 *   - single_instr
 *   - hw_ldac_tg_state
 *   - sampling_frequency
 *   - hw_ldac_tg_pwm
 *   - mux_out_sel
 *   - multi_dac_input_a
 *   - multi_dac_sw_ldac_trigger
 *   - reference_volts
 *   - ref_select
 *   - hw_shutdown_state
 *
 * Channel Attributes:
 *   - raw
 *   - scale
 *   - offset
 *   - input_register_a
 *   - input_register_b
 *   - hw_active_edge
 *   - range_sel
 *   - output_state
 *   - ldac_trigger_chn
 *   - toggle_trigger_chn
 *   - dither_trigger_chn
 *   - multi_dac_sel_ch
 */

struct ad5706r_state {
	struct spi_device *spi;
	/* Mutex lock for clock transitions and device access */
	struct mutex lock;

	__be32 tx_buf __aligned(ARCH_DMA_MINALIGN);
	__be16 rx_buf;

	struct clk *reference_clk;
	struct pwm_device *ldacb_pwm;
	struct gpio_desc *resetb_gpio;
	struct gpio_desc *shdn_gpio;

	/* Debugfs Attributes */
	u64 debug_streaming_len;
	u64 debug_streaming_data;
	u16 debug_streaming_addr;
	u32 debug_spi_speed_hz_write;
	u32 debug_spi_speed_hz_read;

	/*
	 * Sets SPI Frequency via Clock Generator or SPI Engine
	 * 0 = SPI frequency changed, recompute clock mode
	 * 1 = Frequency set via clock generator
	 * 2 = SPI Engine controls speed
	 */
	u8 set_clk_mode;
	u32 spi_max_speed_hz;

	/* Device Attributes */
	unsigned int dev_addr;
	unsigned int addr_ascension;
	unsigned int single_instr;
	unsigned int shift_val;
	unsigned int addr_desc;
	unsigned int hw_ldac_tg_state;
	unsigned int sampling_frequency;
	unsigned int hw_ldac_tg_pwm;
	unsigned int mux_out_sel;
	unsigned int multi_dac_input_a;
	bool multi_dac_sw_ldac_trigger;
	unsigned int reference_volts;
	unsigned int ref_select;
	unsigned int hw_shutdown_state;

	/* Channel Attributes */
	unsigned int hw_active_edge[NUM_CHANNELS];
	unsigned int range_sel[NUM_CHANNELS];
	unsigned int output_state[NUM_CHANNELS];
	unsigned int ldac_trigger_chn[NUM_CHANNELS];
	unsigned int toggle_trigger_chn[NUM_CHANNELS];
	unsigned int dither_trigger_chn[NUM_CHANNELS];
	unsigned int multi_dac_sel_ch[NUM_CHANNELS];
};

/* ENUM Lists */
enum addr_ascension_iio_dev_attr {
	ADDR_ASCENSION_DECREMENT = 0,
	ADDR_ASCENSION_INCREMENT,
};

static const char * const addr_ascension_iio_dev_attr_vals[] = {
	[ADDR_ASCENSION_DECREMENT] = "decrement",
	[ADDR_ASCENSION_INCREMENT] = "increment",
};

enum single_instr_iio_dev_attr {
	SINGLE_INSTR_STREAMING = 0,
	SINGLE_INSTR_SINGLE_INSTRUCTION,
};

static const char * const single_instr_iio_dev_attr_vals[] = {
	[SINGLE_INSTR_STREAMING] = "streaming",
	[SINGLE_INSTR_SINGLE_INSTRUCTION] = "single_instruction",
};

enum hw_ldac_tg_state_iio_dev_attr {
	HW_LDAC_TG_STATE_LOW = 0,
	HW_LDAC_TG_STATE_HIGH
};

static const char * const hw_ldac_tg_state_iio_dev_attr_vals[] = {
	[HW_LDAC_TG_STATE_LOW] = "low",
	[HW_LDAC_TG_STATE_HIGH] = "high",
};

enum hw_ldac_tg_pwm_iio_dev_attr {
	HW_LDAC_TG_PWM_DISABLED,
	HW_LDAC_TG_PWM_ENABLED,
};

static const char * const hw_ldac_tg_pwm_iio_dev_attr_vals[] = {
	[HW_LDAC_TG_PWM_DISABLED] = "disable",
	[HW_LDAC_TG_PWM_ENABLED] = "enable",
};

enum mux_out_sel_iio_dev_attr {
	MUX_OUT_SEL_DISABLED = 0,	/* Index 0 */
	MUX_OUT_SEL_AGND,		/* Index 1 */
	MUX_OUT_SEL_AVDD,		/* Index 2 */
	MUX_OUT_SEL_VREF,		/* Index 3 */
	MUX_OUT_SEL_IOUT0_VMON,		/* Index 4 */
	MUX_OUT_SEL_IOUT1_VMON,		/* Index 5 */
	MUX_OUT_SEL_IOUT2_VMON,		/* Index 6 */
	MUX_OUT_SEL_IOUT3_VMON,		/* Index 7 */
	MUX_OUT_SEL_IOUT0_IMON,		/* Index 8 */
	MUX_OUT_SEL_IOUT1_IMON,		/* Index 9 */
	MUX_OUT_SEL_IOUT2_IMON,		/* Index 10 */
	MUX_OUT_SEL_IOUT3_IMON,		/* Index 11 */
	MUX_OUT_SEL_PVDD0,		/* Index 12 */
	MUX_OUT_SEL_PVDD1,		/* Index 13 */
	MUX_OUT_SEL_PVDD2,		/* Index 14 */
	MUX_OUT_SEL_PVDD3,		/* Index 15 */
	MUX_OUT_SEL_TEMP_SENSOR0,	/* Index 16 */
	MUX_OUT_SEL_TEMP_SENSOR1,	/* Index 17 */
	MUX_OUT_SEL_TEMP_SENSOR2,	/* Index 18 */
	MUX_OUT_SEL_TEMP_SENSOR3,	/* Index 19 */
	MUX_OUT_SEL_MUX_IN0,		/* Index 20 */
	MUX_OUT_SEL_MUX_IN1,		/* Index 21 */
	MUX_OUT_SEL_MUX_IN2,		/* Index 22 */
	MUX_OUT_SEL_MUX_IN3,		/* Index 23 */
};

static const char * const mux_out_sel_iio_dev_attr_vals[] = {
	[MUX_OUT_SEL_DISABLED]		= "disabled",	/* Index 0 */
	[MUX_OUT_SEL_AGND]		= "agnd",	/* Index 1 */
	[MUX_OUT_SEL_AVDD]		= "avdd",	/* Index 2 */
	[MUX_OUT_SEL_VREF]		= "vref",	/* Index 3 */
	[MUX_OUT_SEL_IOUT0_VMON]	= "iout0_vmon",	/* Index 4 */
	[MUX_OUT_SEL_IOUT1_VMON]	= "iout1_vmon",	/* Index 5 */
	[MUX_OUT_SEL_IOUT2_VMON]	= "iout2_vmon",	/* Index 6 */
	[MUX_OUT_SEL_IOUT3_VMON]	= "iout3_vmon",	/* Index 7 */
	[MUX_OUT_SEL_IOUT0_IMON]	= "iout0_imon",	/* Index 8 */
	[MUX_OUT_SEL_IOUT1_IMON]	= "iout1_imon",	/* Index 9 */
	[MUX_OUT_SEL_IOUT2_IMON]	= "iout2_imon",	/* Index 10 */
	[MUX_OUT_SEL_IOUT3_IMON]	= "iout3_imon",	/* Index 11 */
	[MUX_OUT_SEL_PVDD0]		= "pvdd0",	/* Index 12 */
	[MUX_OUT_SEL_PVDD1]		= "pvdd1",	/* Index 13 */
	[MUX_OUT_SEL_PVDD2]		= "pvdd2",	/* Index 14 */
	[MUX_OUT_SEL_PVDD3]		= "pvdd3",	/* Index 15 */
	[MUX_OUT_SEL_TEMP_SENSOR0]	= "tdiode_ch0",	/* Index 16 */
	[MUX_OUT_SEL_TEMP_SENSOR1]	= "tdiode_ch1",	/* Index 17 */
	[MUX_OUT_SEL_TEMP_SENSOR2]	= "tdiode_ch2",	/* Index 18 */
	[MUX_OUT_SEL_TEMP_SENSOR3]	= "tdiode_ch3",	/* Index 19 */
	[MUX_OUT_SEL_MUX_IN0]		= "mux_in0",	/* Index 20 */
	[MUX_OUT_SEL_MUX_IN1]		= "mux_in1",	/* Index 21 */
	[MUX_OUT_SEL_MUX_IN2]		= "mux_in2",	/* Index 22 */
	[MUX_OUT_SEL_MUX_IN3]		= "mux_in3",	/* Index 23 */
};

static const u8 mux_out_sel_reg_values[] = {
	[MUX_OUT_SEL_DISABLED]		= 0x00,
	[MUX_OUT_SEL_AGND]		= 0x80,
	[MUX_OUT_SEL_AVDD]		= 0x81,
	[MUX_OUT_SEL_VREF]		= 0x82,
	[MUX_OUT_SEL_IOUT0_VMON]	= 0x84,
	[MUX_OUT_SEL_IOUT1_VMON]	= 0x85,
	[MUX_OUT_SEL_IOUT2_VMON]	= 0x86,
	[MUX_OUT_SEL_IOUT3_VMON]	= 0x87,
	[MUX_OUT_SEL_IOUT0_IMON]	= 0x88,
	[MUX_OUT_SEL_IOUT1_IMON]	= 0x89,
	[MUX_OUT_SEL_IOUT2_IMON]	= 0x8A,
	[MUX_OUT_SEL_IOUT3_IMON]	= 0x8B,
	[MUX_OUT_SEL_PVDD0]		= 0x8C,
	[MUX_OUT_SEL_PVDD1]		= 0x8D,
	[MUX_OUT_SEL_PVDD2]		= 0x8E,
	[MUX_OUT_SEL_PVDD3]		= 0x8F,
	[MUX_OUT_SEL_TEMP_SENSOR0]	= 0x90,
	[MUX_OUT_SEL_TEMP_SENSOR1]	= 0x91,
	[MUX_OUT_SEL_TEMP_SENSOR2]	= 0x92,
	[MUX_OUT_SEL_TEMP_SENSOR3]	= 0x93,
	[MUX_OUT_SEL_MUX_IN0]		= 0x94,
	[MUX_OUT_SEL_MUX_IN1]		= 0x95,
	[MUX_OUT_SEL_MUX_IN2]		= 0x96,
	[MUX_OUT_SEL_MUX_IN3]		= 0x97,
};

enum multi_dac_sw_ldac_trigger_iio_dev_attr {
	MULTI_DAC_SW_LDAC_TRIGGER_LOW = 0,
	MULTI_DAC_SW_LDAC_TRIGGER_TRIGGER,
};

static const char * const multi_dac_sw_ldac_trigger_iio_dev_attr_vals[] = {
	[MULTI_DAC_SW_LDAC_TRIGGER_LOW] = "low",
	[MULTI_DAC_SW_LDAC_TRIGGER_TRIGGER] = "trigger",
};

enum ref_select_iio_dev_attr {
	REF_SELECT_EXTERNAL,
	REF_SELECT_INTERNAL,
};

static const char * const ref_select_iio_dev_attr_vals[] = {
	[REF_SELECT_EXTERNAL] = "external",
	[REF_SELECT_INTERNAL] = "internal",
};

enum hw_shutdown_state_iio_dev_attr {
	HW_SHUTDOWN_STATE_LOW,
	HW_SHUTDOWN_STATE_HIGH,
};

static const char * const hw_shutdown_state_iio_dev_attr_vals[] = {
	[HW_SHUTDOWN_STATE_LOW] = "low",
	[HW_SHUTDOWN_STATE_HIGH] = "high",
};

enum hw_active_edge_iio_dev_attr {
	HW_ACTIVE_EDGE_RISING_EDGE = 0,
	HW_ACTIVE_EDGE_FALLING_EDGE,
	HW_ACTIVE_EDGE_ANY_EDGE,
};

static const char * const hw_active_edge_iio_dev_attr_vals[] = {
	[HW_ACTIVE_EDGE_RISING_EDGE] = "rising_edge",
	[HW_ACTIVE_EDGE_FALLING_EDGE] = "falling_edge",
	[HW_ACTIVE_EDGE_ANY_EDGE] = "any_edge",
};

enum range_sel_iio_dev_attr {
	RANGE_SEL_50 = 0,
	RANGE_SEL_150 = 1,
	RANGE_SEL_200 = 2,
	RANGE_SEL_300 = 3,
};

static const char * const range_sel_iio_dev_attr_vals[] = {
	[RANGE_SEL_50] = "50mA",
	[RANGE_SEL_150] = "150mA",
	[RANGE_SEL_200] = "200mA",
	[RANGE_SEL_300] = "300mA",
};

enum output_state_iio_dev_attr {
	OUTPUT_STATE_NORMAL_SW = 0,
	OUTPUT_STATE_SHUTDOWN_TO_TRISTATE_SW,
	OUTPUT_STATE_SHUTDOWN_TO_GND_SW,
	OUTPUT_STATE_NORMAL_HW,
	OUTPUT_STATE_SHUTDOWN_TO_TRISTATE_HW,
	OUTPUT_STATE_SHUTDOWN_TO_GND_HW,
};

static const char * const output_state_iio_dev_attr_vals[] = {
	[OUTPUT_STATE_NORMAL_SW] = "normal_sw",
	[OUTPUT_STATE_SHUTDOWN_TO_TRISTATE_SW] = "shutdown_to_tristate_sw",
	[OUTPUT_STATE_SHUTDOWN_TO_GND_SW] = "shutdown_to_gnd_sw",
	[OUTPUT_STATE_NORMAL_HW] = "normal_hw",
	[OUTPUT_STATE_SHUTDOWN_TO_TRISTATE_HW] = "shutdown_to_tristate_hw",
	[OUTPUT_STATE_SHUTDOWN_TO_GND_HW] = "shutdown_to_gnd_hw",
};

enum ldac_trigger_chn_iio_dev_attr {
	LDAC_TRIGGER_CHN_NONE = 0,
	LDAC_TRIGGER_CHN_HW_TRIGGER,
	LDAC_TRIGGER_CHN_SW_TRIGGER,
};

static const char * const ldac_trigger_chn_iio_dev_attr_vals[] = {
	[LDAC_TRIGGER_CHN_NONE] = "None",
	[LDAC_TRIGGER_CHN_HW_TRIGGER] = "hw_ldac",
	[LDAC_TRIGGER_CHN_SW_TRIGGER] = "sw_ldac",
};

enum toggle_trigger_chn_iio_dev_attr {
	TOGGLE_TRIGGER_CHN_NONE = 0,
	TOGGLE_TRIGGER_CHN_HW_TRIGGER,
	TOGGLE_TRIGGER_CHN_SW_TRIGGER,
};

static const char * const toggle_trigger_chn_iio_dev_attr_vals[] = {
	[TOGGLE_TRIGGER_CHN_NONE] = "None",
	[TOGGLE_TRIGGER_CHN_HW_TRIGGER] = "hw_toggle",
	[TOGGLE_TRIGGER_CHN_SW_TRIGGER] = "sw_toggle",
};

enum dither_trigger_chn_iio_dev_attr {
	DITHER_TRIGGER_CHN_NONE = 0,
	DITHER_TRIGGER_CHN_HW_TRIGGER,
	DITHER_TRIGGER_CHN_SW_TRIGGER,
};

static const char * const dither_trigger_chn_iio_dev_attr_vals[] = {
	[DITHER_TRIGGER_CHN_NONE] = "None",
	[DITHER_TRIGGER_CHN_HW_TRIGGER] = "hw_dither",
	[DITHER_TRIGGER_CHN_SW_TRIGGER] = "sw_dither",
};

enum multi_dac_sel_ch_iio_chan_attr {
	MULTI_DAC_SEL_CH_EXCLUDE = 0,
	MULTI_DAC_SEL_CH_INCLUDE = 1,
};

static const char * const multi_dac_sel_ch_iio_chan_attr_vals[] = {
	[MULTI_DAC_SEL_CH_EXCLUDE] = "exclude",
	[MULTI_DAC_SEL_CH_INCLUDE] = "include",
};

static int _ad5706r_set_clk_rate(struct ad5706r_state *st, u32 rate, u8 wr)
{
	int ret, current_rate;

	rate = clamp(rate, (u32)SPI_MIN_SPEED_HZ, (u32)SPI_MAX_SPEED_HZ);
	/* If frequency is already set, do nothing */
	current_rate = DIV_ROUND_CLOSEST(clk_get_rate(st->reference_clk), 2);
	if (wr && current_rate == rate) {
		st->debug_spi_speed_hz_write = current_rate;
		return 0;
	}
	if (!wr && current_rate == rate) {
		st->debug_spi_speed_hz_read = current_rate;
		return 0;
	}

	/* Disable the clock before setting the rate */
	clk_disable_unprepare(st->reference_clk);

	/* spi engine spi clock runs at half the SPI reference clock */
	ret = clk_set_rate(st->reference_clk, rate * 2);
	if (ret)
		return ret;

	/* Re-enable the clock after setting the rate */
	ret = clk_prepare_enable(st->reference_clk);
	if (ret)
		return ret;

	/* If frequency is not a good number, save the closest possible */
	current_rate = DIV_ROUND_CLOSEST(clk_get_rate(st->reference_clk), 2);
	if (wr)
		st->debug_spi_speed_hz_write = current_rate;
	else
		st->debug_spi_speed_hz_read = current_rate;

	/* Wait for clock to stabilize */
	usleep_range(3000, 3100);

	return 0;
}

/*
 * Check if a frequency can be achieved using SPI Engine integer divider.
 * SPI Engine hardware requires:
 * - Even dividers only (hardware limitation)
 * - Divider > 1 (divider of 1 means running at max frequency)
 * - Exact frequency match after division (no rounding errors)
 */
static bool _can_use_spi_engine_divider(u32 max_freq, u32 target_freq)
{
	u32 divider = DIV_ROUND_CLOSEST(max_freq, target_freq);
	u32 achieved_freq = DIV_ROUND_CLOSEST(max_freq, divider);

	if (achieved_freq != target_freq)
		return false;  /* Rounding error, need exact frequency */

	if (divider <= 1)
		return false;  /* Already at max frequency */

	if (divider & 1)
		return false;  /* SPI Engine requires even dividers */

	return true;
}

/*
 * CLKGEN mode contains a 3ms delay after changing the clock frequency, this
 * delay is avoided if frequencies can be set by SPI Engine.
 */
static int _ad5706r_compute_spi_clk(struct ad5706r_state *st, int *write_hz,
				    int *read_hz)
{
	int ret;

	/* Check frequencies if SPI Engine can generate them */
	if (_can_use_spi_engine_divider(st->spi_max_speed_hz, st->debug_spi_speed_hz_write) &&
	    _can_use_spi_engine_divider(st->spi_max_speed_hz, st->debug_spi_speed_hz_read)) {
		/* Disable the clock before setting the rate */
		clk_disable_unprepare(st->reference_clk);

		/* spi engine spi clock runs at half the SPI reference clock */
		ret = clk_set_rate(st->reference_clk, st->spi_max_speed_hz * 2);
		if (ret)
			return ret;

		/* Re-enable the clock after setting the rate */
		ret = clk_prepare_enable(st->reference_clk);
		if (ret)
			return ret;

		*write_hz = st->debug_spi_speed_hz_write;
		*read_hz = st->debug_spi_speed_hz_read;
		st->set_clk_mode = CLK_MODE_SPI_ENGINE;

		return 0;
	}

	/* Otherwise, set to Clock Generator Mode */
	*write_hz = 0;
	*read_hz = 0;
	st->set_clk_mode = CLK_MODE_CLKGEN;

	return 0;
}

static int ad5706r_reg_len(unsigned int reg)
{
	if (reg >= AD5706R_MULTIBYTE_REG_START && reg <= AD5706R_MULTIBYTE_REG_END)
		return AD5706R_DOUBLE_BYTE_LEN;

	return AD5706R_SINGLE_BYTE_LEN;
}

static int ad5706r_spi_write(struct ad5706r_state *st, u16 reg, u16 val)
{
	unsigned int num_bytes;
	int write_hz, read_hz;
	int ret;

	num_bytes = ad5706r_reg_len(reg);

	if (reg >= AD5706R_MULTIBYTE_REG_START && reg <= AD5706R_MULTIBYTE_REG_END)
		reg = reg + st->addr_desc;

	if (st->set_clk_mode == CLK_MODE_UNKNOWN) {
		ret = _ad5706r_compute_spi_clk(st, &write_hz, &read_hz);
		if (ret)
			return ret;
	}

	if (st->set_clk_mode == CLK_MODE_CLKGEN) {
		write_hz = 0;
		ret = _ad5706r_set_clk_rate(st, st->debug_spi_speed_hz_write, 1);
		if (ret)
			return ret;
	} else {
		write_hz = st->debug_spi_speed_hz_write;
	}

	struct spi_transfer xfer = {
		.tx_buf = &st->tx_buf,
		.len = num_bytes + 2,
		.speed_hz = write_hz,
	};

	st->tx_buf = cpu_to_be32(((((st->dev_addr << 12) & AD5706R_ADDR_PIN_MASK) |
				   reg) << 16) | (val << (16 - (num_bytes * 8))));

	ret = spi_sync_transfer(st->spi, &xfer, 1);
	if (ret)
		return ret;

	return 0;
}

static int ad5706r_spi_read(struct ad5706r_state *st, u16 reg, u16 *val)
{
	unsigned int num_bytes;
	u16 cmd;
	int write_hz, read_hz;
	int ret;

	num_bytes = ad5706r_reg_len(reg);

	if (reg >= AD5706R_MULTIBYTE_REG_START && reg <= AD5706R_MULTIBYTE_REG_END)
		reg = reg + st->addr_desc;

	if (st->set_clk_mode == CLK_MODE_UNKNOWN) {
		ret = _ad5706r_compute_spi_clk(st, &write_hz, &read_hz);
		if (ret)
			return ret;
	}

	if (st->set_clk_mode == CLK_MODE_CLKGEN) {
		read_hz = 0;
		ret = _ad5706r_set_clk_rate(st, st->debug_spi_speed_hz_read, 0);
		if (ret)
			return ret;
	} else {
		read_hz = st->debug_spi_speed_hz_read;
	}

	struct spi_transfer xfer[] = {
		{
			.tx_buf = &st->tx_buf,
			.rx_buf = NULL,
			.len    = 2,
			.speed_hz = read_hz,
		},
		{
			.tx_buf = NULL,
			.rx_buf = &st->rx_buf,
			.len    = num_bytes,
			.speed_hz = read_hz,
		},
	};

	cmd = AD5706R_RD_MASK |
	      ((st->dev_addr << 12) & AD5706R_ADDR_PIN_MASK) |
	      (reg & AD5706R_ADDR_MASK);

	/* Convert to 32-bit big-endian (shift to upper 16 bits) */
	st->tx_buf = cpu_to_be32((u32)cmd << 16);

	ret = spi_sync_transfer(st->spi, xfer, ARRAY_SIZE(xfer));
	if (ret)
		return ret;

	*val = be16_to_cpu(st->rx_buf) >> (16 - (num_bytes * 8));

	return 0;
}

/* debugfs Register Access */

static int ad5706r_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			      unsigned int writeval, unsigned int *readval)
{
	struct ad5706r_state *st = iio_priv(indio_dev);

	if (reg >= AD5706R_MULTIBYTE_REG_START && reg <= AD5706R_MULTIBYTE_REG_END)
		reg = reg - st->addr_desc;

	if (readval)
		return ad5706r_spi_read(st, reg, (u16 *)readval);

	return ad5706r_spi_write(st, reg, writeval);
}

static int ad5706r_set_streaming_addr(void *arg, u64 buf)
{
	struct iio_dev *indio_dev = arg;
	struct ad5706r_state *st = iio_priv(indio_dev);

	st->debug_streaming_addr = buf;

	return 0;
}

static int ad5706r_show_streaming_addr(void *arg, u64 *val)
{
	struct iio_dev *indio_dev = arg;
	struct ad5706r_state *st = iio_priv(indio_dev);

	*val = st->debug_streaming_addr;

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(ad5706r_streaming_addr_fops, ad5706r_show_streaming_addr,
			 ad5706r_set_streaming_addr, "%llu\n");

static int ad5706r_set_streaming_len(void *arg, u64 buf)
{
	struct iio_dev *indio_dev = arg;
	struct ad5706r_state *st = iio_priv(indio_dev);

	st->debug_streaming_len = buf;

	return 0;
}

static int ad5706r_show_streaming_len(void *arg, u64 *val)
{
	struct iio_dev *indio_dev = arg;
	struct ad5706r_state *st = iio_priv(indio_dev);

	*val = st->debug_streaming_len;

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(ad5706r_streaming_len_fops, ad5706r_show_streaming_len,
			 ad5706r_set_streaming_len, "%llu\n");

static int ad5706r_set_streaming_data(void *arg, u64 buf)
{
	struct iio_dev *indio_dev = arg;
	struct ad5706r_state *st = iio_priv(indio_dev);

	st->debug_streaming_data = buf;

	return 0;
}

static int ad5706r_show_streaming_data(void *arg, u64 *val)
{
	struct iio_dev *indio_dev = arg;
	struct ad5706r_state *st = iio_priv(indio_dev);

	*val = st->debug_streaming_data;

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(ad5706r_streaming_data_fops, ad5706r_show_streaming_data,
			 ad5706r_set_streaming_data, "%llu\n");

static int ad5706r_set_streaming_reg_access(void *arg, u64 buf)
{
	struct iio_dev *indio_dev = arg;
	struct ad5706r_state *st = iio_priv(indio_dev);
	u16 word;
	__be16 write_val[5];
	int write_hz, read_hz;
	int ret;
	int i;

	write_val[0] = cpu_to_be16(((st->dev_addr << 12) & AD5706R_ADDR_PIN_MASK) |
				 st->debug_streaming_addr);

	for (i = 0; i < 4; i++) {
		/* Extract as native u16 */
		word = (st->debug_streaming_data >> (i * 16)) & 0xFFFF;

		/* Byte swap in native endian */
		word = (word & 0x00FF) << 8 | (word & 0xFF00) >> 8;

		/* Convert to big-endian for transmission */
		write_val[i + 1] = cpu_to_be16(word);
		}

	if (st->set_clk_mode == CLK_MODE_UNKNOWN) {
		ret = _ad5706r_compute_spi_clk(st, &write_hz, &read_hz);
		if (ret)
			return ret;
	}

	if (st->set_clk_mode == CLK_MODE_CLKGEN) {
		write_hz = 0;
		ret = _ad5706r_set_clk_rate(st, st->debug_spi_speed_hz_write, 1);
		if (ret)
			return ret;
	} else {
		write_hz = st->debug_spi_speed_hz_write;
	}

	struct spi_transfer xfer = {
		.tx_buf = write_val,
		.len = st->debug_streaming_len + 2,
		.speed_hz = write_hz,
	};

	ret = spi_sync_transfer(st->spi, &xfer, 1);
	if (ret)
		return ret;

	return 0;
}

static int ad5706r_show_streaming_reg_access(void *arg, u64 *val)
{
	struct iio_dev *indio_dev = arg;
	struct ad5706r_state *st = iio_priv(indio_dev);
	u64 read_val;
	u16 cmd;
	int write_hz, read_hz;
	int ret;

	if (st->set_clk_mode == CLK_MODE_UNKNOWN) {
		ret = _ad5706r_compute_spi_clk(st, &write_hz, &read_hz);
		if (ret)
			return ret;
	}

	if (st->set_clk_mode == CLK_MODE_CLKGEN) {
		read_hz = 0;
		ret = _ad5706r_set_clk_rate(st, st->debug_spi_speed_hz_read, 0);
		if (ret)
			return ret;
	} else {
		read_hz = st->debug_spi_speed_hz_read;
	}

	struct spi_transfer xfer[] = {
		{
			.tx_buf = &st->tx_buf,
			.rx_buf = NULL,
			.len    = 2,
			.speed_hz = read_hz,
		},
		{
			.tx_buf = NULL,
			.rx_buf = &read_val,
			.len    = st->debug_streaming_len,
			.speed_hz = read_hz,
		},
	};

	cmd = AD5706R_RD_MASK |
	      ((st->dev_addr << 12) & AD5706R_ADDR_PIN_MASK) |
	      (st->debug_streaming_addr & AD5706R_ADDR_MASK);

	/* Convert to 32-bit big-endian (shift to upper 16 bits) */
	st->tx_buf = cpu_to_be32((u32)cmd << 16);

	ret = spi_sync_transfer(st->spi, xfer, ARRAY_SIZE(xfer));
	if (ret)
		return ret;

	*val = read_val;

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(ad5706r_streaming_reg_access_fops, ad5706r_show_streaming_reg_access,
			 ad5706r_set_streaming_reg_access, "%llu\n");

static int ad5706r_set_spi_speed_write(void *arg, u64 buf)
{
	struct iio_dev *indio_dev = arg;
	struct ad5706r_state *st = iio_priv(indio_dev);
	int ret;

	st->set_clk_mode = CLK_MODE_UNKNOWN;
	ret = _ad5706r_set_clk_rate(st, buf, 1);
	if (ret)
		return ret;

	return 0;
}

static int ad5706r_show_spi_speed_write(void *arg, u64 *val)
{
	struct iio_dev *indio_dev = arg;
	struct ad5706r_state *st = iio_priv(indio_dev);

	*val = st->debug_spi_speed_hz_write;

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(ad5706r_spi_speed_write_fops, ad5706r_show_spi_speed_write,
			 ad5706r_set_spi_speed_write, "%llu\n");

static int ad5706r_set_spi_speed_read(void *arg, u64 buf)
{
	struct iio_dev *indio_dev = arg;
	struct ad5706r_state *st = iio_priv(indio_dev);
	int ret;

	st->set_clk_mode = CLK_MODE_UNKNOWN;
	ret = _ad5706r_set_clk_rate(st, buf, 0);
	if (ret)
		return ret;

	return 0;
}

static int ad5706r_show_spi_speed_read(void *arg, u64 *val)
{
	struct iio_dev *indio_dev = arg;
	struct ad5706r_state *st = iio_priv(indio_dev);

	*val = st->debug_spi_speed_hz_read;

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(ad5706r_spi_speed_read_fops, ad5706r_show_spi_speed_read,
			 ad5706r_set_spi_speed_read, "%llu\n");

static void ad5706r_debugs_init(struct iio_dev *indio_dev)
{
	struct dentry *d = iio_get_debugfs_dentry(indio_dev);

	debugfs_create_file_unsafe("streaming_addr", 0600, d,
				   indio_dev, &ad5706r_streaming_addr_fops);
	debugfs_create_file_unsafe("streaming_len", 0600, d,
				   indio_dev, &ad5706r_streaming_len_fops);
	debugfs_create_file_unsafe("streaming_data", 0600, d,
				   indio_dev, &ad5706r_streaming_data_fops);
	debugfs_create_file_unsafe("streaming_reg_access", 0600, d,
				   indio_dev, &ad5706r_streaming_reg_access_fops);
	debugfs_create_file_unsafe("spi_speed_hz_write", 0600, d,
				   indio_dev, &ad5706r_spi_speed_write_fops);
	debugfs_create_file_unsafe("spi_speed_hz_read", 0600, d,
				   indio_dev, &ad5706r_spi_speed_read_fops);
}

/* Attributes */

static int _set_reg_channel_mode(struct ad5706r_state *st, int chan_n,
				 bool bool_func_en, bool bool_func_mode,
				 bool bool_sync_async, bool bool_hw_sw)
{
	u16 reg_val;
	u16 reg_val2;
	u16 reg_val3;
	u16 mask_val;
	int ret;

	ret = ad5706r_spi_read(st, AD5706R_REG_FUNC_EN, &reg_val);
	if (ret)
		return ret;
	ret = ad5706r_spi_read(st, AD5706R_REG_LDAC_SYNC_ASYNC, &reg_val2);
	if (ret)
		return ret;
	ret = ad5706r_spi_read(st, AD5706R_REG_LDAC_HW_SW, &reg_val3);
	if (ret)
		return ret;

	mask_val = BIT(chan_n + st->shift_val);
	reg_val = ~mask_val & reg_val;
	reg_val2 = ~mask_val & reg_val2;
	reg_val3 = ~mask_val & reg_val3;

	usleep_range(1, 2); /* Delay for device stability after mode change. */

	/* Write 0 FUNC_EN - device unlock */
	ret = ad5706r_spi_write(st, AD5706R_REG_FUNC_EN, reg_val);
	if (ret)
		return ret;

	/* Write 1/0 to AD5706R_REG_FUNC_MODE_SEL_CH() */
	if (bool_func_mode)
		ret = ad5706r_spi_write(st, AD5706R_REG_FUNC_MODE_SEL_CH(chan_n),
					BIT(st->shift_val));
	else
		ret = ad5706r_spi_write(st, AD5706R_REG_FUNC_MODE_SEL_CH(chan_n),
					0 << st->shift_val);
	if (ret)
		return ret;

	/* Write 1 to FUNC_EN */
	if (bool_func_en)
		ret = ad5706r_spi_write(st, AD5706R_REG_FUNC_EN,
					reg_val | mask_val);
	if (ret)
		return ret;

	/* Write 1/0 to LDAC_SYNC_ASYNC */
	if (bool_sync_async)
		ret = ad5706r_spi_write(st, AD5706R_REG_LDAC_SYNC_ASYNC,
					reg_val2 | mask_val);
	else
		ret = ad5706r_spi_write(st, AD5706R_REG_LDAC_SYNC_ASYNC,
					reg_val2);
	if (ret)
		return ret;

	/* Write 1/0 to LDAC_HW_SW for HW */
	if (bool_hw_sw)
		ret = ad5706r_spi_write(st, AD5706R_REG_LDAC_HW_SW,
					reg_val3 | mask_val);
	else
		ret = ad5706r_spi_write(st, AD5706R_REG_LDAC_HW_SW,
					reg_val3);
	if (ret)
		return ret;

	return 0;
}

static int _set_pwm_duty_cycle(struct ad5706r_state *st, int duty_cycle)
{
	struct pwm_state ldacb_pwm_state;
	int ret;

	pwm_get_state(st->ldacb_pwm, &ldacb_pwm_state);

	ldacb_pwm_state.duty_cycle = duty_cycle == 0 ? 0 :
		DIV_ROUND_CLOSEST_ULL(NANO, st->sampling_frequency * 100 / duty_cycle);

	ret = pwm_apply_might_sleep(st->ldacb_pwm, &ldacb_pwm_state);
	if (ret)
		return ret;

	return 0;
}

/* Device Attributes */
static ssize_t ad5706r_dev_addr_write(struct iio_dev *indio_dev,
				      uintptr_t private, const struct iio_chan_spec *chan,
				      const char *buf, size_t len)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	unsigned int reg_val;
	int ret;

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return ret;

	st->dev_addr = AD5706R_MASK_DEV_ADDR(reg_val);

	return ret ? ret : len;
}

static ssize_t ad5706r_dev_addr_read(struct iio_dev *indio_dev,
				     uintptr_t private, const struct iio_chan_spec *chan, char *buf)
{
	struct ad5706r_state *st = iio_priv(indio_dev);

	return sysfs_emit(buf, "%u\n", st->dev_addr);
}

static int ad5706r_addr_ascension_write(struct iio_dev *indio_dev,
					const struct iio_chan_spec *chan, unsigned int item)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	u16 reg_val;
	int ret;

	ret = ad5706r_spi_read(st, AD5706R_REG_INTERFACE_CONFIG_A, &reg_val);
	if (ret)
		return ret;

	reg_val = (~AD5706R_MASK_ADDR_ASCENSION) & reg_val;
	reg_val = AD5706R_MASK_INTERFACE_CONFIG_A(reg_val);

	if (item == ADDR_ASCENSION_DECREMENT) {
		ret = ad5706r_spi_write(st, AD5706R_REG_INTERFACE_CONFIG_A,
					reg_val);
		st->shift_val = 0;
		st->addr_desc = 1;
	} else {
		ret = ad5706r_spi_write(st, AD5706R_REG_INTERFACE_CONFIG_A,
					reg_val | AD5706R_MASK_ADDR_ASCENSION);
		st->shift_val = 8;
		st->addr_desc = 0;
	}
	if (ret)
		return ret;

	st->addr_ascension = item;

	return 0;
}

static int ad5706r_addr_ascension_read(struct iio_dev *indio_dev,
				       const struct iio_chan_spec *chan)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	u16 reg_val;
	int ret;

	ret = ad5706r_spi_read(st, AD5706R_REG_INTERFACE_CONFIG_A, &reg_val);
	if (ret)
		return ret;

	reg_val = (reg_val & AD5706R_MASK_ADDR_ASCENSION) >> 5;
	st->addr_ascension = reg_val;

	if (st->addr_ascension)
		st->shift_val = 8;
	else
		st->shift_val = 0;

	return st->addr_ascension;
}

static const struct iio_enum ad5706r_addr_ascension_enum = {
	.items = addr_ascension_iio_dev_attr_vals,
	.num_items = ARRAY_SIZE(addr_ascension_iio_dev_attr_vals),
	.set = ad5706r_addr_ascension_write,
	.get = ad5706r_addr_ascension_read,
};

static int ad5706r_single_instr_write(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan, unsigned int item)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	u16 reg_val;
	int ret;

	ret = ad5706r_spi_read(st, AD5706R_REG_INTERFACE_CONFIG_B, &reg_val);
	if (ret)
		return ret;

	reg_val = (~AD5706R_MASK_SINGLE_INSTR) & reg_val;
	reg_val = AD5706R_MASK_INTERFACE_CONFIG_B(reg_val);

	if (item == SINGLE_INSTR_STREAMING)
		ret = ad5706r_spi_write(st, AD5706R_REG_INTERFACE_CONFIG_B,
					reg_val);
	else
		ret = ad5706r_spi_write(st, AD5706R_REG_INTERFACE_CONFIG_B,
					reg_val | AD5706R_MASK_SINGLE_INSTR);

	if (ret)
		return ret;

	st->single_instr = item;

	return 0;
}

static int ad5706r_single_instr_read(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	u16 reg_val;
	int ret;

	ret = ad5706r_spi_read(st, AD5706R_REG_INTERFACE_CONFIG_B, &reg_val);
	if (ret)
		return ret;

	reg_val = (reg_val & AD5706R_MASK_SINGLE_INSTR) >> 7;
	st->single_instr = reg_val;

	return st->single_instr;
}

static const struct iio_enum ad5706r_single_instr_enum = {
	.items = single_instr_iio_dev_attr_vals,
	.num_items = ARRAY_SIZE(single_instr_iio_dev_attr_vals),
	.set = ad5706r_single_instr_write,
	.get = ad5706r_single_instr_read,
};

static int ad5706r_hw_ldac_tg_state_write(struct iio_dev *indio_dev,
					  const struct iio_chan_spec *chan,
					  unsigned int item)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	int ret;

	if (item == HW_LDAC_TG_STATE_HIGH)
		ret = _set_pwm_duty_cycle(st, 100);
	else
		ret = _set_pwm_duty_cycle(st, 0);

	if (!ret)
		st->hw_ldac_tg_state = item;

	return ret;
}

static int ad5706r_hw_ldac_tg_state_read(struct iio_dev *indio_dev,
					 const struct iio_chan_spec *chan)
{
	struct ad5706r_state *st = iio_priv(indio_dev);

	return st->hw_ldac_tg_state;
}

static const struct iio_enum ad5706r_hw_ldac_tg_state_enum = {
	.items = hw_ldac_tg_state_iio_dev_attr_vals,
	.num_items = ARRAY_SIZE(hw_ldac_tg_state_iio_dev_attr_vals),
	.set = ad5706r_hw_ldac_tg_state_write,
	.get = ad5706r_hw_ldac_tg_state_read,
};

static int ad5706r_hw_ldac_tg_pwm_write(struct iio_dev *indio_dev,
					const struct iio_chan_spec *chan,
					unsigned int item)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	int ret;

	if (item == HW_LDAC_TG_PWM_DISABLED)
		ret = _set_pwm_duty_cycle(st, 0);
	else
		ret = _set_pwm_duty_cycle(st, 50);

	if (!ret)
		st->hw_ldac_tg_pwm = item;

	return ret;
}

static int ad5706r_hw_ldac_tg_pwm_read(struct iio_dev *indio_dev,
				       const struct iio_chan_spec *chan)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	struct pwm_state ldacb_pwm_state;

	pwm_get_state(st->ldacb_pwm, &ldacb_pwm_state);
	if (ldacb_pwm_state.duty_cycle == 0 ||
	    ldacb_pwm_state.duty_cycle == DIV_ROUND_CLOSEST_ULL(NANO,
					  st->sampling_frequency))
		st->hw_ldac_tg_pwm = HW_LDAC_TG_PWM_DISABLED;
	else
		st->hw_ldac_tg_pwm = HW_LDAC_TG_PWM_ENABLED;

	return st->hw_ldac_tg_pwm;
}

static const struct iio_enum ad5706r_hw_ldac_tg_pwm_enum = {
	.items = hw_ldac_tg_pwm_iio_dev_attr_vals,
	.num_items = ARRAY_SIZE(hw_ldac_tg_pwm_iio_dev_attr_vals),
	.set = ad5706r_hw_ldac_tg_pwm_write,
	.get = ad5706r_hw_ldac_tg_pwm_read,
};

static int ad5706r_mux_out_sel_write(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     unsigned int item)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	unsigned int reg_value;
	int ret;

	/* Validate index */
	if (item >= ARRAY_SIZE(mux_out_sel_reg_values))
		return -EINVAL;

	/* Convert index to register value */
	reg_value = mux_out_sel_reg_values[item];

	ret = ad5706r_spi_write(st, AD5706R_REG_MUX_OUT_SEL,
				reg_value << st->shift_val);
	if (ret)
		return ret;

	st->mux_out_sel = item;

	return 0;
}

static int ad5706r_mux_out_sel_read(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	u16 reg_val;
	u8 reg_byte;
	int ret;
	int i;

	ret = ad5706r_spi_read(st, AD5706R_REG_MUX_OUT_SEL, &reg_val);
	if (ret)
		return ret;

	/* Extract the 8-bit value */
	reg_byte = (reg_val >> st->shift_val) & 0xFF;

	/* Find which index has this register value */
	for (i = 0; i < ARRAY_SIZE(mux_out_sel_reg_values); i++) {
		if (mux_out_sel_reg_values[i] == reg_byte) {
			st->mux_out_sel = i;
			return i;  /* Return index, not register value */
		}
	}

	/* Unknown value - default to disabled */
	st->mux_out_sel = MUX_OUT_SEL_DISABLED;
	return MUX_OUT_SEL_DISABLED;
}

static const struct iio_enum ad5706r_mux_out_sel_enum = {
	.items = mux_out_sel_iio_dev_attr_vals,
	.num_items = ARRAY_SIZE(mux_out_sel_iio_dev_attr_vals),
	.set = ad5706r_mux_out_sel_write,
	.get = ad5706r_mux_out_sel_read,
};

static ssize_t ad5706r_multi_dac_input_a_write(struct iio_dev *indio_dev,
					       uintptr_t private, const struct iio_chan_spec *chan,
					       const char *buf, size_t len)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	unsigned int reg_val;
	int ret;

	ret = kstrtou32(buf, 16, &reg_val);
	if (ret)
		return ret;

	ret = ad5706r_spi_write(st, AD5706R_REG_MULTI_DAC_INPUT_A,
				AD5706R_MASK_MULTI_DAC_INPUT_A(reg_val));
	if (ret)
		return ret;

	return ret ? ret : len;
}

static ssize_t ad5706r_multi_dac_input_a_read(struct iio_dev *indio_dev,
					      uintptr_t private, const struct iio_chan_spec *chan,
					      char *buf)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	u16 reg_val;
	int ret;

	ret = ad5706r_spi_read(st, AD5706R_REG_MULTI_DAC_INPUT_A, &reg_val);
	if (ret)
		return ret;

	return sysfs_emit(buf, "0x%lx\n", AD5706R_MASK_MULTI_DAC_INPUT_A(reg_val));
}

static int ad5706r_multi_dac_sw_ldac_trigger_write(struct iio_dev *indio_dev,
						   const struct iio_chan_spec *chan,
						   unsigned int item)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	int ret;

	ret = ad5706r_spi_write(st, AD5706R_REG_MULTI_DAC_SW_LDAC, item << st->shift_val);
	if (ret)
		return ret;

	return 0;
}

static int ad5706r_multi_dac_sw_ldac_trigger_read(struct iio_dev *indio_dev,
						  const struct iio_chan_spec *chan)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	u16 reg_val;
	int ret;

	ret = ad5706r_spi_read(st, AD5706R_REG_MULTI_DAC_SW_LDAC, &reg_val);
	if (ret)
		return ret;

	return reg_val;
}

static const struct iio_enum ad5706r_multi_dac_sw_ldac_trigger_enum = {
	.items = multi_dac_sw_ldac_trigger_iio_dev_attr_vals,
	.num_items = ARRAY_SIZE(multi_dac_sw_ldac_trigger_iio_dev_attr_vals),
	.set = ad5706r_multi_dac_sw_ldac_trigger_write,
	.get = ad5706r_multi_dac_sw_ldac_trigger_read,
};

static ssize_t ad5706r_reference_volts_write(struct iio_dev *indio_dev,
					     uintptr_t private, const struct iio_chan_spec *chan,
					     const char *buf, size_t len)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	unsigned int reg_val;
	int ret;

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return ret;

	st->reference_volts = reg_val;

	return ret ? ret : len;
}

static ssize_t ad5706r_reference_volts_read(struct iio_dev *indio_dev,
					    uintptr_t private, const struct iio_chan_spec *chan,
					    char *buf)
{
	struct ad5706r_state *st = iio_priv(indio_dev);

	return sysfs_emit(buf, "%u\n", st->reference_volts);
}

static int ad5706r_ref_select_write(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    unsigned int item)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	int ret;

	ret = ad5706r_spi_write(st, AD5706R_REG_BANDGAP_CONTROL, item);
	if (ret)
		return ret;

	st->ref_select = item;

	return 0;
}

static int ad5706r_ref_select_read(struct iio_dev *indio_dev,
				   const struct iio_chan_spec *chan)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	u16 reg_val;
	int ret;

	ret = ad5706r_spi_read(st, AD5706R_REG_BANDGAP_CONTROL, &reg_val);
	if (ret)
		return ret;

	if (reg_val)
		st->ref_select = REF_SELECT_INTERNAL;
	else
		st->ref_select = REF_SELECT_EXTERNAL;

	return st->ref_select;
}

static const struct iio_enum ad5706r_ref_select_enum = {
	.items = ref_select_iio_dev_attr_vals,
	.num_items = ARRAY_SIZE(ref_select_iio_dev_attr_vals),
	.set = ad5706r_ref_select_write,
	.get = ad5706r_ref_select_read,
};

static int ad5706r_hw_shutdown_state_write(struct iio_dev *indio_dev,
					   const struct iio_chan_spec *chan,
					   unsigned int item)
{
	struct ad5706r_state *st = iio_priv(indio_dev);

	if (item == HW_SHUTDOWN_STATE_LOW)
		gpiod_set_value_cansleep(st->shdn_gpio, 0);
	else
		gpiod_set_value_cansleep(st->shdn_gpio, 1);

	st->hw_shutdown_state = item;

	return 0;
}

static int ad5706r_hw_shutdown_state_read(struct iio_dev *indio_dev,
					  const struct iio_chan_spec *chan)
{
	struct ad5706r_state *st = iio_priv(indio_dev);

	return st->hw_shutdown_state;
}

static const struct iio_enum ad5706r_hw_shutdown_state_enum = {
	.items = hw_shutdown_state_iio_dev_attr_vals,
	.num_items = ARRAY_SIZE(hw_shutdown_state_iio_dev_attr_vals),
	.set = ad5706r_hw_shutdown_state_write,
	.get = ad5706r_hw_shutdown_state_read,
};

/* Channel Attributes */
static ssize_t ad5706r_input_register_a_write(struct iio_dev *indio_dev,
					      uintptr_t private, const struct iio_chan_spec *chan,
					      const char *buf, size_t len)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	unsigned int reg_val;
	int ret;

	ret = kstrtou32(buf, 16, &reg_val);
	if (ret)
		return ret;

	ret = ad5706r_spi_write(st, AD5706R_REG_DAC_INPUT_A_CH(chan->channel),
				AD5706R_MASK_DAC_INPUT_A_CH(reg_val));
	if (ret)
		return ret;

	return ret ? ret : len;
}

static ssize_t ad5706r_input_register_a_read(struct iio_dev *indio_dev,
					     uintptr_t private, const struct iio_chan_spec *chan,
					     char *buf)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	u16 reg_val;
	int ret;

	ret = ad5706r_spi_read(st, AD5706R_REG_DAC_INPUT_A_CH(chan->channel), &reg_val);

	if (ret)
		return ret;

	return sysfs_emit(buf, "0x%lx\n", AD5706R_MASK_DAC_INPUT_A_CH(reg_val));
}

static ssize_t ad5706r_input_register_b_write(struct iio_dev *indio_dev,
					      uintptr_t private, const struct iio_chan_spec *chan,
					      const char *buf, size_t len)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	unsigned int reg_val;
	int ret;

	ret = kstrtou32(buf, 16, &reg_val);
	if (ret)
		return ret;

	ret = ad5706r_spi_write(st, AD5706R_REG_FUNC_DAC_INPUT_B_CH(chan->channel),
				AD5706R_MASK_FUNC_DAC_INPUT_B_CH(reg_val));

	if (ret)
		return ret;

	return ret ? ret : len;
}

static ssize_t ad5706r_input_register_b_read(struct iio_dev *indio_dev,
					     uintptr_t private, const struct iio_chan_spec *chan,
					     char *buf)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	u16 reg_val;
	int ret;

	ret = ad5706r_spi_read(st, AD5706R_REG_FUNC_DAC_INPUT_B_CH(chan->channel),
			       &reg_val);

	if (ret)
		return ret;

	return sysfs_emit(buf, "0x%lx\n", AD5706R_MASK_FUNC_DAC_INPUT_B_CH(reg_val));
}

static int ad5706r_hw_active_edge_write(struct iio_dev *indio_dev,
					const struct iio_chan_spec *chan, unsigned int item)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	int ret;

	ret = ad5706r_spi_write(st, AD5706R_REG_LDAC_EDGE_SEL_CH(chan->channel),
				AD5706R_MASK_LDAC_EDGE_SEL_CH(item) << st->shift_val);
	if (ret)
		return ret;

	st->hw_active_edge[chan->channel] = item;

	return 0;
}

static int ad5706r_hw_active_edge_read(struct iio_dev *indio_dev,
				       const struct iio_chan_spec *chan)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	u16 reg_val;
	int ret;

	ret = ad5706r_spi_read(st, AD5706R_REG_LDAC_EDGE_SEL_CH(chan->channel),
			       &reg_val);
	if (ret)
		return ret;

	st->hw_active_edge[chan->channel] = AD5706R_MASK_LDAC_EDGE_SEL_CH(reg_val >> st->shift_val);

	return st->hw_active_edge[chan->channel];
}

static const struct iio_enum ad5706r_hw_active_edge_enum = {
	.items = hw_active_edge_iio_dev_attr_vals,
	.num_items = ARRAY_SIZE(hw_active_edge_iio_dev_attr_vals),
	.set = ad5706r_hw_active_edge_write,
	.get = ad5706r_hw_active_edge_read,
};

static int ad5706r_range_sel_write(struct iio_dev *indio_dev,
				   const struct iio_chan_spec *chan, unsigned int item)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	int ret;

	ret = ad5706r_spi_write(st, AD5706R_REG_OUT_RANGE_CH(chan->channel),
				AD5706R_MASK_OUT_RANGE_CH(item) << st->shift_val);

	if (ret)
		return ret;

	st->range_sel[chan->channel] = item;

	return 0;
}

static int ad5706r_range_sel_read(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *chan)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	u16 reg_val;
	int ret;

	ret = ad5706r_spi_read(st, AD5706R_REG_OUT_RANGE_CH(chan->channel), &reg_val);
	if (ret)
		return ret;

	st->range_sel[chan->channel] = AD5706R_MASK_OUT_RANGE_CH(reg_val >> st->shift_val);

	return st->range_sel[chan->channel];
}

static const struct iio_enum ad5706r_range_sel_enum = {
	.items = range_sel_iio_dev_attr_vals,
	.num_items = ARRAY_SIZE(range_sel_iio_dev_attr_vals),
	.set = ad5706r_range_sel_write,
	.get = ad5706r_range_sel_read,
};

static int ad5706r_output_state_write(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan,
				      unsigned int item)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	u16 reg_val;
	u16 reg_val2;
	u16 reg_val3;
	u16 mask_val;
	int ret;

	ret = ad5706r_spi_read(st, AD5706R_REG_OUT_OPERATING_MODE, &reg_val);
	if (ret)
		return ret;

	ret = ad5706r_spi_read(st, AD5706R_REG_OUT_SWITCH_EN, &reg_val2);
	if (ret)
		return ret;

	ret = ad5706r_spi_read(st, AD5706R_REG_SHDN_EN, &reg_val3);
	if (ret)
		return ret;

	mask_val = 1 << (chan->channel + st->shift_val);
	reg_val = ~mask_val & reg_val;
	reg_val2 = ~mask_val & reg_val2;
	reg_val3 = ~mask_val & reg_val3;

	switch (item) {
	case OUTPUT_STATE_NORMAL_SW:
		ret = ad5706r_spi_write(st, AD5706R_REG_OUT_OPERATING_MODE,
					reg_val | mask_val);
		if (ret)
			return ret;
		ret = ad5706r_spi_write(st, AD5706R_REG_SHDN_EN, reg_val3);
		if (ret)
			return ret;
		break;
	case OUTPUT_STATE_SHUTDOWN_TO_TRISTATE_SW:
		ret = ad5706r_spi_write(st, AD5706R_REG_OUT_OPERATING_MODE,
					reg_val);
		if (ret)
			return ret;
		ret = ad5706r_spi_write(st, AD5706R_REG_OUT_SWITCH_EN,
					reg_val2);
		if (ret)
			return ret;
		ret = ad5706r_spi_write(st, AD5706R_REG_SHDN_EN, reg_val3);
		if (ret)
			return ret;
		break;
	case OUTPUT_STATE_SHUTDOWN_TO_GND_SW:
		ret = ad5706r_spi_write(st, AD5706R_REG_OUT_OPERATING_MODE,
					reg_val);
		if (ret)
			return ret;
		ret = ad5706r_spi_write(st, AD5706R_REG_OUT_SWITCH_EN,
					reg_val2 | mask_val);
		if (ret)
			return ret;
		ret = ad5706r_spi_write(st, AD5706R_REG_SHDN_EN, reg_val3);
		if (ret)
			return ret;
		break;
	case OUTPUT_STATE_NORMAL_HW:
		ret = ad5706r_spi_write(st, AD5706R_REG_OUT_OPERATING_MODE,
					reg_val | mask_val);
		if (ret)
			return ret;
		ret = ad5706r_spi_write(st, AD5706R_REG_SHDN_EN, reg_val3 | mask_val);
		if (ret)
			return ret;
		gpiod_set_value_cansleep(st->shdn_gpio, 1);
		break;
	case OUTPUT_STATE_SHUTDOWN_TO_TRISTATE_HW:
		ret = ad5706r_spi_write(st, AD5706R_REG_OUT_OPERATING_MODE,
					reg_val | mask_val);
		if (ret)
			return ret;
		ret = ad5706r_spi_write(st, AD5706R_REG_OUT_SWITCH_EN,
					reg_val2);
		if (ret)
			return ret;
		ret = ad5706r_spi_write(st, AD5706R_REG_SHDN_EN, reg_val3 | mask_val);
		if (ret)
			return ret;
		gpiod_set_value_cansleep(st->shdn_gpio, 0);
		break;
	case OUTPUT_STATE_SHUTDOWN_TO_GND_HW:
		ret = ad5706r_spi_write(st, AD5706R_REG_OUT_OPERATING_MODE,
					reg_val | mask_val);
		if (ret)
			return ret;
		ret = ad5706r_spi_write(st, AD5706R_REG_OUT_SWITCH_EN,
					reg_val2 | mask_val);
		if (ret)
			return ret;
		ret = ad5706r_spi_write(st, AD5706R_REG_SHDN_EN, reg_val3 | mask_val);
		if (ret)
			return ret;
		gpiod_set_value_cansleep(st->shdn_gpio, 0);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ad5706r_output_state_read(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	u16 reg_val;
	u16 reg_val2;
	u16 reg_val3;
	u16 gpio_val;
	u16 mask_val;
	int ret;

	ret = ad5706r_spi_read(st, AD5706R_REG_OUT_OPERATING_MODE, &reg_val);
	if (ret)
		return ret;

	ret = ad5706r_spi_read(st, AD5706R_REG_OUT_SWITCH_EN, &reg_val2);
	if (ret)
		return ret;

	ret = ad5706r_spi_read(st, AD5706R_REG_SHDN_EN, &reg_val3);
	if (ret)
		return ret;

	mask_val = 1 << (chan->channel + st->shift_val);
	reg_val = mask_val & reg_val;
	reg_val2 = mask_val & reg_val2;
	reg_val3 = mask_val & reg_val3;
	gpio_val = gpiod_get_value_cansleep(st->shdn_gpio);

	if (reg_val && !reg_val3)
		st->output_state[chan->channel] = OUTPUT_STATE_NORMAL_SW;
	else if (!reg_val && !reg_val2)
		st->output_state[chan->channel] = OUTPUT_STATE_SHUTDOWN_TO_TRISTATE_SW;
	else if (!reg_val && reg_val2)
		st->output_state[chan->channel] = OUTPUT_STATE_SHUTDOWN_TO_GND_SW;
	else if (reg_val && reg_val3 && gpio_val)
		st->output_state[chan->channel] = OUTPUT_STATE_NORMAL_HW;
	else if (reg_val && !reg_val2 && reg_val3 && !gpio_val)
		st->output_state[chan->channel] = OUTPUT_STATE_SHUTDOWN_TO_TRISTATE_HW;
	else if (reg_val && reg_val2 && reg_val3 && !gpio_val)
		st->output_state[chan->channel] = OUTPUT_STATE_SHUTDOWN_TO_GND_HW;

	return st->output_state[chan->channel];
}

static const struct iio_enum ad5706r_output_state_enum = {
	.items = output_state_iio_dev_attr_vals,
	.num_items = ARRAY_SIZE(output_state_iio_dev_attr_vals),
	.set = ad5706r_output_state_write,
	.get = ad5706r_output_state_read,
};

static int ad5706r_ldac_trigger_chn_write(struct iio_dev *indio_dev,
					  const struct iio_chan_spec *chan,
					  unsigned int item)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	bool val_func_en = 0;
	bool val_func_mode = 0;
	bool val_sync_async = 0;
	bool val_hw_sw = 0;
	int ret;

	guard(mutex)(&st->lock);

	st->ldac_trigger_chn[chan->channel] = LDAC_TRIGGER_CHN_NONE;
	st->toggle_trigger_chn[chan->channel] = TOGGLE_TRIGGER_CHN_NONE;
	st->dither_trigger_chn[chan->channel] = DITHER_TRIGGER_CHN_NONE;

	if (item != LDAC_TRIGGER_CHN_NONE)
		val_sync_async = 1;	/* Write 1 LDAC_SYNC_ASYNC */

	if (item == LDAC_TRIGGER_CHN_SW_TRIGGER)
		val_hw_sw = 1;		/* Write 1 LDAC_HW_SW for SW */

	ret = _set_reg_channel_mode(st, chan->channel, val_func_en, val_func_mode,
				    val_sync_async, val_hw_sw);
	if (ret)
		return ret;

	st->ldac_trigger_chn[chan->channel] = item;

	return 0;
}

static int ad5706r_ldac_trigger_chn_read(struct iio_dev *indio_dev,
					 const struct iio_chan_spec *chan)
{
	struct ad5706r_state *st = iio_priv(indio_dev);

	return st->ldac_trigger_chn[chan->channel];
}

static const struct iio_enum ad5706r_ldac_trigger_chn_enum = {
	.items = ldac_trigger_chn_iio_dev_attr_vals,
	.num_items = ARRAY_SIZE(ldac_trigger_chn_iio_dev_attr_vals),
	.set = ad5706r_ldac_trigger_chn_write,
	.get = ad5706r_ldac_trigger_chn_read,
};

static int ad5706r_toggle_trigger_chn_write(struct iio_dev *indio_dev,
					    const struct iio_chan_spec *chan,
					    unsigned int item)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	bool val_func_en = 0;
	bool val_func_mode = 0;
	bool val_sync_async = 0;
	bool val_hw_sw = 0;
	int ret;

	guard(mutex)(&st->lock);

	st->ldac_trigger_chn[chan->channel] = LDAC_TRIGGER_CHN_NONE;
	st->toggle_trigger_chn[chan->channel] = TOGGLE_TRIGGER_CHN_NONE;
	st->dither_trigger_chn[chan->channel] = DITHER_TRIGGER_CHN_NONE;

	if (item != TOGGLE_TRIGGER_CHN_NONE)
		val_func_en = 1;	/* Write 1 FUNC_EN */
	if (item == TOGGLE_TRIGGER_CHN_SW_TRIGGER)
		val_hw_sw = 1;		/* Write 1 LDAC_HW_SW for SW */

	ret = _set_reg_channel_mode(st, chan->channel, val_func_en, val_func_mode,
				    val_sync_async, val_hw_sw);
	if (ret)
		return ret;

	st->toggle_trigger_chn[chan->channel] = item;

	return 0;
}

static int ad5706r_toggle_trigger_chn_read(struct iio_dev *indio_dev,
					   const struct iio_chan_spec *chan)
{
	struct ad5706r_state *st = iio_priv(indio_dev);

	return st->toggle_trigger_chn[chan->channel];
}

static const struct iio_enum ad5706r_toggle_trigger_chn_enum = {
	.items = toggle_trigger_chn_iio_dev_attr_vals,
	.num_items = ARRAY_SIZE(toggle_trigger_chn_iio_dev_attr_vals),
	.set = ad5706r_toggle_trigger_chn_write,
	.get = ad5706r_toggle_trigger_chn_read,
};

static int ad5706r_dither_trigger_chn_write(struct iio_dev *indio_dev,
					    const struct iio_chan_spec *chan,
					    unsigned int item)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	bool val_func_en = 0;
	bool val_func_mode = 1;
	bool val_sync_async = 0;
	bool val_hw_sw = 0;
	int ret;

	guard(mutex)(&st->lock);

	st->ldac_trigger_chn[chan->channel] = LDAC_TRIGGER_CHN_NONE;
	st->toggle_trigger_chn[chan->channel] = TOGGLE_TRIGGER_CHN_NONE;
	st->dither_trigger_chn[chan->channel] = DITHER_TRIGGER_CHN_NONE;

	if (item != DITHER_TRIGGER_CHN_NONE)
		val_func_en = 1;	/* Write 1 FUNC_EN */
	if (item == DITHER_TRIGGER_CHN_SW_TRIGGER)
		val_hw_sw = 1;		/* Write 1 LDAC_HW_SW for SW */

	ret = _set_reg_channel_mode(st, chan->channel, val_func_en, val_func_mode,
				    val_sync_async, val_hw_sw);
	if (ret)
		return ret;

	st->dither_trigger_chn[chan->channel] = item;

	return 0;
}

static int ad5706r_dither_trigger_chn_read(struct iio_dev *indio_dev,
					   const struct iio_chan_spec *chan)
{
	struct ad5706r_state *st = iio_priv(indio_dev);

	return st->dither_trigger_chn[chan->channel];
}

static const struct iio_enum ad5706r_dither_trigger_chn_enum = {
	.items = dither_trigger_chn_iio_dev_attr_vals,
	.num_items = ARRAY_SIZE(dither_trigger_chn_iio_dev_attr_vals),
	.set = ad5706r_dither_trigger_chn_write,
	.get = ad5706r_dither_trigger_chn_read,
};

static int ad5706r_multi_dac_sel_ch_write(struct iio_dev *indio_dev,
					  const struct iio_chan_spec *chan,
					  unsigned int item)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	u16 reg_val;
	u16 mask_val;
	int ret;

	ret = ad5706r_spi_read(st, AD5706R_REG_MULTI_DAC_SEL_CH, &reg_val);
	if (ret)
		return ret;

	mask_val = BIT(chan->channel + st->shift_val);
	reg_val = ~mask_val & reg_val;

	if (item == MULTI_DAC_SEL_CH_EXCLUDE)
		ret = ad5706r_spi_write(st, AD5706R_REG_MULTI_DAC_SEL_CH,
					reg_val);
	else
		ret = ad5706r_spi_write(st, AD5706R_REG_MULTI_DAC_SEL_CH,
					reg_val | mask_val);

	if (ret)
		return ret;

	return 0;
}

static int ad5706r_multi_dac_sel_ch_read(struct iio_dev *indio_dev,
					 const struct iio_chan_spec *chan)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	u16 reg_val;
	u16 mask_val;
	int ret;

	ret = ad5706r_spi_read(st, AD5706R_REG_MULTI_DAC_SEL_CH, &reg_val);
	if (ret)
		return ret;

	mask_val = BIT(chan->channel + st->shift_val);
	reg_val = mask_val & reg_val;

	if (reg_val)
		st->multi_dac_sel_ch[chan->channel] = MULTI_DAC_SEL_CH_INCLUDE;
	else
		st->multi_dac_sel_ch[chan->channel] = MULTI_DAC_SEL_CH_EXCLUDE;

	return st->multi_dac_sel_ch[chan->channel];
}

static const struct iio_enum ad5706r_multi_dac_sel_ch_enum = {
	.items = multi_dac_sel_ch_iio_chan_attr_vals,
	.num_items = ARRAY_SIZE(multi_dac_sel_ch_iio_chan_attr_vals),
	.set = ad5706r_multi_dac_sel_ch_write,
	.get = ad5706r_multi_dac_sel_ch_read,
};

#define AD5706R_CHAN_EXT_INFO(_name, _what, _shared, _read, _write) {	\
	.name = _name,							\
	.read = (_read),						\
	.write = (_write),						\
	.private = (_what),						\
	.shared = (_shared),						\
}

static struct iio_chan_spec_ext_info ad5706r_ext_info[] = {
	/* device_attribute */
	AD5706R_CHAN_EXT_INFO("dev_addr", 0, IIO_SHARED_BY_ALL,
			      ad5706r_dev_addr_read, ad5706r_dev_addr_write),

	IIO_ENUM("addr_ascension", IIO_SHARED_BY_ALL, &ad5706r_addr_ascension_enum),
	IIO_ENUM_AVAILABLE("addr_ascension", IIO_SHARED_BY_ALL, &ad5706r_addr_ascension_enum),

	IIO_ENUM("single_instr", IIO_SHARED_BY_ALL, &ad5706r_single_instr_enum),
	IIO_ENUM_AVAILABLE("single_instr", IIO_SHARED_BY_ALL, &ad5706r_single_instr_enum),

	IIO_ENUM("hw_ldac_tg_state", IIO_SHARED_BY_ALL, &ad5706r_hw_ldac_tg_state_enum),
	IIO_ENUM_AVAILABLE("hw_ldac_tg_state", IIO_SHARED_BY_ALL, &ad5706r_hw_ldac_tg_state_enum),

	/* Sampling Frequency part of read/write RAW */

	IIO_ENUM("hw_ldac_tg_pwm", IIO_SHARED_BY_ALL, &ad5706r_hw_ldac_tg_pwm_enum),
	IIO_ENUM_AVAILABLE("hw_ldac_tg_pwm", IIO_SHARED_BY_ALL, &ad5706r_hw_ldac_tg_pwm_enum),

	IIO_ENUM("mux_out_sel", IIO_SHARED_BY_ALL, &ad5706r_mux_out_sel_enum),
	IIO_ENUM_AVAILABLE("mux_out_sel", IIO_SHARED_BY_ALL, &ad5706r_mux_out_sel_enum),

	AD5706R_CHAN_EXT_INFO("multi_dac_input_a", 0, IIO_SHARED_BY_ALL,
			      ad5706r_multi_dac_input_a_read, ad5706r_multi_dac_input_a_write),

	IIO_ENUM("multi_dac_sw_ldac_trigger", IIO_SHARED_BY_ALL,
		 &ad5706r_multi_dac_sw_ldac_trigger_enum),
	IIO_ENUM_AVAILABLE("multi_dac_sw_ldac_trigger", IIO_SHARED_BY_ALL,
			   &ad5706r_multi_dac_sw_ldac_trigger_enum),

	AD5706R_CHAN_EXT_INFO("reference_volts", 0, IIO_SHARED_BY_ALL,
			      ad5706r_reference_volts_read, ad5706r_reference_volts_write),

	IIO_ENUM("ref_select", IIO_SHARED_BY_ALL, &ad5706r_ref_select_enum),
	IIO_ENUM_AVAILABLE("ref_select", IIO_SHARED_BY_ALL, &ad5706r_ref_select_enum),

	IIO_ENUM("hw_shutdown_state", IIO_SHARED_BY_ALL, &ad5706r_hw_shutdown_state_enum),
	IIO_ENUM_AVAILABLE("hw_shutdown_state", IIO_SHARED_BY_ALL, &ad5706r_hw_shutdown_state_enum),

	/* Channel Attributes */
	AD5706R_CHAN_EXT_INFO("input_register_a", 0, IIO_SEPARATE,
			      ad5706r_input_register_a_read, ad5706r_input_register_a_write),

	AD5706R_CHAN_EXT_INFO("input_register_b", 0, IIO_SEPARATE,
			      ad5706r_input_register_b_read, ad5706r_input_register_b_write),

	IIO_ENUM("hw_active_edge", IIO_SEPARATE, &ad5706r_hw_active_edge_enum),
	IIO_ENUM_AVAILABLE("hw_active_edge", IIO_SEPARATE, &ad5706r_hw_active_edge_enum),

	IIO_ENUM("range_sel", IIO_SEPARATE, &ad5706r_range_sel_enum),
	IIO_ENUM_AVAILABLE("range_sel", IIO_SEPARATE, &ad5706r_range_sel_enum),

	IIO_ENUM("output_state", IIO_SEPARATE, &ad5706r_output_state_enum),
	IIO_ENUM_AVAILABLE("output_state", IIO_SEPARATE, &ad5706r_output_state_enum),

	IIO_ENUM("ldac_trigger_chn", IIO_SEPARATE, &ad5706r_ldac_trigger_chn_enum),
	IIO_ENUM_AVAILABLE("ldac_trigger_chn", IIO_SEPARATE, &ad5706r_ldac_trigger_chn_enum),

	IIO_ENUM("toggle_trigger_chn", IIO_SEPARATE, &ad5706r_toggle_trigger_chn_enum),
	IIO_ENUM_AVAILABLE("toggle_trigger_chn", IIO_SEPARATE, &ad5706r_toggle_trigger_chn_enum),

	IIO_ENUM("dither_trigger_chn", IIO_SEPARATE, &ad5706r_dither_trigger_chn_enum),
	IIO_ENUM_AVAILABLE("dither_trigger_chn", IIO_SEPARATE, &ad5706r_dither_trigger_chn_enum),

	IIO_ENUM("multi_dac_sel_ch", IIO_SEPARATE, &ad5706r_multi_dac_sel_ch_enum),
	IIO_ENUM_AVAILABLE("multi_dac_sel_ch", IIO_SEPARATE, &ad5706r_multi_dac_sel_ch_enum),

	{},
};

/* Channel */
static int ad5706r_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val,
			    int *val2,
			    long mask)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	u16 reg_val;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		scoped_guard(mutex, &st->lock) {
			ret = ad5706r_spi_read(st, AD5706R_REG_DAC_DATA_READBACK_CH(chan->channel),
					       &reg_val);

			if (ret)
				return ret;

			*val = reg_val;
		}
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		switch (st->range_sel[chan->channel]) {
		case RANGE_SEL_50:
			*val = 50 * HZ_PER_MHZ / AD5706R_DAC_MAX_CODE;
			break;
		case RANGE_SEL_150:
			*val = 150 * HZ_PER_MHZ / AD5706R_DAC_MAX_CODE;
			break;
		case RANGE_SEL_200:
			*val = 200 * HZ_PER_MHZ / AD5706R_DAC_MAX_CODE;
			break;
		case RANGE_SEL_300:
		default:
			*val = 300 * HZ_PER_MHZ / AD5706R_DAC_MAX_CODE;
			break;
		}
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_OFFSET:
		*val = 0;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = st->sampling_frequency;
		return IIO_VAL_INT;
	}

	return -EINVAL;
}

static int ad5706r_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val,
			     int val2,
			     long mask)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	struct pwm_state ldacb_pwm_state;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		/* Sets minimum and maximum frequency */
		val = clamp(val, SAMPLING_FREQUENCY_MIN_HZ, SAMPLING_FREQUENCY_MAX_HZ);

		scoped_guard(mutex, &st->lock) {
			pwm_get_state(st->ldacb_pwm, &ldacb_pwm_state);
			ldacb_pwm_state.duty_cycle = DIV_ROUND_CLOSEST_ULL(NANO, 2 * val);
			ldacb_pwm_state.period = DIV_ROUND_CLOSEST_ULL(NANO, val);
			ldacb_pwm_state.enabled = true;

			ret = pwm_apply_might_sleep(st->ldacb_pwm, &ldacb_pwm_state);
			if (ret)
				return ret;

			st->sampling_frequency = val;
		}
		return 0;
	}

	return -EINVAL;
}

static const struct iio_info ad5706r_info = {
	.read_raw = &ad5706r_read_raw,
	.write_raw = &ad5706r_write_raw,
	.debugfs_reg_access = &ad5706r_reg_access,
};

#define AD5706R_CHAN(_channel) {				\
	.type = IIO_CURRENT,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
			      BIT(IIO_CHAN_INFO_SCALE) |	\
			      BIT(IIO_CHAN_INFO_OFFSET),	\
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),\
	.output = 1,						\
	.indexed = 1,						\
	.channel = _channel,					\
	.ext_info = ad5706r_ext_info,				\
}

static const struct iio_chan_spec ad5706r_channels[] = {
	AD5706R_CHAN(0),
	AD5706R_CHAN(1),
	AD5706R_CHAN(2),
	AD5706R_CHAN(3),
};

static int _ad5706r_setup(struct ad5706r_state *st)
{
	struct pwm_state ldacb_pwm_state;
	struct device *dev = &st->spi->dev;
	int ret;
	int i;

	guard(mutex)(&st->lock);

	st->debug_streaming_len = 0;
	st->debug_streaming_data = 0;
	st->debug_streaming_addr = 0;
	st->debug_spi_speed_hz_write = 10000000;
	st->debug_spi_speed_hz_read = 10000000;

	st->dev_addr = 0x00;
	st->addr_ascension = ADDR_ASCENSION_DECREMENT;
	st->single_instr = SINGLE_INSTR_STREAMING;
	st->shift_val = 0;
	st->addr_desc = 1;
	st->hw_ldac_tg_state = HW_LDAC_TG_STATE_LOW;
	st->sampling_frequency = 1000000;
	st->hw_ldac_tg_pwm = HW_LDAC_TG_PWM_DISABLED;
	st->mux_out_sel = MUX_OUT_SEL_DISABLED;
	st->multi_dac_input_a = 0;
	st->reference_volts = 2500;
	st->ref_select = REF_SELECT_EXTERNAL;
	st->hw_shutdown_state = HW_SHUTDOWN_STATE_LOW;

	for (i = 0; i < 4; i++) {
		st->hw_active_edge[i] = HW_ACTIVE_EDGE_RISING_EDGE;
		st->range_sel[i] = RANGE_SEL_50;
		st->output_state[i] = OUTPUT_STATE_NORMAL_SW;
		st->ldac_trigger_chn[i] = LDAC_TRIGGER_CHN_HW_TRIGGER;
		st->toggle_trigger_chn[i] = TOGGLE_TRIGGER_CHN_HW_TRIGGER;
		st->dither_trigger_chn[i] = DITHER_TRIGGER_CHN_HW_TRIGGER;
		st->multi_dac_sel_ch[i] = MULTI_DAC_SEL_CH_EXCLUDE;
	}

	/* get spi_clk axi_clkgen, no enable as spi_engine driver enables it */
	st->reference_clk = devm_clk_get(dev, "spi_clk");
	if (IS_ERR(st->reference_clk))
		return dev_err_probe(dev, PTR_ERR(st->reference_clk),
				     "Failed to get AXI CLKGEN clock\n");

	st->ldacb_pwm = devm_pwm_get(dev, "ad5706r_ldacb");
	if (IS_ERR(st->ldacb_pwm))
		return dev_err_probe(dev, PTR_ERR(st->ldacb_pwm),
				     "Failed to get LDACB PWM\n");
	pwm_get_state(st->ldacb_pwm, &ldacb_pwm_state);
	ldacb_pwm_state.duty_cycle = 0;
	ldacb_pwm_state.period = DIV_ROUND_CLOSEST_ULL(NANO, st->sampling_frequency);
	ldacb_pwm_state.enabled = true;
	ret = pwm_apply_might_sleep(st->ldacb_pwm, &ldacb_pwm_state);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to apply PWM state\n");

	st->resetb_gpio = devm_gpiod_get_optional(dev, "dac-resetb", GPIOD_OUT_LOW);
	if (IS_ERR(st->resetb_gpio)) {
		return dev_err_probe(dev, PTR_ERR(st->resetb_gpio),
				     "Failed to get RESET_B GPIO\n");
	}

	st->shdn_gpio = devm_gpiod_get_optional(dev, "dac-shdn", GPIOD_OUT_HIGH);
	if (IS_ERR(st->shdn_gpio)) {
		return dev_err_probe(dev, PTR_ERR(st->shdn_gpio),
				     "Failed to get SHDN GPIO\n");
	}

	/*
	 * Get SPI max speed from device tree. Allows up to 100MHz.
	 * If value is taken from spi->max_speed_hz, it is capped at 25MHz.
	 */
	ret = device_property_read_u32(dev, "spi-max-frequency", &st->spi_max_speed_hz);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to set SPI Max Speed\n");

	st->spi_max_speed_hz = clamp(st->spi_max_speed_hz, SPI_MIN_SPEED_HZ, SPI_MAX_SPEED_HZ);

	return 0;
}

static int ad5706r_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct ad5706r_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	mutex_init(&st->lock);
	st->spi = spi;

	ret = _ad5706r_setup(st);
	if (ret)
		return ret;

	indio_dev->name = "ad5706r";
	indio_dev->info = &ad5706r_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ad5706r_channels;
	indio_dev->num_channels = ARRAY_SIZE(ad5706r_channels);

	ret = devm_iio_device_register(&spi->dev, indio_dev);
	if (ret)
		return ret;

	ad5706r_debugs_init(indio_dev);

	return 0;
}

static const struct of_device_id ad5706r_of_match[] = {
	{ .compatible = "adi,ad5706r" },
	{ },
};
MODULE_DEVICE_TABLE(of, ad5706r_of_match);

static const struct spi_device_id ad5706r_id[] = {
	{ "ad5706r", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, ad5706r_id);

static struct spi_driver ad5706r_driver = {
	.driver = {
		.name = "ad5706r",
		.of_match_table = ad5706r_of_match,
	},
	.probe = ad5706r_probe,
	.id_table = ad5706r_id,
};

module_spi_driver(ad5706r_driver);

MODULE_AUTHOR("Alexis Czezar Torreno <alexisczezar.torreno@analog.com>");
MODULE_DESCRIPTION("AD5706R Driver");
MODULE_LICENSE("GPL");
