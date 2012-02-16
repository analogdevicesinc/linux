/*
 * Copyright 2008 Michal Simek <monstr@monstr.eu>
 *
 * based on virtex.c file
 *
 * Copyright 2007 Secret Lab Technologies Ltd.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/init.h>
#include <linux/of_platform.h>
#include <asm/prom.h>
#include <asm/setup.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>

static struct of_device_id xilinx_of_bus_ids[] __initdata = {
	{ .compatible = "simple-bus", },
	{ .compatible = "xlnx,compound", },
	{}
};

static struct i2c_board_info __initdata xcomm_i2c_board_info[] = {
	{
		I2C_BOARD_INFO("spi-xcomm", 0x50),
	},
};

#if defined(CONFIG_AD9523) || defined(CONFIG_AD9523_MODULE)
#include "../../../../drivers/staging/iio/frequency/ad9523.h"

struct ad9523_channel_spec ad9523_channels[] = {
	{	/* ZD output */
		.channel_num = 0,
		.extended_name = "ZD_OUTPUT",
		.divider_output_invert_en = false,
		.sync_ignore_en = false,
		.low_power_mode_en = false,
		.driver_mode = LVDS_7mA,
		.divider_phase = 0,
		.channel_divider = 5,
		.use_alt_clock_src = false,
		.output_dis = true,
	},
	{	/* DAC CLK */
		.channel_num = 1,
		.extended_name = "DAC_CLK",
		.divider_output_invert_en = false,
		.sync_ignore_en = false,
		.low_power_mode_en = false,
		.driver_mode = LVPECL_8mA,
		.divider_phase = 0,
		.channel_divider = 2,
	},
	{	/* ADC CLK */
		.channel_num = 2,
		.extended_name = "ADC_CLK",
		.divider_output_invert_en = false,
		.sync_ignore_en = false,
		.low_power_mode_en = false,
		.driver_mode = LVDS_7mA,
		.divider_phase = 0,
		.channel_divider = 4,
	},
	{	/* DAC REF CLK */
		.channel_num = 4,
		.extended_name = "DAC_REF_CLK",
		.divider_output_invert_en = false,
		.sync_ignore_en = false,
		.low_power_mode_en = false,
		.driver_mode = LVDS_7mA,
		.divider_phase = 0,
		.channel_divider = 8,
	},
	{	/* TX LO REF */
		.channel_num = 5,
		.extended_name = "TX_LO_REF_CLK",
		.divider_output_invert_en = false,
		.sync_ignore_en = false,
		.low_power_mode_en = false,
		.driver_mode = CMOS_CONF3, /* HiZ on - */
		.divider_phase = 0,
		.channel_divider = 8,
	},
	{	/* DAC DCO */
		.channel_num = 6,
		.extended_name = "DAC_DCO_CLK",
		.divider_output_invert_en = false,
		.sync_ignore_en = false,
		.low_power_mode_en = false,
		.driver_mode = LVDS_7mA,
		.divider_phase = 0,
		.channel_divider = 2,
	},
	{	/* ADC SYNC */
		.channel_num = 8,
		.extended_name = "ADC_SYNC_CLK",
		.divider_output_invert_en = false,
		.sync_ignore_en = false,
		.low_power_mode_en = false,
		.driver_mode = CMOS_CONF3, /* HiZ on - */
		.divider_phase = 0,
		.channel_divider = 32,
		.output_dis = true,
	},
	{	/* RX LO REF */
		.channel_num = 9,
		.extended_name = "RX_LO_REF_CLK",
		.divider_output_invert_en = false,
		.sync_ignore_en = false,
		.low_power_mode_en = false,
		.driver_mode = CMOS_CONF3, /* HiZ on - */
		.divider_phase = 0,
		.channel_divider = 8,
	},
};
#if 0
struct ad9523_platform_data ad9523_pdata = {
	.vcxo_freq = 122880000,

	/* Single-Ended Input Configuration */
	.refa_diff_rcv_en = true,
	.refb_diff_rcv_en = true,
	.zd_in_diff_en = true,
	.osc_in_diff_en = false,

	.osc_in_cmos_neg_inp_en = true,

	.refa_r_div = 8,
	.refb_r_div = 8,
	.pll1_feedback_div = 32,
	.pll1_charge_pump_current_nA = 9000,
	.zero_delay_mode_internal_en = false,
	.osc_in_feedback_en = true,
	.pll1_loop_filter_rzero = 3,

	.ref_mode = 1,

	.pll2_charge_pump_current_nA = 420000,
	.pll2_ndiv_a_cnt = 0,
	.pll2_ndiv_b_cnt = 3,
	.pll2_freq_doubler_en = true,
	.pll2_r2_div = 1,
	.pll2_vco_diff_m1 = 3,
	.pll2_vco_diff_m2 = 3,

	.rpole2 = 0,
	.rzero = 2,
	.cpole1 = 2,
	.rzero_bypass_en = false,

	/* Output Channel Configuration */
	.num_channels = ARRAY_SIZE(ad9523_channels),
	.channels = ad9523_channels,
};
#endif
struct ad9523_platform_data ad9523_pdata = {
	.vcxo_freq = 122880000,

	/* Single-Ended Input Configuration */
	.refa_diff_rcv_en = true,
	.refb_diff_rcv_en = true,
	.zd_in_diff_en = false,
	.osc_in_diff_en = false,
	.osc_in_cmos_neg_inp_en = true,

	.refa_r_div = 0,
	.refb_r_div = 0,
	.pll1_feedback_div = 4,
	.pll1_charge_pump_current_nA = 6000,
	.zero_delay_mode_internal_en = true,
	.osc_in_feedback_en = true,
	.pll1_loop_filter_rzero = 0,

	.ref_mode = 1,

	.pll2_charge_pump_current_nA = 420000,
	.pll2_ndiv_a_cnt = 0,
	.pll2_ndiv_b_cnt = 3,
	.pll2_freq_doubler_en = true,
	.pll2_r2_div = 1,
	.pll2_vco_diff_m1 = 3,
	.pll2_vco_diff_m2 = 3,

	.rpole2 = 0,
	.rzero = 2,
	.cpole1 = 2,
	.rzero_bypass_en = false,

	/* Output Channel Configuration */
	.num_channels = ARRAY_SIZE(ad9523_channels),
	.channels = ad9523_channels,
};
#endif

#if defined(CONFIG_ADF4350) || defined(CONFIG_ADF4350_MODULE)
#include "../../../../drivers/staging/iio/frequency/adf4350.h"
static struct adf4350_platform_data adf4350_tx_pdata = {
	.name = "adf4351-tx",
	.clkin = 122880000,
	.channel_spacing = 10000,
	.r2_user_settings = ADF4350_REG2_PD_POLARITY_POS,
			    ADF4350_REG2_CHARGE_PUMP_CURR_uA(2500),
	.r3_user_settings = ADF4350_REG3_12BIT_CLKDIV_MODE(0),
	.r4_user_settings = ADF4350_REG4_OUTPUT_PWR(3) |
			    ADF4350_REG4_MUTE_TILL_LOCK_EN,
	.gpio_lock_detect = -1,
	.power_up_frequency = 2400000000,
};

static struct adf4350_platform_data adf4350_rx_pdata = {
	.name = "adf4351-rx",
	.clkin = 122880000,
	.channel_spacing = 10000,
	.r2_user_settings = ADF4350_REG2_PD_POLARITY_POS,
			    ADF4350_REG2_CHARGE_PUMP_CURR_uA(2500),
	.r3_user_settings = ADF4350_REG3_12BIT_CLKDIV_MODE(0),
	.r4_user_settings = ADF4350_REG4_OUTPUT_PWR(3) |
			    ADF4350_REG4_MUTE_TILL_LOCK_EN,
	.gpio_lock_detect = -1,
	.power_up_frequency = 2400000000,
};
#endif

static struct spi_board_info xcomm_spi_board_info[] __initdata = {
#if defined(CONFIG_AD9523) || defined(CONFIG_AD9523_MODULE)
	{
		.modalias = "ad9523",
		.max_speed_hz = 1000000,     /* max spi clock (SCK) speed in HZ */
		.bus_num = 0,
		.chip_select = 3,	/* GPIO controlled SSEL */
		.platform_data = &ad9523_pdata, /* No spi_driver specific config */
		.mode = SPI_MODE_0 | SPI_3WIRE,
	},
#endif
#if defined(CONFIG_AD8366) || defined(CONFIG_AD8366_MODULE)
	{
		.modalias = "ad8366",
		.max_speed_hz = 1000000,     /* max spi clock (SCK) speed in HZ */
		.bus_num = 0,
		.chip_select = 6,	/* GPIO controlled SSEL */
		.mode = SPI_MODE_0 | SPI_3WIRE,
	},
#endif
#if defined(CONFIG_ADF4350) || defined(CONFIG_ADF4350_MODULE)
	{
		.modalias = "adf4351",
		.max_speed_hz = 1000000,     /* max spi clock (SCK) speed in HZ */
		.bus_num = 0,
		.chip_select = 4,	/* GPIO controlled SSEL */
		.platform_data = &adf4350_rx_pdata, /* No spi_driver specific config */
		.mode = SPI_MODE_0,
	},
	{
		.modalias = "adf4351",
		.max_speed_hz = 1000000,     /* max spi clock (SCK) speed in HZ */
		.bus_num = 0,
		.chip_select = 5,	/* GPIO controlled SSEL */
		.platform_data = &adf4350_tx_pdata, /* No spi_driver specific config */
		.mode = SPI_MODE_0,
	},
#endif
#if defined(CONFIG_SPI_SPIDEV) || defined(CONFIG_SPI_SPIDEV_MODULE)
	{
		.modalias = "spidev",
		.max_speed_hz = 10000000,     /* max spi clock (SCK) speed in HZ */
		.bus_num = 0,
		.chip_select = 2, /* AD9548 */
	},
#if 0
{
		.modalias = "spidev",
		.max_speed_hz = 10000000,     /* max spi clock (SCK) speed in HZ */
		.bus_num = 0,
		.chip_select = 0, /* AD9122 */
	},
#endif
#endif
};

static int __init microblaze_device_probe(void)
{
	of_platform_bus_probe(NULL, xilinx_of_bus_ids, NULL);
	of_platform_reset_gpio_probe();

	i2c_register_board_info(1, xcomm_i2c_board_info,
				ARRAY_SIZE(xcomm_i2c_board_info));
	spi_register_board_info(xcomm_spi_board_info, ARRAY_SIZE(xcomm_spi_board_info));

	return 0;
}
device_initcall(microblaze_device_probe);
