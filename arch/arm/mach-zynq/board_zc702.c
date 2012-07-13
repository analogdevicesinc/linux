/*
 *  Copyright (C) 2011 Xilinx
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/of_platform.h>

#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/spi/eeprom.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/mtd/physmap.h>
#include <linux/spi/flash.h>
#include <linux/xilinx_devices.h>
#include <linux/i2c/pca954x.h>
#include <linux/i2c/pca953x.h>
#include <linux/i2c/si570.h>

#include <mach/slcr.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/hardware/gic.h>

#include "common.h"

#define IRQ_SPI1		81

#ifdef CONFIG_SPI_SPIDEV

static struct xspi_platform_data spi_0_pdata = {
	.speed_hz = 75000000,
	.bus_num = 0,
	.num_chipselect = 1
};

#endif

#ifdef CONFIG_MTD_M25P80

static struct mtd_partition spi_flash_partitions[] = {
	{
		.name		= "spi-flash",
		.size		= 0x100000,
		.offset		= 0,
	},
};

static struct flash_platform_data spi_flash_pdata = {
	.name			= "serial_flash",
	.parts			= spi_flash_partitions,
	.nr_parts		= ARRAY_SIZE(spi_flash_partitions),
	.type			= "sst25wf080"
};

#endif

#if defined(CONFIG_I2C_XILINX_PS) && defined(CONFIG_I2C_MUX_PCA954x)

static struct pca954x_platform_mode pca954x_platform_modes[] = {
	{
		.adap_id 		= 1,
		.deselect_on_exit	= 0,
	},
	{
		.adap_id 		= 2,
		.deselect_on_exit	= 0,
	},
	{
		.adap_id 		= 3,
		.deselect_on_exit	= 0,
	},
	{
		.adap_id 		= 4,
		.deselect_on_exit	= 0,
	},
	{
		.adap_id 		= 5,
		.deselect_on_exit	= 0,
	},
	{
		.adap_id 		= 6,
		.deselect_on_exit	= 0,
	},
	{
		.adap_id 		= 7,
		.deselect_on_exit	= 0,
	},
	{
		.adap_id 		= 8,
		.deselect_on_exit	= 0,
	},
};

static struct pca954x_platform_data pca954x_i2cmux_adap_data = {
	.modes 		= pca954x_platform_modes,
	.num_modes 	= 8,
};

static struct i2c_board_info __initdata pca954x_i2c_devices[] = {
	{
		I2C_BOARD_INFO("pca9548", 0x74),
		.platform_data = &pca954x_i2cmux_adap_data,
	},
};

#if defined(CONFIG_RTC_DRV_PCF8563)

static struct i2c_board_info __initdata rtc8564_board_info[] = {
	{
		I2C_BOARD_INFO("rtc8564", 0x51),
	},
};

#endif /*CONFIG_RTC_DRV_PCF8563 */

#if defined(CONFIG_GPIO_PCA953X)

static struct pca953x_platform_data tca6416_0 = {
	.gpio_base = 256,
};

static struct i2c_board_info __initdata tca6416_board_info[] = {
	{
		I2C_BOARD_INFO("tca6416", 0x21),
		.platform_data = &tca6416_0,
	}
};

#endif /* CONFIG_GPIO_PCF8563 */

#if defined(CONFIG_SI570)

/* Initial FOUT is set per the ADV7511 video clocking requirement */
static struct si570_platform_data si570_0 = {
	.factory_fout = 156250000LL,
	.initial_fout = 148500000,
};

static struct i2c_board_info __initdata si570_board_info[] = {
	{
		I2C_BOARD_INFO("si570", 0x5d),
		.platform_data = &si570_0,
	}
};

#endif /* CONFIG_SI570 */

#if defined(CONFIG_EEPROM_AT24)

static struct i2c_board_info __initdata m24c08_board_info[] = {
	{
		I2C_BOARD_INFO("24c08", 0x54),
	},
};

#endif /* CONFIG_EEPROM_AT24 */

#endif /* CONFIG_I2C_XILINX_PS && CONFIG_I2C_MUX_PCA954x */

#define SPIBUS_NUM_LPC		0

#if defined(CONFIG_AD9523) || defined(CONFIG_AD9523_MODULE)
#include <linux/iio/frequency/ad9523.h>
struct ad9523_channel_spec ad9523_channels[] = {
	{	/* ZD output */
		.channel_num = 0,
		.extended_name = "ZD_OUTPUT",
		.divider_output_invert_en = false,
		.sync_ignore_en = false,
		.low_power_mode_en = false,
		.driver_mode = LVDS_4mA,
		.divider_phase = 0,
		.channel_divider = 8,
		.use_alt_clock_src = false,
		.output_dis = false,
	},
	{	/* DAC CLK */
		.channel_num = 12,
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
		.channel_num = 10,
		.extended_name = "DAC_REF_CLK",
		.divider_output_invert_en = false,
		.sync_ignore_en = false,
		.low_power_mode_en = false,
		.driver_mode = LVDS_4mA,
		.divider_phase = 0,
		.channel_divider = 16,
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
		.channel_num = 7,
		.extended_name = "ADC_SYNC_CLK",
		.divider_output_invert_en = false,
		.sync_ignore_en = false,
		.low_power_mode_en = false,
		.driver_mode = CMOS_CONF3, /* HiZ on - */
		.divider_phase = 1,
		.channel_divider = 32,
		.output_dis = false,
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

struct ad9523_platform_data ad9523_pdata_lpc = {
	.vcxo_freq = 122880000,

	/* Single-Ended Input Configuration */
	.refa_diff_rcv_en = false,
	.refb_diff_rcv_en = true,
	.zd_in_diff_en = true,
	.osc_in_diff_en = false,
	.osc_in_cmos_neg_inp_en = true,

	.refa_r_div = 0,
	.refb_r_div = 0,
	.pll1_feedback_div = 4,
	.pll1_charge_pump_current_nA = 2000,
#if defined(CONFIG_ADIXCOMM_SYNC)
	.zero_delay_mode_internal_en = false,
#else
	.zero_delay_mode_internal_en = true,
#endif
	.osc_in_feedback_en = false,
	.refa_cmos_neg_inp_en = true,
	.pll1_loop_filter_rzero = 3,

#if defined(CONFIG_ADIXCOMM_SYNC)
	.ref_mode = 2, /* 3 ?*/
#else
	.ref_mode = 3, /* 3 ?*/
#endif

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
	.name = "ad9523-lpc"
};
#endif

#if defined(CONFIG_ADF4350) || defined(CONFIG_ADF4350_MODULE)
#include <linux/iio/frequency/adf4350.h>
static struct adf4350_platform_data adf4350_tx_pdata_lpc = {
	.name = "adf4351-tx-lpc",
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

static struct adf4350_platform_data adf4350_rx_pdata_lpc  = {
	.name = "adf4351-rx-lpc",
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

static struct spi_board_info __initdata xilinx_spipss_0_boardinfo[] = {
#if defined(CONFIG_AD9548) || defined(CONFIG_AD9548_MODULE)
	{
		.modalias = "ad9548",
		.max_speed_hz = 10000000,     /* max spi clock (SCK) speed in HZ */
		.bus_num = SPIBUS_NUM_LPC,
		.chip_select = 2, /* AD9548 */
		.mode = SPI_MODE_0 | SPI_3WIRE,
	},
#endif
#if defined(CONFIG_AD9523) || defined(CONFIG_AD9523_MODULE)
	{
		.modalias = "ad9523-1",
		.max_speed_hz = 1000000,     /* max spi clock (SCK) speed in HZ */
		.bus_num = SPIBUS_NUM_LPC,
		.chip_select = 3,	/* GPIO controlled SSEL */
		.platform_data = &ad9523_pdata_lpc, /* spi_driver specific config */
		.mode = SPI_MODE_0 | SPI_3WIRE,
	},
#endif
#if defined(CONFIG_AD8366) || defined(CONFIG_AD8366_MODULE)
	{
		.modalias = "ad8366",
		.max_speed_hz = 1000000,     /* max spi clock (SCK) speed in HZ */
		.bus_num = SPIBUS_NUM_LPC,
		.chip_select = 6,	/* GPIO controlled SSEL */
		.platform_data = "ad8366-lpc", /* spi_driver specific config */
		.mode = SPI_MODE_0 | SPI_3WIRE,
	},
#endif
#if defined(CONFIG_ADF4350) || defined(CONFIG_ADF4350_MODULE)
	{
		.modalias = "adf4351",
		.max_speed_hz = 1000000,     /* max spi clock (SCK) speed in HZ */
		.bus_num = SPIBUS_NUM_LPC,
		.chip_select = 4,	/* GPIO controlled SSEL */
		.platform_data = &adf4350_rx_pdata_lpc, /* No spi_driver specific config */
		.mode = SPI_MODE_0,
	},
	{
		.modalias = "adf4351",
		.max_speed_hz = 1000000,     /* max spi clock (SCK) speed in HZ */
		.bus_num = SPIBUS_NUM_LPC,
		.chip_select = 5,	/* GPIO controlled SSEL */
		.platform_data = &adf4350_tx_pdata_lpc, /* No spi_driver specific config */
		.mode = SPI_MODE_0,
	},
#endif
};

extern struct sys_timer xttcpss_sys_timer;

static void __init board_zc702_init(void)
{

	/* initialize the xilinx common code before the board
	 * specific
	 */
	xilinx_init_machine();

#if 	defined(CONFIG_SPI_SPIDEV) || defined(CONFIG_MTD_M25P80)
	spi_register_board_info(&xilinx_spipss_0_boardinfo[0],
		ARRAY_SIZE(xilinx_spipss_0_boardinfo));
#endif

#if 0
#if	defined(CONFIG_I2C_XILINX_PS) && defined(CONFIG_I2C_MUX_PCA954x)
	i2c_register_board_info(0, pca954x_i2c_devices,
				ARRAY_SIZE(pca954x_i2c_devices));

#if	defined(CONFIG_SI570)
	i2c_register_board_info(1, si570_board_info,
				ARRAY_SIZE(si570_board_info));
#endif

#if	defined(CONFIG_EEPROM_AT24)
	i2c_register_board_info(3, m24c08_board_info,
				ARRAY_SIZE(m24c08_board_info));
#endif

#if	defined(CONFIG_GPIO_PCA953X)
	i2c_register_board_info(4, tca6416_board_info,
				ARRAY_SIZE(tca6416_board_info));
#endif

#if	defined(CONFIG_RTC_DRV_PCF8563)
	i2c_register_board_info(5, rtc8564_board_info,
				ARRAY_SIZE(rtc8564_board_info));
#endif


#endif
#endif
}

static const char *xilinx_dt_match[] = {
	"xlnx,zynq-zc702",
	NULL
};

MACHINE_START(XILINX, "Xilinx Zynq Platform")
	.map_io		= xilinx_map_io,
	.init_irq	= xilinx_irq_init,
	.handle_irq	= gic_handle_irq,
	.init_machine	= board_zc702_init,
	.timer		= &xttcpss_sys_timer,
	.dt_compat	= xilinx_dt_match,
	.reserve	= xilinx_memory_init,
	.restart	= xilinx_system_reset,
MACHINE_END
