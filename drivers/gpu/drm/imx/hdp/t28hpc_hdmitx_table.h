/******************************************************************************
 *
 * Copyright (C) 2015-2017 Cadence Design Systems, Inc.
 * All rights reserved worldwide.
 *
 * The material contained herein is the proprietary and confidential
 * information of Cadence or its licensors, and is supplied subject to, and may
 * be used only by Cadence's customer in accordance with a previously executed
 * license and maintenance agreement between Cadence and that customer.
 *
 * Copyright 2018 NXP
 *
 ******************************************************************************
 *
 * t28hpc_hdmitx_table.h
 *
 ******************************************************************************
 */

#ifndef T28HPC_HDMITX_TABLE_H_
#define T28HPC_HDMITX_TABLE_H_

#include <linux/io.h>

#define T28HPC_HDMITX_CLOCK_CONTROL_TABLE_ROWS_PIXEL_IN 22

#define T28HPC_HDMITX_PLL_TUNING_TABLE_ROWS_PIXEL_IN 38

#define T28HPC_HDMITX_CLOCK_CONTROL_TABLE_ROWS_PIXEL_OUT 47

#define T28HPC_HDMITX_PLL_TUNING_TABLE_ROWS_PIXEL_OUT 27

/* Table 6. HDMI TX clock control settings (pixel clock is input) */
typedef enum {
    T6_PIXEL_CLK_FREQ_KHZ_MIN,
    T6_PIXEL_CLK_FREQ_KHZ_MAX,
    T6_FEEDBACK_FACTOR,
    T6_DATA_RANGE_MBPS_MIN,
    T6_DATA_RANGE_MBPS_MAX,
    T6_CMNDA_PLL0_IP_DIV,
    T6_CMN_REF_CLK_DIG_DIV,
    T6_REF_CLK_DIVIDER_SCALER,
    T6_PLL_FB_DIV_TOTAL,
    T6_CMNDA_PLL0_FB_DIV_LOW,
    T6_CMNDA_PLL0_FB_DIV_HIGH,
    T6_VCO_FREQ_KHZ_MIN,
    T6_VCO_FREQ_KHZ_MAX,
    T6_VCO_RING_SELECT,
    T6_CMNDA_HS_CLK_0_SEL,
    T6_CMNDA_HS_CLK_1_SEL,
    T6_HSCLK_DIV_AT_XCVR,
    T6_HSCLK_DIV_TX_SUB_RATE,
    T6_TX_CLK_KHZ_MIN,
    T6_TX_CLK_KHZ_MAX,
    T6_CMNDA_PLL0_HS_SYM_DIV_SEL,
    T6_CMNDA_PLL0_CLK_FREQ_KHZ_MIN,
    T6_CMNDA_PLL0_CLK_FREQ_KHZ_MAX,
	T28HPC_HDMITX_CLOCK_CONTROL_TABLE_COLS_PIXEL_IN
} CLK_CTRL_PARAM_PIXEL_IN;


/* Table 7. HDMI TX PLL tuning settings (pixel clock is input) */
typedef enum {
    T7_VCO_FREQ_BIN,
    T7_PLL_VCO_FREQ_KHZ_MIN,
    T7_PLL_VCO_FREQ_KHZ_MAX,
    T7_VOLTAGE_TO_CURRENT_COARSE,
    T7_VOLTAGE_TO_CURRENT,
    T7_NDAC_CTRL,
    T7_PMOS_CTRL,
    T7_PTAT_NDAC_CTRL,
    T7_PLL_FEEDBACK_DIV_TOTAL,
    T7_CHARGE_PUMP_GAIN,
	T28HPC_HDMITX_PLL_TUNING_TABLE_COLS_PIXEL_IN
} PLL_TUNE_PARAM_PIXEL_IN;


/* Table 8. HDMI TX control settings (pixel clock is output) */
typedef enum {
    T8_PIXEL_CLK_FREQ_KHZ,
    T8_FEEDBACK_FACTOR,
    T8_DATA_RANGE_MBPS,
    T8_CMNDA_PLL0_IP_DIV,
    T8_CMN_REF_CLK_DIG_DIV,
    T8_REF_CLK_DIVIDER_SCALER,
    T8_PLL_FB_DIV_TOTAL,
    T8_CMNDA_PLL0_FB_DIV_LOW,
    T8_CMNDA_PLL0_FB_DIV_HIGH,
    T8_PIXEL_DIV_TOTAL,
    T8_CMNDA_PLL0_PXDIV_LOW,
    T8_CMNDA_PLL0_PXDIV_HIGH,
    T8_VCO_FREQ_KHZ,
    T8_VCO_RING_SELECT,
    T8_CMNDA_HS_CLK_0_SEL,
    T8_CMNDA_HS_CLK_1_SEL,
    T8_HSCLK_DIV_AT_XCVR,
    T8_HSCLK_DIV_TX_SUB_RATE,
    T8_TX_CLK_KHZ,
    T8_CMNDA_PLL0_HS_SYM_DIV_SEL,
    T8_CMNDA_PLL0_CLK_FREQ_KHZ,
    T8_PIXEL_CLK_OUTPUT_ENABLE,
	T28HPC_HDMITX_CLOCK_CONTROL_TABLE_COLS_PIXEL_OUT
} CLK_CTRL_PARAM_PIXEL_OUT;

/* Table 9. HDMI TX PLL tuning settings (pixel clock is output) */
typedef enum {
    T9_VCO_FREQ_BIN,
    T9_PLL_VCO_FREQ_KHZ_MIN,
    T9_PLL_VCO_FREQ_KHZ_MAX,
    T9_VOLTAGE_TO_CURRENT_COARSE,
    T9_VOLTAGE_TO_CURRENT,
    T9_NDAC_CTRL,
    T9_PMOS_CTRL,
    T9_PTAT_NDAC_CTRL,
    T9_PLL_FEEDBACK_DIV_TOTAL,
    T9_CHARGE_PUMP_GAIN,
    T9_COARSE_CODE,
    T9_V2I_CODE,
    T9_VCO_CAL_CODE,
	T28HPC_HDMITX_PLL_TUNING_TABLE_COLS_PIXEL_OUT
} PLL_TUNE_PARAM_PIXEL_OUT;

extern const u32 t28hpc_hdmitx_clock_control_table_pixel_in[T28HPC_HDMITX_CLOCK_CONTROL_TABLE_ROWS_PIXEL_IN][T28HPC_HDMITX_CLOCK_CONTROL_TABLE_COLS_PIXEL_IN];
extern const u32 t28hpc_hdmitx_pll_tuning_table_pixel_in[T28HPC_HDMITX_PLL_TUNING_TABLE_ROWS_PIXEL_IN][T28HPC_HDMITX_PLL_TUNING_TABLE_COLS_PIXEL_IN];
extern const u32 t28hpc_hdmitx_clock_control_table_pixel_out[T28HPC_HDMITX_CLOCK_CONTROL_TABLE_ROWS_PIXEL_OUT][T28HPC_HDMITX_CLOCK_CONTROL_TABLE_COLS_PIXEL_OUT];
extern const u32 t28hpc_hdmitx_pll_tuning_table_pixel_out[T28HPC_HDMITX_PLL_TUNING_TABLE_ROWS_PIXEL_OUT][T28HPC_HDMITX_PLL_TUNING_TABLE_COLS_PIXEL_OUT];

#endif

