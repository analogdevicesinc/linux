/* SPDX-License-Identifier: GPL-2.0-only OR BSD 2-Clause */
/*
 * This header provides macros for MAXIM MAX77675 device bindings.
 *
 * Copyright (c) 2025, Analog Device inc.
 * Author: Joan Na <joan.na@analog.com>
 */

#ifndef _DT_BINDINGS_REGULATOR_MAX77675_
#define _DT_BINDINGS_REGULATOR_MAX77675_

// Define the voltage limits for 12.5mV and 25mV steps
#define MAX77675_MIN_UV            500000    // 500 mV

#define MAX77675_MAX_UV_25MV       5500000   // 5.5V in microvolts for 25mV step
#define MAX77675_MAX_UV_12_5MV     3687500   // 3.6875V in microvolts for 12.5mV step

#define MAX77675_STEP_25MV         25000     // 25 mV
#define MAX77675_STEP_12_5MV       12500     // 12.5 mV

#define MAX77675_NUM_LEVELS_25MV   201
#define MAX77675_NUM_LEVELS_12_5MV 255

/* FPS source */
#define MAX77675_FPS_SLOT_0       0
#define MAX77675_FPS_SLOT_1       1
#define MAX77675_FPS_SLOT_2       2
#define MAX77675_FPS_SLOT_3       3
#define MAX77675_FPS_NONE         4
#define MAX77675_FPS_DEF          5

/* nEN Manual Reset Time Configuration (MRT) */
#define MAX77675_MRT_4S           0x0
#define MAX77675_MRT_8S           0x1
#define MAX77675_MRT_12S          0x2
#define MAX77675_MRT_16S          0x3

/* Internal Pull-Up Disable (PU_DIS) */
#define MAX77675_PU_EN            0x0
#define MAX77675_PU_DIS           0x1

/* Bias Low-Power Mode (BIAS_LPM) */
#define MAX77675_BIAS_NORMAL      0x0
#define MAX77675_BIAS_LPM_REQ     0x1

/* SIMO Internal Channel Disable (SIMO_INT_CH_DIS) */
#define MAX77675_SIMO_INT_NORMAL  0x0
#define MAX77675_SIMO_INT_LDO     0x1

/* nEN Mode Configuration */
#define MAX77675_EN_PUSH_BUTTON   0x0
#define MAX77675_EN_SLIDE_SWITCH  0x1
#define MAX77675_EN_LOGIC         0x2
#define MAX77675_EN_RESERVED      0x3

/* Debounce Timer Enable (DBEN_nEN) */
#define MAX77675_DBEN_100US       0x0
#define MAX77675_DBEN_30MS        0x1

/* Rising slew rate control for SBB0 when ramping up */
#define MAX77675_SR_2MV_PER_US    0x0  // 2 mV/탎
#define MAX77675_SR_USE_DVS       0x1  // Use DVS slew rate setting (maxim,dvs-slew-rate)

/* Dynamic Voltage Scaling (DVS) Slew Rate */
#define MAX77675_DVS_SLEW_5MV     0x0  // 5 mV/탎
#define MAX77675_DVS_SLEW_10MV    0x1  // 10 mV/탎

/* Latency Mode */
#define MAX77675_LAT_MODE_HIGH_LATENCY    0  // Low quiescent current, high latency (~100탎)
#define MAX77675_LAT_MODE_LOW_LATENCY     1   // High quiescent current, low latency (~10탎)

/* SIMO Buck-Boost Drive Strength (All Channels) */
#define MAX77675_DRV_SBB_FASTEST      0  // Fastest transition (~0.6 ns)
#define MAX77675_DRV_SBB_FAST         1  // Faster transition (~1.2 ns)
#define MAX77675_DRV_SBB_MEDIUM       2  // Moderate transition (~1.8 ns)
#define MAX77675_DRV_SBB_SLOWEST      3  // Slowest transition (~8 ns)

#endif
