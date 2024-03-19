// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2024, Analog Devices Incorporated, All Rights Reserved
 */

#ifndef __PTP_ADI_H
#define __PTP_ADI_H

#include <linux/clk.h>
#include <linux/ktime.h>
#include <linux/platform_device.h>
#include <linux/ptp_clock_kernel.h>

#define ADRV906X_HW_TOD_CDC_DOMAIN_CNT                       (4u)
#define ADRV906X_HW_TOD_COUNTER_CNT                          (3u)
#define ADRV906X_HW_TOD_PPS_CNT                              (4u)

#define ADRV906X_HW_TOD_DISABLE                              (0)
#define ADRV906X_HW_TOD_ENABLE                               (1)

#define ADRV906X_HW_TOD_PPS_OUTPUT_OFF                       (0)
#define ADRV906X_HW_TOD_PPS_OUTPUT_ON                        (1)

#define ADRV906X_HW_TOD_PPS_IRQ_OFF                          (0)
#define ADRV906X_HW_TOD_PPS_IRQ_ON                           (1)

struct hw_tod;

enum adrv906x_hw_tod_trig_mode {
	HW_TOD_TRIG_MODE_GC	= 0,    /* ToD triggered by the Golden Counter */
	HW_TOD_TRIG_MODE_PPS	= 1,    /* ToD triggered by the PPS */
	HW_TOD_TRIG_MODE_CNT,
};

enum adrv906x_hw_tod_lc_clk_freq {
	HW_TOD_LC_100_P_000_M = 0,
	HW_TOD_LC_122_P_880_M,
	HW_TOD_LC_125_P_000_M,
	HW_TOD_LC_156_P_250_M,
	HW_TOD_LC_245_P_760_M,
	HW_TOD_LC_250_P_000_M,
	HW_TOD_LC_312_P_500_M,
	HW_TOD_LC_322_P_265_M,
	HW_TOD_LC_390_P_625_M,
	HW_TOD_LC_491_P_520_M,
	HW_TOD_LC_500_P_000_M,
	HW_TOD_LC_983_P_040_M,
	HW_TOD_LC_CLK_FREQ_CNT,
};

enum adrv906x_hw_tod_trig_op {
	HW_TOD_TRIG_OP_WR	= 0,            /* Trigger reading the ToD */
	HW_TOD_TRIG_OP_RD	= 1,            /* Trigger writing the ToD */
	HW_TOD_TRIG_OP_CNT
};

enum adrv906x_hw_tod_trig_set_flag {
	HW_TOD_TRIG_SET_FLAG_CLEAR	= 0,
	HW_TOD_TRIG_SET_FLAG_TRIG	= 1,
	HW_TOD_TRIG_SET_FALG_CNT
};

enum adrv906x_hw_tod_trig_op_flag {
	HW_TOD_TRIG_OP_FLAG_GOING	= 0,
	HW_TOD_TRIG_OP_FLAG_DONE	= 1,
	HW_TOD_TRIG_OP_FALG_CNT
};

enum adrv906x_hw_tod_pps_isr_source {
	HW_TOD_PPS_ISR_INTERNAL_0 = 0,  /* Internal generated PPS interrupt source 0 */
	HW_TOD_PPS_ISR_INTERNAL_1,      /* Internal generated PPS interrupt source 1 */
	HW_TOD_PPS_ISR_INTERNAL_GNSS,   /* Internal generated PPS interrupt source for GNSS */
	HW_TOD_PPS_ISR_EXTERNAL,        /* External input PPS interrupt source */
};

struct adrv906x_tod_tstamp {
	u16 frac_nanoseconds;
	u32 nanoseconds;
	u64 seconds;
};

struct adrv906x_tod_trig_delay {
	u64 ns;
	u32 rem_ns;                             /* remainder part of the clock tick in kHz */
};

struct adrv906x_tod_lc_clk_cfg {
	u32 freq_khz;                   /* frequency of the local clock */
	u32 ns_per_clk;                 /* nanosecond per clock */
	u32 frac_ns_per_clk;            /* fraction part of nanosecond per clock */
	u32 cnt_ctrl;                   /* correction control word */
};

struct adrv906x_tod_ppsx {
	u32 en;
	u32 delay_offset_ns;
	u32 pulse_width_ns;
};

struct adrv906x_tod_cdc {
	u32 delay_cnt[ADRV906X_HW_TOD_CDC_DOMAIN_CNT];
};

/* PTP Hardware Clock interface */
struct adrv906x_tod_counter {
	u8 id;
	u8 trigger_mode;                /* Trigger mode of ToD, 0 for GC, 1 for PPS */
	bool en;
	u64 trig_delay_tick;
	struct adrv906x_tod *parent;
	struct adrv906x_tod_ppsx ppsx;
	struct ptp_clock_info caps;
	struct ptp_clock *ptp_clk;
};

/* ADRV906X ToD module */
struct adrv906x_tod {
	struct device *dev;
	void __iomem *regs;
	u8 irq;
	u8 tod_counter_src;
	u32 lc_freq_khz;                /* Clock frequency for the ToD counter block */
	u32 gc_clk_freq_khz;            /* Clock frequency for the Golden counter block */
	struct adrv906x_tod_cdc cdc;
	struct adrv906x_tod_counter counter[ADRV906X_HW_TOD_COUNTER_CNT];
	struct clk *lc_clk;
	struct clk *gc_clk;
	void *priv_data;
	spinlock_t reg_lock;            /* Serialize access to hw_registers of the ToD module */
};

int adrv906x_tod_probe(struct platform_device *pdev);
int adrv906x_tod_remove(struct platform_device *pdev);
int adrv906x_tod_register_pll(struct ptp_clock_info *pll_caps);

#endif
