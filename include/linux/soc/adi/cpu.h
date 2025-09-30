/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 */

#ifndef __MACH_CPU_H
#define __MACH_CPU_H

#ifdef CONFIG_ARCH_SC57X
#define SYS_L2_START	0x20000000
#define SYS_SRAM_BASE	(0x20000000 + SZ_16K)
#else
#define SYS_L2_START	0x20080000
#define SYS_SRAM_BASE	(0x20080000 + SZ_16K)
#endif

#define SYS_SRAM_SIZE	(SZ_16K + SZ_32K * 3)
#define SYS_SRAM_ICC_SIZE   SZ_4K
#define SYS_MMR_BASE        0x31000000
#define SYS_MMR_SIZE        SZ_1M
#define SYS_SMC_BANK1       0x44000000

#define SC57X_GIC_PORT0     0x310B2000
#define SC57X_GIC_PORT1     0x310B4000

#define SC58X_GIC_PORT0     0x310B2000
#define SC58X_GIC_PORT1     0x310B4000

#define SC59X_GIC_PORT0     0x310B2000
#define SC59X_GIC_PORT1     0x310B4000

/*
 * Timer Configuration Register Bits
 */
#define TIMER_EMU_RUN       0x8000
#define TIMER_BPER_EN       0x4000
#define TIMER_BWID_EN       0x2000
#define TIMER_BDLY_EN       0x1000
#define TIMER_OUT_DIS       0x0800
#define TIMER_TIN_SEL       0x0400
#define TIMER_CLK_SEL       0x0300
#define TIMER_CLK_SCLK      0x0000
#define TIMER_CLK_ALT_CLK0  0x0100
#define TIMER_CLK_ALT_CLK1  0x0300
#define TIMER_PULSE_HI      0x0080
#define TIMER_SLAVE_TRIG    0x0040
#define TIMER_IRQ_MODE      0x0030
#define TIMER_IRQ_ACT_EDGE  0x0000
#define TIMER_IRQ_DLY       0x0010
#define TIMER_IRQ_WID_DLY   0x0020
#define TIMER_IRQ_PER       0x0030
#define TIMER_MODE          0x000f
#define TIMER_MODE_WDOG_P   0x0008
#define TIMER_MODE_WDOG_W   0x0009
#define TIMER_MODE_PWM_CONT 0x000c
#define TIMER_MODE_PWM      0x000d
#define TIMER_MODE_WDTH     0x000a
#define TIMER_MODE_WDTH_D   0x000b
#define TIMER_MODE_EXT_CLK  0x000e
#define TIMER_MODE_PININT   0x000f

#define __BFP(m) u16 m; u16 __pad_##m

struct gptimer3 {
	__BFP(config);
	u32 counter;
	u32 period;
	u32 width;
	u32 delay;
};

struct sc5xx_gptimer {
	int id;
	int irq;
	int reserved;
	int int_enable;
	void __iomem *io_base;
	void __iomem *cgu0_ctl;
	unsigned long isr_count;
	struct platform_device *pdev;
	struct list_head node;
};

struct gptimer3_group_regs {
	__BFP(run);
	__BFP(enable);
	__BFP(disable);
	__BFP(stop_cfg);
	__BFP(stop_cfg_set);
	__BFP(stop_cfg_clr);
	__BFP(data_imsk);
	__BFP(stat_imsk);
	__BFP(tr_msk);
	__BFP(tr_ie);
	__BFP(data_ilat);
	__BFP(stat_ilat);
	__BFP(err_status);
	__BFP(bcast_per);
	__BFP(bcast_wid);
	__BFP(bcast_dly);
};

/* The actual gptimer API */
struct sc5xx_gptimer *gptimer_request(int id);
int gptimer_free(struct sc5xx_gptimer *timer);
void set_gptimer_pwidth(struct sc5xx_gptimer *timer, uint32_t width);
void set_gptimer_period(struct sc5xx_gptimer *timer, uint32_t period);
uint32_t get_gptimer_count(struct sc5xx_gptimer *timer);
void set_gptimer_config(struct sc5xx_gptimer *timer, u16 config);
void enable_gptimers(u16 mask);
void disable_gptimers(u16 mask);
void map_gptimers(void);
u16 get_gptimer_status(void);
void set_gptimer_status(u16 value);
void set_spu_securep_msec(u16 n, bool msec);
void platform_ipi_init(void);

#endif				/* __MACH_CPU_H */
