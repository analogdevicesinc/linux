// SPDX-License-Identifier: GPL-2.0-only
/*
 * flexio driver
 *
 * Copyright 2023 NXP
 * Copyright 2021 NXP
 *
 */

#ifndef _IMX_FLEXIO_H_
#define _IMX_FLEXIO_H_

#include <linux/clk.h>
#include <linux/io.h>

#define CTRL            0x8
#define FLEXIO_CTRL_DOZEN_MASK          0x80000000
#define FLEXIO_CTRL_DOZEN(x)            (((u32)(((u32)(x)) << 31)) & 0x80000000)
#define FLEXIO_CTRL_DBGE_MASK           0x40000000
#define FLEXIO_CTRL_DBGE(x)             (((u32)(((u32)(x)) << 30)) & 0x40000000)
#define FLEXIO_CTRL_FASTACC_MASK        0x4
#define FLEXIO_CTRL_FASTACC(x)          (((u32)(((u32)(x)) << 2)) & 0x4)
#define FLEXIO_CTRL_SWRST_MASK          0x2
#define FLEXIO_CTRL_SWRST(x)            (((u32)(((u32)(x)) << 1)) & 0x2)
#define FLEXIO_CTRL_FLEXEN_MASK         0x1
#define FLEXIO_CTRL_FLEXEN(x)           (((u32)(((u32)(x)) << 0)) & 0x1)

#define PIN             0xc
#define SHIFTSTAT       0x10
#define SHIFTERR        0x14
#define TIMSTAT         0x18
#define SHIFTSIEN       0x20
#define SHIFTEIEN       0x24
#define PINSTAT         0x50
#define PINREN          0x58

#define SHIFTCTL_0      0x80
#define SHIFTCTL_1      0x84
#define SHIFTCTL_TIMSEL_MASK    0x7000000
#define SHIFTCTL_TIMSEL(x)      (((u32)(((u32)(x)) << 24)) & 0x7000000)
#define SHIFTCTL_TIMPOL_MASK    0x800000
#define SHIFTCTL_TIMPOL(x)      (((u32)(((u32)(x)) << 23)) & 0x800000)
#define SHIFTCTL_PINCFG_MASK    0x30000
#define SHIFTCTL_PINCFG(x)      (((u32)(((u32)(x)) << 16)) & 0x30000)
#define SHIFTCTL_PINSEL_MASK    0x1f00
#define SHIFTCTL_PINSEL(x)      (((u32)(((u32)(x)) << 8)) & 0x1f00)
#define SHIFTCTL_PINPOL_MASK    0x80
#define SHIFTCTL_PINPOL(x)      (((u32)(((u32)(x)) << 7)) & 0x80)
#define SHIFTCTL_SMOD_MASK      0x7
#define SHIFTCTL_SMOD(x)        (((u32)(((u32)(x)) << 0)) & 0x7)
#define SHIFT_ON_POSEDGE                0x0
#define SHIFT_ON_NEGEDGE                0x1
#define SHIFTER_PIN_OUTPUT_DISABLE      0x0
#define SHIFTER_PIN_OPEN_DRAIN_OUTPUT   0x1
#define SHIFTER_PIN_OUTPUT		0x3
#define PIN_ACTIVE_HIGH                 0x0
#define PIN_ACTIVE_LOW                  0x1
#define SHIFTER_DISABLE                 0x0
#define SHIFTER_RECEIVE                 0x1
#define SHIFTER_TRANSMIT                0x2

#define SHIFTCFG_0      0x100
#define SHIFTCFG_1      0x104
#define SHIFTCFG_INSRC_MASK      0x100
#define SHIFTCFG_INSRC(x)        (((u32)(((u32)(x)) << 8)) & 0x100)
#define SHIFTCFG_SSTOP_MASK      0x30
#define SHIFTCFG_SSTOP(x)        (((u32)(((u32)(x)) << 4)) & 0x30)
#define SHIFTCFG_SSTART_MASK     0x3
#define SHIFTCFG_SSTART(x)       (((u32)(((u32)(x)) << 0)) & 0x3)
#define INPUT_SRC_PIN           0x0
#define SSTOP_BIT_LOW           0x2
#define SSTOP_BIT_HIGH          0x3
#define SSTART_BIT_DISABLE      0X0
#define SSTART_BIT_LOW          0x2

#define SHIFTBUFBIS_1   0x284
#define SHIFTBUFBBS_0   0x380

#define TIMCTL_0        0x400
#define TIMCTL_1        0x404
#define TIMCTL_TRGSEL_MASK       0x3F000000
#define TIMCTL_TRGSEL(x)         (((u32)(((u32)(x)) << 24)) & 0x3F000000)
#define TIMCTL_TRGPOL_MASK       0x800000
#define TIMCTL_TRGPOL(x)         (((u32)(((u32)(x)) << 23)) & 0x800000)
#define TIMCTL_TRGSRC_MASK       0x400000
#define TIMCTL_TRGSRC(x)         (((u32)(((u32)(x)) << 22)) & 0x400000)
#define TIMCTL_PINCFG_MASK       0x30000
#define TIMCTL_PINCFG(x)         (((u32)(((u32)(x)) << 16)) & 0x30000)
#define TIMCTL_PINSEL_MASK       0x1f00
#define TIMCTL_PINSEL(x)         (((u32)(((u32)(x)) << 8)) & 0x1f00)
#define TIMCTL_PINPOL_MASK       0x80
#define TIMCTL_PINPOL(x)         (((u32)(((u32)(x)) << 7)) & 0x80)
#define TIMCTL_TIMOD_MASK        0x7
#define TIMCTL_TIMOD(x)          (((u32)(((u32)(x)) << 0)) & 0x7)
#define TIMER_TRGSEL_SHIFTER(x)         (((u32)(x) << 2U) | 0x1U)
#define TIMER_TRG_ACTIVE_LOW            0x1
#define TIMER_TRGSRC_INTER              0x1
#define TIMPIN_OUTPUT_DISABLE           0x0
#define TIMPIN_OPEN_DRAIN_OUTPUT        0x1
#define TIMPIN_BIDIR_OUTOUT		0x2
#define TIMPIN_OUTPUT			0x3
#define TIMPIN_ACTIVE_HIGH              0x0
#define TIMPIN_ACTIVE_LOW               0x1
#define TIMER_DISABLE                   0x0
#define DUAL_8BIT_COUNTERS_BAUD         0x1
#define SINGLE_16BIT_COUNTER            0x3

#define TIMCFG_0        0x480
#define TIMCFG_1        0x484
#define TIMCFG_TIMOUT_MASK      0x3000000
#define TIMCFG_TIMOUT(x)        (((u32)(((u32)(x)) << 24)) & 0x3000000)
#define TIMCFG_TIMDEC_MASK      0x700000
#define TIMCFG_TIMDEC(x)        (((u32)(((u32)(x)) << 20)) & 0x700000)
#define TIMCFG_TIMRST_MASK      0x70000
#define TIMCFG_TIMRST(x)        (((u32)(((u32)(x)) << 16)) & 0x70000)
#define TIMCFG_TIMDIS_MASK      0x7000
#define TIMCFG_TIMDIS(x)        (((u32)(((u32)(x)) << 12)) & 0x7000)
#define TIMCFG_TIMENA_MASK      0x700
#define TIMCFG_TIMENA(x)        (((u32)(((u32)(x)) << 8)) & 0x700)
#define TIMCFG_TSTOP_MASK       0x30
#define TIMCFG_TSTOP(x)         (((u32)(((u32)(x)) << 4)) & 0x30)
#define TIMCFG_TSTART_MASK      0x2
#define TIMCFG_TSTART(x)        (((u32)(((u32)(x)) << 1)) & 0x2)
#define TIMOUT_ONE_NOTAFFECT_BY_RESET   0x0
#define TIMOUT_ZERO_NOTAFFECT_BY_RESET  0x1
#define TIMDEC_FLEXIO_CLK               0x0
#define TIMDEC_PIN_INPUT                0x2
#define TIMRST_NEVER                    0x0
#define TIMRST_TIMPIN_EQUAL_TIMOUTPUT   0x2
#define TIMDIS_TIMER_DISABLE            0x1
#define TIMDIS_TIMER_COMPARE            0x2
#define TIMDIS_PIN_EDGE                 0x4
#define TIMENA_PREV_TIMENA              0X1
#define TIMENA_TRG_HIGH                 0x2
#define TSTOP_BIT_DISABLE               0x0
#define TSTOP_BIT_ENABLE_TIMCMP         0x1
#define TSTOP_BIT_ENABLE_TIMDIS         0x2
#define TSTART_BIT_DISABLE              0x0
#define TSTART_BIT_ENABLE               0x1

#define TIMCMP_0        0x500
#define TIMCMP_1        0x504

struct flexio_control {
	bool dozen;
	bool dbge;
	bool fastacc;
	bool swrst;
	bool flexen;
};

struct flexio_shifter_control {
	u8 timsel;
	u8 timpol;
	u8 pincfg;
	u8 pinsel;
	u8 pinpol;
	u8 smod;
};

struct flexio_shifter_config {
	u8 pwidth;
	u8 sszie;
	u8 latst;
	u8 insrc;
	u8 sstop;
	u8 sstart;
};

struct flexio_timer_control {
	u8 trgsel;
	u8 trgpol;
	u8 trgsrc;
	u8 pincfg;
	u8 pinsel;
	u8 pinpol;
	u8 timod;
};

struct flexio_timer_config {
	u8 timout;
	u8 timdec;
	u8 timrst;
	u8 timdis;
	u8 timena;
	u8 tstop;
	u8 tstart;
};

struct flexio_ddata {
	void __iomem *base;
	struct clk *per_clk;
	struct clk *ipg_clk;
	int irq;
};

/* flexio helper funcs */
void flexio_writel(void *base, u32 val, unsigned int reg);
u32 flexio_readl(void *base, unsigned int reg);
void flexio_sw_reset(void *base, unsigned int reg);
void flexio_get_default_ctrl(struct flexio_control *ctrl);
void flexio_setup_ctrl(void *base, struct flexio_control *ctrl,
		       unsigned int reg);
void flexio_setup_shiftctl(void *base, struct flexio_shifter_control *ctl,
			   unsigned int reg);
void flexio_setup_shiftcfg(void *base, struct flexio_shifter_config *cfg,
			   unsigned int reg);
void flexio_setup_timerctl(void *base, struct flexio_timer_control *ctl,
			   unsigned int reg);
void flexio_setup_timercfg(void *base, struct flexio_timer_config *cfg,
			   unsigned int reg);

#endif /* _IMX_FLEXIO_H_ */
